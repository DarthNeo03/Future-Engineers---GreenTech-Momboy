// ===========================================================================
// esp32_carro.ino — Controlador de hardware del carro WRO Future Engineers.
//
// El ESP32 ya NO tiene WiFi ni pagina web: es un controlador de hardware puro
// que obedece tramas binarias por serial. Toda la inteligencia (camara,
// navegacion, interfaz) vive en la Raspberry Pi. Motivos:
//   - una sola fuente de ordenes = no hay que arbitrar quien manda;
//   - ~40 KB de RAM y el nucleo 0 libres para atender el serial sin jitter;
//   - si la Pi se cuelga, el ESP32 para el carro solo (failsafe), que es
//     justamente la razon de tenerlo en medio.
//
// ENLACE: escucha las DOS bocas a la vez y contesta por la que recibio la
// ultima trama valida, asi el mismo binario sirve tanto si conectas por USB
// como si conectas TX/RX a los GPIO. No hay que recompilar para cambiar.
//   - Serial  (USB / UART0)
//   - Serial2 (GPIO16 = RX2, GPIO17 = TX2)  <- a GPIO14/15 de la Pi, cruzados
//
// TAREAS FreeRTOS
//   nucleo 0: tareaRx (prio 5)        lee UARTs, valida CRC, publica el mando
//             tareaTelemetria (2)     manda estado cada 50 ms
//   nucleo 1: tareaControl (4)        tick de 10 ms: rampas y escritura al HW
//             tareaVigilante (6)      si el control o la Pi se callan, corta
//
// SEGURIDAD: ver seguridad.h. Los topes del servo son constantes de
// compilacion y se aplican tres veces (al convertir, al rampar y al escribir).
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include "protocolo.h"
#include "seguridad.h"
#include "lineas.h"
#include "sensores_i2c.h"

// ======================= PINES =======================
// Puente H (tal cual tu montaje actual)
const int PIN_RPWM = 25;
const int PIN_LPWM = 26;
const int PIN_R_EN = 27;
const int PIN_L_EN = 33;

// Servo de direccion (MG996R)
const int PIN_SERVO = 32;

// UART hacia la Raspberry
const int PIN_RX2 = 16;   // <- TX de la Pi (GPIO14)
const int PIN_TX2 = 17;   // -> RX de la Pi (GPIO15)
const uint32_t BAUDIOS = 115200;

// Sensores I2C (los dos en el mismo bus: 0x68 el MPU, 0x29 el TCS)
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const int PIN_INT_MPU = 34;   // solo entrada, perfecto para una interrupcion
const int PIN_INT_TCS = 35;   // idem. Pon -1 si no lo cableas
const bool USAR_INT_MPU = true;

// Si el carro avanza al reves, cambia esto a 1 en vez de recablear.
#define INVERTIR_MOTOR 0

// ======================= PWM =======================
const int MOTOR_PWM_FREQ = 20000;   // 20 kHz: fuera del rango audible
const int MOTOR_PWM_RES  = 8;       // 0-255

const int SERVO_PWM_FREQ = 50;
const int SERVO_PWM_RES  = 16;
const int SERVO_PULSO_MIN_US = 500;
const int SERVO_PULSO_MAX_US = 2400;

// ======================= TIEMPOS =======================
const uint32_t TICK_CONTROL_MS = 10;    // 100 Hz
const uint32_t PERIODO_TELE_MS = 50;    // 20 Hz
const uint32_t FAILSAFE_MS     = 300;   // silencio tolerado de la Pi
const uint32_t VIGILANTE_MS    = 200;   // silencio tolerado del propio control
const uint32_t PERIODO_IMU_MS  = 5;     // 200 Hz de integracion del rumbo
const uint32_t ENVIO_IMU_MS    = 20;    // 50 Hz de publicacion a la Pi
const float    ENVIO_IMU_GRADOS = 0.4f; // ...o antes si el rumbo se mueve mas
const uint32_t PERIODO_COLOR_MS = 16;   // ~60 Hz de muestreo del color
const uint32_t PERIODO_SENSORES_MS = 2000;
const uint32_t REINTENTO_SENSORES_MS = 3000;  // si falta un chip, se vuelve a buscar
const uint32_t ESPERA_ARRANQUE_I2C_MS = 250;  // los chips tardan en despertar

const uint8_t VERSION_FIRMWARE = 3;

// ======================= ESTADO COMPARTIDO =======================
QueueHandle_t colaMando = NULL;         // longitud 1, el nuevo pisa al viejo

volatile uint32_t msUltimoMando   = 0;
volatile uint32_t msUltimoControl = 0;
volatile uint8_t  ultimaSeq       = 0;
volatile uint8_t  estadoBits      = 0;
volatile uint8_t  pwmActual       = 0;
volatile uint8_t  anguloActual    = 100;
volatile uint32_t tramasMalas     = 0;
volatile int8_t   enlaceActivo    = -1;  // 0 = USB, 1 = GPIO, -1 = ninguno aun

seg::ControlServo servo;
seg::ControlMotor motor;

MPU6050 mpu;
TCS34725 tcs;
lineas::Detector detectorLinea;
volatile uint8_t ordenSensores = 0;      // AUX_* pendientes de atender
volatile uint8_t sensoresPresentes = 0;
volatile uint8_t hzImu = 0, hzColor = 0;
volatile bool intMpu = false;

void IRAM_ATTR isrMpu() { intMpu = true; }

HardwareSerial *enlaces[2] = { &Serial, &Serial2 };
proto::Lector lectores[2];

// ======================= PROTOTIPOS =======================
void enviarTrama(uint8_t tipo, const uint8_t *payload, uint8_t n);
void enviarLog(const char *txt);
void enviarComandoParada();
void tareaRx(void *);
void tareaControl(void *);
void tareaVigilante(void *);
void tareaTelemetria(void *);
void tareaSensores(void *);
void buscarSensores(bool avisar);

// ======================= HARDWARE =======================
static inline void escribirServoHW(int angulo) {
  // Cuarta red: aunque llegue aqui un valor imposible, no sale del rango.
  angulo = seg::lim(angulo, seg::SERVO_TOPE_MIN, seg::SERVO_TOPE_MAX);
  int pulsoUs = map(angulo, 0, 180, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  uint32_t duty = (uint32_t)((pulsoUs / 20000.0) * 65535.0);
  ledcWrite(PIN_SERVO, duty);
  anguloActual = (uint8_t)angulo;
}

static inline void escribirMotorHW(int pwmFirmado) {
#if INVERTIR_MOTOR
  pwmFirmado = -pwmFirmado;
#endif
  if (pwmFirmado > 0) {
    ledcWrite(PIN_LPWM, 0);
    ledcWrite(PIN_RPWM, pwmFirmado);
  } else if (pwmFirmado < 0) {
    ledcWrite(PIN_RPWM, 0);
    ledcWrite(PIN_LPWM, -pwmFirmado);
  } else {
    ledcWrite(PIN_RPWM, 0);
    ledcWrite(PIN_LPWM, 0);
  }
  pwmActual = (uint8_t)abs(pwmFirmado);
}

static inline void pararMotorHW() {
  ledcWrite(PIN_RPWM, 0);
  ledcWrite(PIN_LPWM, 0);
  pwmActual = 0;
}

// ======================= ENVIO =======================
void enviarTrama(uint8_t tipo, const uint8_t *payload, uint8_t n) {
  uint8_t buf[5 + proto::MAX_PAYLOAD];
  uint8_t total = proto::empaquetar(tipo, payload, n, buf);
  if (!total) return;
  int8_t e = enlaceActivo;
  if (e < 0) {                     // nadie ha hablado aun: por las dos
    enlaces[0]->write(buf, total);
    enlaces[1]->write(buf, total);
  } else {
    enlaces[e]->write(buf, total);
  }
}

// Fuerza el failsafe: la tarea de control ve el silencio y para el motor.
// Se usa antes de calibrar el giroscopo, que exige el carro completamente
// quieto y tarda algo mas de un segundo.
void enviarComandoParada() {
  msUltimoMando = millis() - FAILSAFE_MS - 1;
}

void enviarLog(const char *txt) {
  uint8_t n = 0;
  while (txt[n] && n < proto::MAX_PAYLOAD) n++;
  enviarTrama(proto::TIPO_LOG, (const uint8_t *)txt, n);
}

// ======================= TAREA: RECEPCION =======================
void tareaRx(void *) {
  for (;;) {
    for (int e = 0; e < 2; e++) {
      while (enlaces[e]->available() > 0) {
        lectores[e].alimentar((uint8_t)enlaces[e]->read());

        while (lectores[e].siguiente()) {
        enlaceActivo = e;
        const uint8_t tipo = lectores[e].tipo();

        if (tipo == proto::TIPO_MANDO) {
          proto::Mando m;
          if (proto::decodificarMando(lectores[e].payload(), lectores[e].len(), m)) {
            xQueueOverwrite(colaMando, &m);       // el mas reciente gana
            msUltimoMando = millis();
            ultimaSeq = m.seq;
            if (m.limpiar()) {
              tramasMalas = 0;
              lectores[0].crcMalos = lectores[1].crcMalos = 0;
            }
            if (m.aux) ordenSensores |= m.aux;   // cero de yaw, calibraciones
          }
        } else if (tipo == proto::TIPO_PING) {
          uint8_t eco = lectores[e].len() ? lectores[e].payload()[0] : 0;
          enviarTrama(proto::TIPO_PONG, &eco, 1);
        } else if (tipo == proto::TIPO_CONFIG && lectores[e].len() >= 6) {
          const uint8_t *p = lectores[e].payload();
          seg::ConfigServo cs;
          cs.centro       = p[0];
          cs.izquierda    = p[1];
          cs.derecha      = p[2];
          cs.gradosPorSeg = p[4] * 10;
          servo.configurar(cs);            // configurar() recorta a los topes
          seg::ConfigMotor cm;
          cm.rampaPorTick = p[3];
          motor.configurar(cm);
          enviarLog("CFG OK");
        }
        }  // while siguiente()
      }
    }
    tramasMalas = lectores[0].crcMalos + lectores[1].crcMalos;
    vTaskDelay(1);                          // 1 ms
  }
}

// ======================= TAREA: CONTROL (100 Hz) =======================
void tareaControl(void *) {
  proto::Mando m;
  m.seq = 0; m.flags = 0; m.vel = 0; m.dir = 0; m.vmax = 0; m.aux = 0;

  TickType_t ultimo = xTaskGetTickCount();
  uint32_t tAnterior = millis();

  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(TICK_CONTROL_MS));

    proto::Mando nuevo;
    if (xQueueReceive(colaMando, &nuevo, 0) == pdPASS) m = nuevo;

    const uint32_t ahora = millis();
    const uint32_t dt = ahora - tAnterior;
    tAnterior = ahora;
    msUltimoControl = ahora;

    const bool silencio = (ahora - msUltimoMando) > FAILSAFE_MS;
    const bool frenar = silencio || m.parada() || !m.armado();

    // ---- Servo -----------------------------------------------------------
    // Ojo: el servo se sigue atendiendo aunque el motor este parado. Si la Pi
    // se calla, el servo va al centro (el carro se detiene recto, no torcido).
    int objetivo;
    if (silencio || m.centrar()) objetivo = servo.config().centro;
    else                         objetivo = servo.anguloDesdePorcentaje(m.dir);
    escribirServoHW(servo.paso(objetivo, dt ? dt : TICK_CONTROL_MS));

    // ---- Motor -----------------------------------------------------------
    int pedido = frenar ? 0 : seg::pwmDesdePorcentaje(m.vel, m.vmax);
    if (silencio || m.parada()) {
      motor.cortar(ahora);            // sin rampa: parada dura
      pararMotorHW();
    } else {
      escribirMotorHW(motor.paso(pedido, ahora));
    }

    // ---- Estado para la telemetria --------------------------------------
    uint8_t bits = 0;
    if (m.armado())               bits |= proto::E_ARMADO;
    if (motor.actual() != 0)      bits |= proto::E_MOTOR;
    if (silencio)                 bits |= proto::E_FAILSAFE;
    if (servo.saturado())         bits |= proto::E_SERVO_TOPE;
    if (motor.inversionBloqueada()) bits |= proto::E_INV_BLOQUEADA;
    estadoBits = bits;
  }
}

// ======================= TAREA: VIGILANTE =======================
// Prioridad maxima y casi sin trabajo: si la tarea de control se atasca (bug,
// bloqueo, cola llena), este corta la traccion directamente sobre el hardware.
// Es la unica tarea que puede escribir el PWM del motor sin pasar por control.
void tareaVigilante(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(20));
    const uint32_t ahora = millis();
    if ((ahora - msUltimoControl) > VIGILANTE_MS) {
      pararMotorHW();
      estadoBits |= proto::E_FAILSAFE;
    }
  }
}

// ======================= BUSQUEDA DE SENSORES =======================
// Se llama al arrancar, cada pocos segundos si falta alguno, y cuando la Pi lo
// pide desde la interfaz. Solo sondea los que NO estan, asi que reintentar no
// molesta a los que ya funcionan.
void buscarSensores(bool avisar) {
  if (!(sensoresPresentes & proto::S_MPU)) {
    if (mpu.iniciar(Wire, USAR_INT_MPU, PIN_INT_MPU)) {
      sensoresPresentes |= proto::S_MPU;
      if (USAR_INT_MPU && PIN_INT_MPU >= 0)
        attachInterrupt(digitalPinToInterrupt(PIN_INT_MPU), isrMpu, RISING);
      enviarLog("MPU6050 OK");
    } else if (avisar) {
      enviarLog("sin MPU6050");
    }
  }
  if (!(sensoresPresentes & proto::S_TCS)) {
    if (tcs.iniciar(Wire, PIN_INT_TCS)) {
      sensoresPresentes |= proto::S_TCS;
      lineas::Config lc;
      detectorLinea.configurar(lc);
      enviarLog("TCS34725 OK");
    } else if (avisar) {
      enviarLog("sin TCS34725");
    }
  }
}

// ======================= TAREA: SENSORES =======================
// Vive en el nucleo 1 pero con prioridad BAJA: si hay que elegir entre leer el
// giroscopo y atender el lazo de control, gana el control. Un rumbo que llega
// 5 ms tarde no rompe nada; un PWM que llega 5 ms tarde, si.
//
// Filosofia de envio: datos ya masticados y solo cuando importan.
//   - el rumbo se integra aqui a 200 Hz y se publica a 50 Hz (o antes si se
//     movio mas de 0.4 grados). La Pi recibe grados, no muestras crudas.
//   - el color solo se manda CUANDO CAMBIA: cruzar una linea son dos tramas
//     de 11 bytes, no un chorro continuo.
void tareaSensores(void *) {
  uint32_t tAnterior = millis();
  uint32_t tEnvioImu = 0, tColor = 0, tEstado = 0, tHz = 0, tReintento = 0;
  uint16_t cuentaImu = 0, cuentaColor = 0;
  float yawEnviado = 0.0f;
  bool calibrandoImu = false;

  for (;;) {
    // ---- ordenes que llegaron desde la Pi ----------------------------
    uint8_t ordenes = ordenSensores;
    if (ordenes) {
      ordenSensores = 0;
      if ((ordenes & proto::AUX_CERO_YAW) && mpu.presente) {
        mpu.cero();
        enviarLog("YAW=0");
      }
      if ((ordenes & proto::AUX_CALIB_IMU) && mpu.presente) {
        calibrandoImu = true;
        enviarLog("CALIB IMU...");
        // Bloquea esta tarea ~1.2 s, pero NO el control ni el serial: son
        // tareas distintas. Aun asi el motor se para por seguridad.
        enviarComandoParada();
        bool ok = mpu.calibrar(400);
        calibrandoImu = false;
        enviarLog(ok ? "CALIB IMU OK" : "CALIB IMU MAL");
      }
      if ((ordenes & proto::AUX_CALIB_COLOR) && tcs.presente) {
        detectorLinea.calibrarBlanco();
        enviarLog("BLANCO OK");
      }
      if (ordenes & proto::AUX_REINIT_SENSORES) {
        enviarLog("BUSCANDO I2C");
        buscarSensores(true);
      }
    }

    const uint32_t ahora = millis();

    // ---- rumbo --------------------------------------------------------
    if (mpu.presente && !calibrandoImu) {
      const float dt = (ahora - tAnterior) / 1000.0f;
      if (dt > 0.0f && mpu.actualizar(dt > 0.2f ? 0.2f : dt)) cuentaImu++;
      tAnterior = ahora;
      const bool porTiempo = (ahora - tEnvioImu) >= ENVIO_IMU_MS;
      const bool porCambio = fabsf(mpu.yaw - yawEnviado) >= ENVIO_IMU_GRADOS;
      if (porTiempo || porCambio) {
        tEnvioImu = ahora;
        yawEnviado = mpu.yaw;
        uint8_t buf[5 + proto::MAX_PAYLOAD];
        uint8_t n = proto::empaquetarIMU(mpu.yaw, mpu.giroZ, mpu.calibrado,
                                         (uint8_t)mpu.temp, buf);
        int8_t e = enlaceActivo;
        if (e >= 0) enlaces[e]->write(buf, n);
      }
    } else {
      tAnterior = ahora;
    }

    // ---- color del suelo ----------------------------------------------
    if (tcs.presente && (ahora - tColor) >= PERIODO_COLOR_MS) {
      tColor = ahora;
      // Si el pin INT esta cableado y activo, el sensor ya avisa de que el
      // color se salio de la ventana; leemos igual porque necesitamos valores.
      if (tcs.leerColor()) {
        cuentaColor++;
        if (detectorLinea.actualizar(tcs.r, tcs.g, tcs.b, tcs.c)) {
          uint8_t buf[5 + proto::MAX_PAYLOAD];
          uint8_t n = proto::empaquetarColor(
              detectorLinea.estado(), detectorLinea.r(), detectorLinea.g(),
              detectorLinea.b(), detectorLinea.luzComprimida(), buf);
          int8_t e = enlaceActivo;
          if (e >= 0) enlaces[e]->write(buf, n);
        }
      }
    }

    // ---- reintento automatico si falta algun chip ----------------------
    // No hace falta pulsar nada: si el MPU o el TCS no estaban al arrancar
    // (tipico cuando comparten alimentacion con el motor y despiertan tarde),
    // se les vuelve a sondear cada 3 s hasta que aparezcan. Cuesta dos
    // transacciones I2C, nada.
    if (sensoresPresentes != (proto::S_MPU | proto::S_TCS) &&
        (ahora - tReintento) >= REINTENTO_SENSORES_MS) {
      tReintento = ahora;
      buscarSensores(false);
    }

    // ---- que sensores hay, cada 2 s -----------------------------------
    if ((ahora - tEstado) >= PERIODO_SENSORES_MS) {
      tEstado = ahora;
      uint8_t buf[5 + proto::MAX_PAYLOAD];
      uint8_t n = proto::empaquetarSensores(sensoresPresentes, hzImu, hzColor, buf);
      int8_t e = enlaceActivo;
      if (e >= 0) enlaces[e]->write(buf, n);
    }
    if ((ahora - tHz) >= 1000) {
      tHz = ahora;
      hzImu = (uint8_t)(cuentaImu > 255 ? 255 : cuentaImu);
      hzColor = (uint8_t)(cuentaColor > 255 ? 255 : cuentaColor);
      cuentaImu = cuentaColor = 0;
    }

    // Si hay pin INT del MPU y ya llego el flanco, no esperamos el tick entero
    if (USAR_INT_MPU && intMpu) {
      intMpu = false;
      vTaskDelay(1);
    } else {
      vTaskDelay(pdMS_TO_TICKS(PERIODO_IMU_MS));
    }
  }
}

// ======================= TAREA: TELEMETRIA =======================
void tareaTelemetria(void *) {
  TickType_t ultimo = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(PERIODO_TELE_MS));
    proto::Telemetria t;
    t.seq_eco = ultimaSeq;
    t.estado = estadoBits;
    t.pwm = pwmActual;
    t.angulo = anguloActual;
    uint32_t edad = millis() - msUltimoMando;
    t.ms_desde_mando = (edad > 65535) ? 65535 : (uint16_t)edad;
    t.tramas_malas = (tramasMalas > 255) ? 255 : (uint8_t)tramasMalas;
    t.version = VERSION_FIRMWARE;

    uint8_t buf[5 + proto::MAX_PAYLOAD];
    uint8_t n = proto::empaquetarTelemetria(t, buf);
    int8_t e = enlaceActivo;
    if (e >= 0) enlaces[e]->write(buf, n);
  }
}

// ======================= SETUP =======================
void setup() {
  // --- Puente H ---
  pinMode(PIN_L_EN, OUTPUT); digitalWrite(PIN_L_EN, HIGH);
  pinMode(PIN_R_EN, OUTPUT); digitalWrite(PIN_R_EN, HIGH);
  ledcAttach(PIN_RPWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttach(PIN_LPWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  pararMotorHW();

  // --- Servo: al centro antes que nada ---
  ledcAttach(PIN_SERVO, SERVO_PWM_FREQ, SERVO_PWM_RES);
  seg::ConfigServo cs;                 // valores medidos en tu mecanismo
  cs.centro = 100;
  cs.izquierda = 65;
  cs.derecha = 135;
  cs.gradosPorSeg = 320;
  servo.configurar(cs);
  servo.forzarCentro();
  escribirServoHW(servo.config().centro);

  seg::ConfigMotor cm;
  cm.rampaPorTick = 10;                // 0 -> 255 en ~260 ms
  cm.msFrenoAntesDeInvertir = 150;
  motor.configurar(cm);

  // --- Enlaces ---
  Serial.begin(BAUDIOS);
  Serial2.begin(BAUDIOS, SERIAL_8N1, PIN_RX2, PIN_TX2);

  // --- Sensores I2C: si no estan, no pasa nada -----------------------------
  // Se espera un poco antes de sondear: el MPU6050 necesita ~100 ms desde que
  // le llega tension y el TCS34725 otro tanto. Sin esta espera el ESP32 los
  // sondeaba antes de que estuvieran despiertos y los daba por ausentes. Y si
  // aun asi no aparecen, la tarea de sensores los reintenta sola cada 3 s.
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  delay(ESPERA_ARRANQUE_I2C_MS);
  buscarSensores(false);

  colaMando = xQueueCreate(1, sizeof(proto::Mando));
  if (colaMando == NULL) {
    // Sin cola no hay control posible: mejor quedarse parado y gritando.
    for (;;) { pararMotorHW(); Serial.println("FATAL: sin cola"); delay(1000); }
  }

  msUltimoMando = millis() - FAILSAFE_MS - 1;   // arranca en failsafe, a proposito
  msUltimoControl = millis();

  xTaskCreatePinnedToCore(tareaRx,          "Rx",       4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(tareaTelemetria,  "Tele",     3072, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(tareaControl,     "Control",  4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(tareaVigilante,   "Vigilante",2048, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(tareaSensores,    "Sensores", 4096, NULL, 2, NULL, 1);

  enviarLog("ESP32 listo");
  if (sensoresPresentes & proto::S_MPU) enviarLog("MPU6050 OK");
  if (sensoresPresentes & proto::S_TCS) enviarLog("TCS34725 OK");
  if (!sensoresPresentes) enviarLog("sin sensores I2C");
}

// Nada que hacer aqui: todo vive en las tareas. Se deja dormir para no
// robarle tiempo al planificador.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
