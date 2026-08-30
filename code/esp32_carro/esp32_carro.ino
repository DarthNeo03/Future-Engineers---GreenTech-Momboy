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
//             tareaSensores (3)       MPU6050 (yaw) + TCS34725 (lineas) por I2C
//             tareaTelemetria (2)     estado a 20 Hz + sensores a 40 Hz
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
#include "sensores_i2c.h"
#include "lineas.h"

// ======================= PINES =======================
// Puente H (tal cual tu montaje actual)
const int PIN_RPWM = 25;
const int PIN_LPWM = 26;
const int PIN_R_EN = 27;
const int PIN_L_EN = 33;

// Servo de direccion (MG996R)
const int PIN_SERVO = 32;

// I2C compartido: MPU6050 (0x68/0x69) + TCS34725 (0x29)
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const uint32_t I2C_HZ = 400000;

// UART hacia la Raspberry
const int PIN_RX2 = 16;   // <- TX de la Pi (GPIO14)
const int PIN_TX2 = 17;   // -> RX de la Pi (GPIO15)
const uint32_t BAUDIOS = 115200;

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
const uint32_t TICK_CONTROL_MS  = 10;   // 100 Hz
const uint32_t PERIODO_TELE_MS  = 50;   // 20 Hz (estado del motor/servo)
const uint32_t PERIODO_SENS_MS  = 25;   // 40 Hz (yaw + color, si hay sensores)
const uint32_t FAILSAFE_MS      = 300;  // silencio tolerado de la Pi
const uint32_t VIGILANTE_MS     = 200;  // silencio tolerado del propio control
const uint32_t REINTENTO_I2C_MS = 3000; // sondear sensores que falten

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

// --- sensores I2C (los toca solo tareaSensores; la copia publicada se
//     protege con un spinlock porque telemetria la lee desde otro nucleo) ---
sens::Mpu6050 mpu;
sens::Tcs34725 tcs;
lin::Clasificador clasificador;
proto::Sensores sensoresPub = {0, 0, 0, 0, 0, 0, 0, 0};
portMUX_TYPE muxSensores = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t calPendiente = 0;       // proto::CAL_* pedido por la Pi
volatile bool cfgTcsPendiente = false;
proto::CfgTcs cfgTcsNueva;

HardwareSerial *enlaces[2] = { &Serial, &Serial2 };
proto::Lector lectores[2];

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
          }
        } else if (tipo == proto::TIPO_PING) {
          uint8_t eco = lectores[e].len() ? lectores[e].payload()[0] : 0;
          enviarTrama(proto::TIPO_PONG, &eco, 1);
        } else if (tipo == proto::TIPO_CFG_TCS) {
          proto::CfgTcs c;
          if (proto::decodificarCfgTcs(lectores[e].payload(), lectores[e].len(), c)) {
            cfgTcsNueva = c;
            cfgTcsPendiente = true;     // la aplica tareaSensores (toca el I2C)
          }
        } else if (tipo == proto::TIPO_CMD_CAL && lectores[e].len() >= 1) {
          calPendiente = lectores[e].payload()[0];
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

// ======================= TAREA: SENSORES (I2C) =======================
// Nucleo 0, prioridad baja: leer el MPU a ~200 Hz e integrar el yaw, leer el
// TCS a su ritmo de integracion y clasificar la linea. Publica una copia
// para telemetria. Si un sensor no esta, se reintenta cada 3 s sin bloquear.
void tareaSensores(void *) {
  vTaskDelay(pdMS_TO_TICKS(300));   // dejar despertar a los chips (leccion vieja)
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire.setTimeOut(5);               // un sensor colgado no congela la tarea

  uint32_t msSondeo = millis() - REINTENTO_I2C_MS;   // sondear ya mismo
  uint32_t msTcs = 0;
  TickType_t ultimo = xTaskGetTickCount();
  uint32_t msPrev = millis();

  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(5));   // 200 Hz
    const uint32_t ahora = millis();

    // --- redeteccion de sensores ausentes -----------------------------
    if ((!mpu.presente || !tcs.presente) &&
        (ahora - msSondeo >= REINTENTO_I2C_MS || calPendiente == proto::CAL_REDETECTAR)) {
      msSondeo = ahora;
      if (!mpu.presente && mpu.detectar()) enviarLog("MPU6050 OK");
      if (!tcs.presente && tcs.detectar()) enviarLog("TCS34725 OK");
      if (calPendiente == proto::CAL_REDETECTAR) calPendiente = 0;
    }

    // --- ordenes de calibracion ---------------------------------------
    if (calPendiente == proto::CAL_GIRO) {
      calPendiente = 0;
      enviarLog("CAL GIRO...");
      // publicar "calibrando" antes de bloquear ~1 s
      portENTER_CRITICAL(&muxSensores);
      sensoresPub.estado |= proto::S_CALIBRANDO;
      portEXIT_CRITICAL(&muxSensores);
      mpu.calibrar();
      enviarLog("CAL GIRO OK");
    } else if (calPendiente == proto::CAL_CERO_YAW) {
      calPendiente = 0;
      mpu.ceroYaw();
    }

    if (cfgTcsPendiente) {
      cfgTcsPendiente = false;
      lin::Config lc;
      lc.c_min = cfgTcsNueva.c_min;
      lc.naranja_r_min = cfgTcsNueva.naranja_r_min;
      lc.naranja_b_max = cfgTcsNueva.naranja_b_max;
      lc.azul_b_min = cfgTcsNueva.azul_b_min;
      lc.azul_r_max = cfgTcsNueva.azul_r_max;
      lc.muestras_min = cfgTcsNueva.muestras_min;
      lc.refractario_ms = (uint16_t)cfgTcsNueva.refractario_ds * 100;
      clasificador.configurar(lc);
      if (tcs.presente) tcs.configurar(cfgTcsNueva.atime, cfgTcsNueva.gain);
      enviarLog("CFG TCS OK");
    }

    // --- giroscopio ----------------------------------------------------
    const uint32_t dt = ahora - msPrev;
    msPrev = ahora;
    if (mpu.presente) mpu.paso(dt ? dt : 5);

    // --- color ---------------------------------------------------------
    if (tcs.presente && (ahora - msTcs) >= tcs.periodoMs()) {
      msTcs = ahora;
      if (tcs.leerColor()) {
        clasificador.paso(tcs.c, tcs.r, tcs.g, tcs.b, ahora);
      }
    }

    // --- publicar ------------------------------------------------------
    proto::Sensores s;
    s.yaw_deci = (int16_t)(mpu.yaw * 10.0f);
    s.gz_deci = (int16_t)(mpu.gz * 10.0f);
    s.c = tcs.c; s.r = tcs.r; s.g = tcs.g; s.b = tcs.b;
    s.estado = 0;
    if (mpu.presente) s.estado |= proto::S_MPU_OK;
    if (tcs.presente) s.estado |= proto::S_TCS_OK;
    if (mpu.calibrando) s.estado |= proto::S_CALIBRANDO;
    if (clasificador.sobreLinea()) s.estado |= proto::S_SOBRE_LINEA;
    s.estado |= (uint8_t)(clasificador.clase() << 6);
    s.cnt_lineas = clasificador.contadores();
    portENTER_CRITICAL(&muxSensores);
    sensoresPub = s;
    portEXIT_CRITICAL(&muxSensores);
  }
}

// ======================= TAREA: TELEMETRIA =======================
void tareaTelemetria(void *) {
  TickType_t ultimo = xTaskGetTickCount();
  uint32_t msTele = 0;
  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(PERIODO_SENS_MS));
    const uint32_t ahora = millis();
    int8_t e = enlaceActivo;
    if (e < 0) continue;

    // Sensores a 40 Hz (el yaw fresco es lo que mas ayuda a la navegacion)
    proto::Sensores s;
    portENTER_CRITICAL(&muxSensores);
    s = sensoresPub;
    portEXIT_CRITICAL(&muxSensores);
    uint8_t buf[5 + proto::MAX_PAYLOAD];
    uint8_t n = proto::empaquetarSensores(s, buf);
    enlaces[e]->write(buf, n);

    // Estado del motor/servo a 20 Hz
    if (ahora - msTele >= PERIODO_TELE_MS) {
      msTele = ahora;
      proto::Telemetria t;
      t.seq_eco = ultimaSeq;
      t.estado = estadoBits;
      t.pwm = pwmActual;
      t.angulo = anguloActual;
      uint32_t edad = ahora - msUltimoMando;
      t.ms_desde_mando = (edad > 65535) ? 65535 : (uint16_t)edad;
      t.tramas_malas = (tramasMalas > 255) ? 255 : (uint8_t)tramasMalas;
      t.version = VERSION_FIRMWARE;
      n = proto::empaquetarTelemetria(t, buf);
      enlaces[e]->write(buf, n);
    }
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

  colaMando = xQueueCreate(1, sizeof(proto::Mando));
  if (colaMando == NULL) {
    // Sin cola no hay control posible: mejor quedarse parado y gritando.
    for (;;) { pararMotorHW(); Serial.println("FATAL: sin cola"); delay(1000); }
  }

  msUltimoMando = millis() - FAILSAFE_MS - 1;   // arranca en failsafe, a proposito
  msUltimoControl = millis();

  xTaskCreatePinnedToCore(tareaRx,          "Rx",       4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(tareaTelemetria,  "Tele",     3072, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(tareaSensores,    "Sensores", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(tareaControl,     "Control",  4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(tareaVigilante,   "Vigilante",2048, NULL, 6, NULL, 1);

  enviarLog("ESP32 listo");
}

// Nada que hacer aqui: todo vive en las tareas. Se deja dormir para no
// robarle tiempo al planificador.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
