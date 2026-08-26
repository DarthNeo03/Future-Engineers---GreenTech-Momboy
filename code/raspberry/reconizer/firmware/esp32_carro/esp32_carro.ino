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
//             tareaSensores (3)       MPU6050 y TCS34725 a 100 Hz
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
#include "sensores.h"

// ======================= PINES =======================
// Puente H (tal cual tu montaje actual)
const int PIN_RPWM = 25;
const int PIN_LPWM = 26;
const int PIN_R_EN = 27;
const int PIN_L_EN = 33;

// Servo de direccion (MG996R)
const int PIN_SERVO = 32;

// I2C de los sensores: MPU6050 (0x68) y TCS34725 (0x29) en el mismo bus.
// Van aqui y no en la Pi porque el sensor de color necesita un muestreo
// deterministico -a 0,4 m/s una linea de 20 mm dura 50 ms- y porque estan
// fisicamente pegados al ESP32. Ver la cabecera de sensores.h.
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
const uint32_t TICK_CONTROL_MS = 10;    // 100 Hz
const uint32_t PERIODO_TELE_MS = 50;    // 20 Hz
const uint32_t FAILSAFE_MS     = 300;   // silencio tolerado de la Pi
const uint32_t VIGILANTE_MS    = 200;   // silencio tolerado del propio control

const uint32_t PERIODO_SENSORES_MS = 10;   // 100 Hz

const uint8_t VERSION_FIRMWARE = 3;   // 3 = telemetria con yaw y lineas

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

// --- sensores --------------------------------------------------------------
sens::Yaw yaw;
sens::DetectorLinea linea;
volatile bool imu_ok = false;
volatile bool piso_ok = false;
volatile bool pedir_cal_imu = false;
uint8_t mpu_dir = 0;

// Perfiles de color. El indice 0 tiene que ser el BLANCO: el detector de
// linea usa ese convenio para saber cuando NO esta sobre una linea.
// Se miden con tools/calibrar_piso.py en la Pi y se copian aqui.
sens::PerfilColor perfiles[3] = {
  {0.33f, 0.34f, 0.33f, 0.05f},   // 0 blanco
  {0.55f, 0.30f, 0.15f, 0.09f},   // 1 naranja
  {0.20f, 0.32f, 0.48f, 0.09f},   // 2 azul
};
const uint16_t CLEAR_MIN = 60;
const uint16_t CLEAR_MAX = 65000;

HardwareSerial *enlaces[2] = { &Serial, &Serial2 };
proto::Lector lectores[2];

// ======================= I2C =======================
static bool i2cEscribir(uint8_t dir, uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(dir);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

static bool i2cLeer(uint8_t dir, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(dir);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)dir, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// --- MPU6050 ---------------------------------------------------------------
static bool iniciarMPU() {
  const uint8_t dirs[2] = {sens::MPU_DIR_A, sens::MPU_DIR_B};
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t id = 0;
    if (!i2cLeer(dirs[k], sens::MPU_WHO_AM_I, &id, 1)) continue;
    if (id != 0x68 && id != 0x70 && id != 0x71 && id != 0x73) continue;
    mpu_dir = dirs[k];
    i2cEscribir(mpu_dir, sens::MPU_PWR_MGMT_1, 0x80);   // reset
    delay(60);
    i2cEscribir(mpu_dir, sens::MPU_PWR_MGMT_1, 0x01);   // reloj del giro X
    i2cEscribir(mpu_dir, sens::MPU_CONFIG, 0x03);       // filtro 44 Hz
    i2cEscribir(mpu_dir, sens::MPU_SMPLRT_DIV, 0x04);   // 200 Hz
    i2cEscribir(mpu_dir, sens::MPU_GYRO_CONFIG, 0x00);  // +-250 dps
    delay(20);
    return true;
  }
  return false;
}

static bool leerGiroZ(int16_t &gz) {
  uint8_t b[2];
  if (!i2cLeer(mpu_dir, sens::MPU_GYRO_ZOUT_H, b, 2)) return false;
  gz = (int16_t)((b[0] << 8) | b[1]);
  return true;
}

// --- TCS34725 --------------------------------------------------------------
static bool iniciarTCS() {
  uint8_t id = 0;
  if (!i2cLeer(sens::TCS_DIR, sens::TCS_CMD | sens::TCS_ID, &id, 1)) return false;
  if (id != 0x44 && id != 0x4D) return false;
  i2cEscribir(sens::TCS_DIR, sens::TCS_CMD | sens::TCS_ENABLE, sens::TCS_PON);
  delay(3);
  // ATIME 0xF6 = 24 ms. ES EL PARAMETRO QUE DECIDE SI SE VE LA LINEA: con los
  // 154 o 700 ms que traen por defecto muchas librerias, una linea de 20 mm a
  // 0,4 m/s se promedia con el piso blanco y no aparece nunca.
  i2cEscribir(sens::TCS_DIR, sens::TCS_CMD | sens::TCS_ATIME, 0xF6);
  i2cEscribir(sens::TCS_DIR, sens::TCS_CMD | sens::TCS_CONTROL, 0x01);  // x4
  i2cEscribir(sens::TCS_DIR, sens::TCS_CMD | sens::TCS_ENABLE,
              sens::TCS_PON | sens::TCS_AEN);
  delay(30);
  return true;
}

static bool leerTCS(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b) {
  uint8_t d[8];
  if (!i2cLeer(sens::TCS_DIR, sens::TCS_CMD_AUTO | sens::TCS_CDATAL, d, 8))
    return false;
  c = (uint16_t)(d[0] | (d[1] << 8));
  r = (uint16_t)(d[2] | (d[3] << 8));
  g = (uint16_t)(d[4] | (d[5] << 8));
  b = (uint16_t)(d[6] | (d[7] << 8));
  return true;
}

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
              linea.reiniciar();
            }
            if (m.flags & proto::F_CAL_IMU) pedir_cal_imu = true;
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

// ======================= TAREA: SENSORES (100 Hz) =======================
// Nucleo 0, prioridad baja: si alguna vez compite con la recepcion del UART,
// que pierda esta. Perder una muestra de yaw no se nota; perder una trama de
// mando si.
//
// Si un sensor deja de responder se marca como caido y se sigue: el resto del
// programa ya sabe funcionar sin giroscopio y sin sensor de color.
void tareaSensores(void *) {
  TickType_t ultimo = xTaskGetTickCount();
  uint32_t t_prev = millis();
  uint8_t fallos_mpu = 0, fallos_tcs = 0;

  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(PERIODO_SENSORES_MS));
    const uint32_t ahora = millis();
    const float dt = (ahora - t_prev) / 1000.0f;
    t_prev = ahora;

    if (pedir_cal_imu) {
      pedir_cal_imu = false;
      yaw.empezarCalibracion(200);       // 2 s a 100 Hz, con el carro QUIETO
    }

    // --- giroscopio ---
    if (imu_ok) {
      int16_t gz = 0;
      if (leerGiroZ(gz)) {
        fallos_mpu = 0;
        yaw.paso(gz, dt > 0.0f ? dt : PERIODO_SENSORES_MS / 1000.0f);
      } else if (++fallos_mpu > 20) {
        imu_ok = false;
      }
    }

    // --- color del piso ---
    if (piso_ok) {
      uint16_t c, r, g, b;
      if (leerTCS(c, r, g, b)) {
        fallos_tcs = 0;
        const uint8_t idx = sens::clasificar(c, r, g, b, perfiles, 3,
                                             CLEAR_MIN, CLEAR_MAX);
        linea.paso(idx, ahora);
      } else if (++fallos_tcs > 20) {
        piso_ok = false;
      }
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
    t.yaw_dg = yaw.decigrados();
    t.sensores = (uint8_t)((imu_ok ? proto::S_IMU_OK : 0) |
                           (piso_ok ? proto::S_PISO_OK : 0) |
                           (yaw.calibrando() ? proto::S_IMU_CAL : 0));
    t.lineas = linea.contador();
    t.color_linea = linea.ultimoColor();

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

  // --- Sensores I2C ---
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  imu_ok = iniciarMPU();
  piso_ok = iniciarTCS();
  if (imu_ok) yaw.empezarCalibracion(200);   // el carro tiene que estar quieto

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
  xTaskCreatePinnedToCore(tareaControl,     "Control",  4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(tareaVigilante,   "Vigilante",2048, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(tareaSensores,    "Sensores", 3072, NULL, 3, NULL, 0);

  enviarLog(imu_ok ? (piso_ok ? "listo imu+piso" : "listo imu")
                   : (piso_ok ? "listo piso" : "listo sin sensores"));
}

// Nada que hacer aqui: todo vive en las tareas. Se deja dormir para no
// robarle tiempo al planificador.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
