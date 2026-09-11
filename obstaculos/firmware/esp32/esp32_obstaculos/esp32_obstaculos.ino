// ===========================================================================
// esp32_obstaculos.ino — Controlador de hardware del carro WRO Future
// Engineers, RETO DE OBSTACULOS.
//
// El ESP32 no tiene WiFi, ni pagina web, ni decisiones. Es un controlador de
// hardware puro que obedece tramas binarias por serial. Toda la inteligencia
// (camara, planificacion, maquina de estados) vive en la Raspberry Pi 5.
// Razones, en orden de importancia:
//
//   1. UNA sola fuente de ordenes: no hay que arbitrar quien manda.
//   2. Si la Pi se cuelga, el ESP32 para el carro EL SOLO. Esa es justamente
//      la razon de tener un microcontrolador en medio y no un puente H
//      colgado del GPIO de la Pi.
//   3. Los ~40 KB de RAM y el nucleo 0 quedan libres para atender el serial
//      sin jitter, que es lo que hace que el lazo de control sea de verdad
//      de 100 Hz y no "100 Hz cuando Linux quiere".
//
// ENLACE: escucha las DOS bocas a la vez y contesta por la que recibio la
// ultima trama valida. El mismo binario sirve por USB (banco de pruebas) y
// por GPIO (carrera), sin recompilar.
//     Serial   -> USB / UART0
//     Serial2  -> GPIO16 = RX2, GPIO17 = TX2, cruzados contra la Pi
//
// REPARTO DE TAREAS FreeRTOS
//   nucleo 0: tareaRx (prio 5)         lee UARTs, valida CRC, publica mando
//             tareaSensores (prio 3)   MPU-6050 + TCS34725, despertada por
//                                      las patas INT (latido de 5 ms)
//             tareaTelemetria (prio 2) estado a 20 Hz, sensores a 40 Hz
//   nucleo 1: tareaControl (prio 4)    tick de 10 ms: boton, rampas, HW
//             tareaVigilante (prio 6)  si el control o la Pi se callan, corta
//
// Nucleo 1 solo tiene control y vigilante a proposito: nada que pueda
// bloquear (I2C, serial) comparte nucleo con lo que mueve el motor.
//
// REQUISITO: Arduino-ESP32 core 3.x (API ledcAttach / ledcWrite por pin).
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>

#include "protocolo.h"
#include "seguridad.h"
#include "sensores_i2c.h"
#include "lineas.h"
#include "panel.h"

// ============================== PINES =====================================
// Traccion: driver IBT-2 (BTS7960), motor DC de 500 rpm.
const int PIN_RPWM = 25;    // PWM adelante
const int PIN_LPWM = 26;    // PWM atras
const int PIN_R_EN = 27;    // habilitacion de la media puente derecha
const int PIN_L_EN = 33;    // habilitacion de la media puente izquierda

// Direccion: servo MG996R sobre cremallera Ackermann.
const int PIN_SERVO = 32;

// I2C compartido: MPU-6050 (0x68) + TCS34725 (0x29).
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const uint32_t I2C_HZ = 400000;

// Patas INT de los sensores. Las dos son OPCIONALES: si no estan cableadas el
// firmware lo detecta solo y sigue por sondeo (mas lento y con mas jitter).
const int PIN_INT_MPU = 18;   // push-pull, activo ALTO   (data ready)
const int PIN_INT_TCS = 19;   // open-drain, activo BAJO  (borde de linea)

// UART hacia la Raspberry Pi.
const int PIN_RX2 = 16;       // <- TX de la Pi
const int PIN_TX2 = 17;       // -> RX de la Pi
const uint32_t BAUDIOS = 115200;   // 8N1

// Pulsador de competencia, entre el pin y GND (pull-up interno). El 13 no es
// pin de arranque ni de entrada-sola: pisado al encender no impide el boot.
const int PIN_BOTON = 13;
// LED de estado: el azul de a bordo (GPIO 2 en casi todas las placas).
const int PIN_LED = 2;

// Si el carro avanza al reves, cambia esto a 1 en vez de recablear el motor.
#define INVERTIR_MOTOR 0

// ============================== PWM =======================================
const int MOTOR_PWM_FREQ = 20000;   // 20 kHz: fuera del rango audible y lejos
const int MOTOR_PWM_RES  = 8;       //   de la banda donde el IBT-2 silba
const int SERVO_PWM_FREQ = 50;      // 50 Hz estandar de servo
const int SERVO_PWM_RES  = 16;      // 16 bits: ~0.3 us de resolucion de pulso
const int SERVO_PULSO_MIN_US = 500;
const int SERVO_PULSO_MAX_US = 2400;

// ============================== TIEMPOS ===================================
const uint32_t TICK_CONTROL_MS  = 10;    // 100 Hz
const uint32_t PERIODO_TELE_MS  = 50;    // 20 Hz
const uint32_t PERIODO_SENS_MS  = 25;    // 40 Hz
const uint32_t FAILSAFE_MS      = 300;   // silencio tolerado de la Pi
const uint32_t VIGILANTE_MS     = 200;   // silencio tolerado del propio control
const uint32_t REINTENTO_I2C_MS = 3000;  // resondeo de sensores que falten

const uint8_t VERSION_FIRMWARE = 1;

// ====================== ESTADO COMPARTIDO =================================
QueueHandle_t colaMando = NULL;          // longitud 1: el nuevo pisa al viejo

volatile uint32_t msUltimoMando   = 0;
volatile uint32_t msUltimoControl = 0;
volatile uint8_t  ultimaSeq       = 0;
volatile uint8_t  estadoBits      = 0;
volatile uint8_t  pwmActual       = 0;
volatile uint8_t  anguloActual    = 100;
volatile uint32_t tramasMalas     = 0;
volatile int8_t   enlaceActivo    = -1;  // 0 = USB, 1 = GPIO, -1 = ninguno

// ARMADO LOCAL: lo pone y lo quita el BOTON, aqui, sin preguntar a la Pi.
// El motor solo gira si ademas la Pi manda F_ARMADO. Dos llaves, una a cada
// lado del enlace: ninguna sola basta.
volatile bool armadoLocal = false;
volatile bool cortadoPorBoton = false;

seg::ControlServo servo;
seg::ControlMotor motor;

panel::Boton boton;
panel::Led   led;
volatile uint8_t bitsBoton = 0;
volatile uint8_t pulsacionesPub = 0;

// --- sensores I2C: los toca solo tareaSensores; la copia publicada se
//     protege con spinlock porque telemetria la lee desde el otro nucleo ---
sens::Mpu6050 mpu;
sens::Tcs34725 tcs;
lin::Clasificador lineas;
proto::Sensores sensoresPub = {0, 0, 0, 0, 0, 0, 0, 0, VERSION_FIRMWARE};
portMUX_TYPE muxSensores = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t calPendiente = 0;       // proto::CAL_* pedido por la Pi
volatile bool    cfgPendiente = false;
seg::ConfigServo cfgServoNueva;
seg::ConfigMotor cfgMotorNueva;

// Umbral de la INT del TCS, en % del blanco que el firmware aprende solo.
// Al ser relativo no se estropea si cambia el tiempo de integracion.
// Umbral de la INT del TCS, en % del blanco aprendido. Con 55 no saltaba
// nunca para la naranja, que apenas oscurece el piso. Desde que se sondea a
// 500 Hz la INT solo es un adelanto —no se depende de ella para nada— pero
// un umbral que ignora medio juego de lineas es mentira en el codigo.
const uint8_t PCT_UMBRAL_INT = 90;

// ----------------------- interrupciones de sensores ------------------------
// Los ISR NO tocan el I2C (Wire no es seguro dentro de una interrupcion):
// solo apuntan CUANDO paso y despiertan a tareaSensores, que es la que lee.
TaskHandle_t hSensores = NULL;
static const uint32_t AVISO_MPU = (1u << 0);
static const uint32_t AVISO_TCS = (1u << 1);
volatile uint32_t usIntMpu = 0, usIntTcs = 0;
volatile uint32_t nIntMpu = 0, nIntTcs = 0;

void IRAM_ATTR isrMpu() {
  usIntMpu = micros();
  nIntMpu++;
  BaseType_t despertar = pdFALSE;
  if (hSensores) xTaskNotifyFromISR(hSensores, AVISO_MPU, eSetBits, &despertar);
  if (despertar) portYIELD_FROM_ISR();
}

void IRAM_ATTR isrTcs() {
  usIntTcs = micros();
  nIntTcs++;
  BaseType_t despertar = pdFALSE;
  if (hSensores) xTaskNotifyFromISR(hSensores, AVISO_TCS, eSetBits, &despertar);
  if (despertar) portYIELD_FROM_ISR();
}

HardwareSerial *enlaces[2] = { &Serial, &Serial2 };
proto::Lector lectores[2];

// ============================ HARDWARE ====================================
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

// Habilitacion del puente H. Bajar R_EN/L_EN deja el IBT-2 en alta impedancia:
// es una parada MAS FUERTE que poner el PWM a cero, porque no depende de que
// el registro de PWM tenga el valor que creemos.
static inline void habilitarPuente(bool on) {
  digitalWrite(PIN_R_EN, on ? HIGH : LOW);
  digitalWrite(PIN_L_EN, on ? HIGH : LOW);
}

// ============================ ENVIOS ======================================
static void enviarPorEnlaceActivo(const uint8_t *trama, uint8_t n) {
  if (n == 0) return;
  int8_t e = enlaceActivo;
  if (e < 0) {                       // aun no hablo nadie: contesta por las dos
    Serial.write(trama, n);
    Serial2.write(trama, n);
    return;
  }
  enlaces[e]->write(trama, n);
}

static void enviarLog(const char *texto) {
  uint8_t trama[5 + proto::MAX_PAYLOAD];
  uint8_t n = (uint8_t)strnlen(texto, proto::MAX_PAYLOAD);
  uint8_t total = proto::empaquetar(proto::TIPO_LOG, (const uint8_t *)texto, n,
                                    trama);
  enviarPorEnlaceActivo(trama, total);
}

// ============================ TAREA RX ====================================
// Prioridad alta (5) y nucleo 0. Solo hace tres cosas: sacar bytes de las dos
// UARTs, validar CRC y publicar. No calcula nada: cualquier trabajo aqui se
// paga en latencia de mando, que es lo unico que el failsafe puede confundir
// con "la Pi murio".
void tareaRx(void *) {
  uint8_t payload[proto::MAX_PAYLOAD];
  uint8_t tipo = 0, n = 0;
  uint32_t malas = tramasMalas;

  for (;;) {
    bool hubo = false;
    for (int e = 0; e < 2; e++) {
      while (enlaces[e]->available()) {
        hubo = true;
        uint8_t b = (uint8_t)enlaces[e]->read();
        if (!lectores[e].alimentar(b, tipo, payload, n, malas)) continue;

        enlaceActivo = (int8_t)e;      // contesta por donde te hablaron
        tramasMalas = malas > 255 ? 255 : malas;

        switch (tipo) {
          case proto::TIPO_MANDO: {
            proto::Mando m;
            if (!proto::decodificarMando(payload, n, m)) break;
            msUltimoMando = millis();
            ultimaSeq = m.seq;
            xQueueOverwrite(colaMando, &m);   // el nuevo pisa al viejo
            break;
          }
          case proto::TIPO_PING: {
            uint8_t trama[8];
            uint8_t eco = n ? payload[0] : 0;
            uint8_t t = proto::empaquetar(proto::TIPO_PONG, &eco, 1, trama);
            enviarPorEnlaceActivo(trama, t);
            break;
          }
          case proto::TIPO_CONFIG: {
            if (n < 6) break;
            cfgServoNueva.centro       = payload[0];
            cfgServoNueva.izquierda    = payload[1];
            cfgServoNueva.derecha      = payload[2];
            cfgServoNueva.gradosPorSeg = payload[3] * 10;
            cfgMotorNueva.rampaPorTick = payload[4];
            cfgMotorNueva.msFrenoAntesDeInvertir = payload[5] * 10;
            // PWM MINIMO DE ARRANQUE (byte 7, opcional).
            // Un PWM de 40 no mueve un motor de 500 rpm cargado con el peso
            // del carro y la reduccion del diferencial: solo calienta el
            // driver y hace zumbar el motor, y desde fuera parece que el
            // carro "se traba". Por debajo de este valor se empuja al minimo
            // util. Se lee condicionalmente para seguir aceptando la trama
            // vieja de 6 bytes sin romper nada.
            if (n >= 7) cfgMotorNueva.pwmMinArranque = payload[6];
            cfgPendiente = true;
            break;
          }
          case proto::TIPO_CAL:
            if (n >= 1) calPendiente = payload[0];
            break;
          default:
            break;
        }
      }
    }
    // Sin bytes: cede el nucleo 1 ms. Con bytes: vuelve enseguida, porque un
    // burst de la Pi cabe entero en el FIFO y no queremos dejarlo enfriar.
    vTaskDelay(hubo ? 0 : pdMS_TO_TICKS(1));
  }
}

// ========================= TAREA SENSORES =================================
// Nucleo 0, prioridad 3. Duerme hasta que una INT la despierta, con un latido
// de 5 ms como red por si las patas INT no estan cableadas.
void tareaSensores(void *) {
  vTaskDelay(pdMS_TO_TICKS(300));      // el riel de 5 V tiene que asentarse

  mpu.detectar();
  tcs.detectar();

  // Calibracion automatica del giroscopio al arrancar. El reglamento (9.9)
  // prohibe "realizar calibraciones de sensores en el vehiculo" como forma de
  // meter datos: por eso esto NO lo dispara nadie, se hace solo al encender,
  // con el carro donde el juez lo dejo, y el LED avisa para que nadie lo mueva.
  if (mpu.presente) mpu.calibrar(400);

  if (tcs.presente) tcs.probarInterrupcion(PIN_INT_TCS);
  if (mpu.presente) {
    uint32_t antes = nIntMpu;
    vTaskDelay(pdMS_TO_TICKS(50));
    mpu.int_ok = (nIntMpu - antes) > 5;    // a 200 Hz deberian ser ~10
  }
  if (tcs.presente && tcs.int_ok)
    tcs.configurarInterrupcion(lineas.umbralInterrupcion(PCT_UMBRAL_INT));

  uint32_t usPrevMpu = micros();
  uint32_t msPrevTcs = millis();
  uint32_t msProxSondeo = millis() + REINTENTO_I2C_MS;

  for (;;) {
    uint32_t aviso = 0;
    // 2 ms, no 5. El sondeo del TCS sale de esta espera, y cada milisegundo
    // que se tarda en preguntar se suma al periodo real de muestreo: con 12 ms
    // de integracion y 5 de granularidad salen ~17 ms entre muestras, y en una
    // linea de 20 mm eso deja UNA sola lectura. Con 2 quedan ~14 y caben dos.
    xTaskNotifyWait(0, 0xFFFFFFFF, &aviso, pdMS_TO_TICKS(2));
    uint32_t ahora = millis();

    // ---- comandos de calibracion pedidos por la Pi ---------------------
    uint8_t cal = calPendiente;
    if (cal) {
      calPendiente = 0;
      if (cal == proto::CAL_SESGO_GIRO)      mpu.calibrar(400);
      else if (cal == proto::CAL_CERO_YAW)   mpu.ceroYaw();
      else if (cal == proto::CAL_CERO_LINEAS) lineas.reiniciar();
      else if (cal == proto::CAL_REDETECTAR) {
        mpu.detectar();
        tcs.detectar();
        if (tcs.presente) tcs.probarInterrupcion(PIN_INT_TCS);
      }
    }

    // ---- MPU: integrar el rumbo con el dt real -------------------------
    if (mpu.presente) {
      uint32_t usAhora = mpu.int_ok ? usIntMpu : micros();
      uint32_t dt = usAhora - usPrevMpu;
      if (dt > 0) { mpu.paso_us(dt); usPrevMpu = usAhora; }
    }

    // ---- TCS: PREGUNTAR MUCHO, COGER LA MUESTRA EN CUANTO EXISTA -------
    //
    // Aqui habia un fallo que dejaba al carro ciego a las lineas la mitad de
    // las veces, y no se ve mirando el sensor: se ve haciendo la cuenta. El
    // chip integra durante `periodoMs()` y el codigo preguntaba CADA
    // `periodoMs()`. Dos relojes a la misma frecuencia, sin sincronizar,
    // derivan; y en cuanto derivan la pregunta cae siempre un pelo ANTES de
    // que el dato exista. Como el hueco hasta la pregunta siguiente ya estaba
    // reservado, el periodo real pasaba a ser el DOBLE. Una linea de 20 mm a
    // 1 m/s dura 20 ms: en un hueco de 48 ms cabe entera sin dejar rastro.
    //
    // La solucion no es afinar el periodo —dos relojes libres siempre acaban
    // derivando— sino dejar de programarlo: se sondea en cada vuelta de la
    // tarea (~200 Hz) y se coge la muestra en el instante en que el chip dice
    // que vale. Asi el periodo real ES el de integracion, sin depender de que
    // dos relojes se lleven bien. Preguntar de mas cuesta una lectura de un
    // byte por I2C: nada al lado de perderse una linea.
    if (tcs.presente) {
      if (tcs.leerColor()) {
        uint16_t dt = (uint16_t)(ahora - msPrevTcs);
        msPrevTcs = ahora;
        lineas.paso(tcs.c, tcs.r, tcs.g, tcs.b, dt ? dt : 1);
        // El umbral de la INT sigue al blanco aprendido: se reajusta solo,
        // despacio, para que no haya que recalibrar al cambiar de sala.
        if (tcs.int_ok) {
          uint16_t u = lineas.umbralInterrupcion(PCT_UMBRAL_INT);
          if (u > tcs.umbral_int + 30 || u + 30 < tcs.umbral_int)
            tcs.configurarInterrupcion(u);
          else
            tcs.limpiarInterrupcion();
        }
      }
    }

    // ---- resondeo de lo que falte --------------------------------------
    if ((int32_t)(ahora - msProxSondeo) >= 0) {
      msProxSondeo = ahora + REINTENTO_I2C_MS;
      if (!mpu.presente) mpu.detectar();
      if (!tcs.presente) tcs.detectar();
    }

    // ---- publicar copia coherente para la telemetria --------------------
    proto::Sensores s;
    s.estado = 0;
    if (mpu.presente)      s.estado |= proto::S_MPU_OK;
    if (tcs.presente)      s.estado |= proto::S_TCS_OK;
    if (mpu.calibrando)    s.estado |= proto::S_CALIBRANDO;
    if (lineas.sobreLinea()) s.estado |= proto::S_SOBRE_LINEA;
    if (mpu.int_ok)        s.estado |= proto::S_MPU_INT;
    if (tcs.int_ok)        s.estado |= proto::S_TCS_INT;
    s.estado |= (uint8_t)(lineas.clase << 6);
    s.yaw_d10 = (int16_t)(mpu.yaw * 10.0f);
    s.gz_d10  = (int16_t)(mpu.gz * 10.0f);
    s.claro   = tcs.c;
    s.blanco = lineas.blanco;
    s.separacion = lineas.separacion;
    s.cruces_naranja = lineas.cruces_naranja;
    s.cruces_azul    = lineas.cruces_azul;
    s.botones     = bitsBoton;
    s.pulsaciones = pulsacionesPub;
    s.version     = VERSION_FIRMWARE;

    portENTER_CRITICAL(&muxSensores);
    sensoresPub = s;
    portEXIT_CRITICAL(&muxSensores);
  }
}

// ======================== TAREA TELEMETRIA ================================
// Nucleo 0, prioridad 2 (la mas baja): si algo tiene que perder un turno, que
// sea contar lo que pasa, no hacerlo.
void tareaTelemetria(void *) {
  uint32_t proxTele = 0, proxSens = 0;
  uint8_t trama[5 + proto::MAX_PAYLOAD];

  for (;;) {
    uint32_t ahora = millis();

    if ((int32_t)(ahora - proxTele) >= 0) {
      proxTele = ahora + PERIODO_TELE_MS;
      proto::Telemetria t;
      t.seq_eco = ultimaSeq;
      t.estado  = estadoBits;
      t.pwm     = pwmActual;
      t.angulo  = anguloActual;
      uint32_t d = ahora - msUltimoMando;
      t.ms_desde_mando = (uint16_t)(d > 65535 ? 65535 : d);
      t.tramas_malas = (uint8_t)tramasMalas;
      t.version = VERSION_FIRMWARE;
      enviarPorEnlaceActivo(trama, proto::empaquetarTelemetria(t, trama));
    }

    if ((int32_t)(ahora - proxSens) >= 0) {
      proxSens = ahora + PERIODO_SENS_MS;
      proto::Sensores s;
      portENTER_CRITICAL(&muxSensores);
      s = sensoresPub;
      portEXIT_CRITICAL(&muxSensores);
      enviarPorEnlaceActivo(trama, proto::empaquetarSensores(s, trama));
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ========================= TAREA CONTROL ==================================
// Nucleo 1, prioridad 4. Tick fijo de 10 ms con vTaskDelayUntil: el periodo no
// se desplaza aunque un ciclo tarde de mas, que es lo que mantiene honestas
// las rampas (estan expresadas en cuentas POR TICK).
void tareaControl(void *) {
  TickType_t ultimo = xTaskGetTickCount();
  proto::Mando m = {0, 0, 0, 0, 255, 0};
  bool hayMando = false;

  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(TICK_CONTROL_MS));
    uint32_t ahora = millis();
    msUltimoControl = ahora;

    // ---- configuracion en caliente -------------------------------------
    if (cfgPendiente) {
      cfgPendiente = false;
      servo.configurar(cfgServoNueva);    // los topes de compilacion mandan
      motor.configurar(cfgMotorNueva);
    }

    // ---- BOTON: arma, desarma y corta AQUI -----------------------------
    if (boton.paso(ahora)) {
      if (armadoLocal) {                 // pulsado con el carro corriendo
        armadoLocal = false;
        cortadoPorBoton = true;
        motor.cortar(ahora);
        habilitarPuente(false);
        escribirMotorHW(0);
      } else {
        armadoLocal = true;              // esta es la senal de INICIO (9.13)
        cortadoPorBoton = false;
      }
    }
    bitsBoton = (boton.pisado() ? proto::B_NIVEL : 0) |
                (cortadoPorBoton ? proto::B_CORTE : 0);
    pulsacionesPub = boton.pulsaciones();

    // ---- ultimo mando recibido -----------------------------------------
    proto::Mando nuevo;
    if (xQueueReceive(colaMando, &nuevo, 0) == pdTRUE) { m = nuevo; hayMando = true; }

    bool failsafe = (ahora - msUltimoMando) > FAILSAFE_MS;
    if (!hayMando) failsafe = true;      // nunca hablo nadie: no se mueve

    // La Pi puede pedir rearmar tras un corte, pero solo si el boton volvio a
    // armar: F_LIMPIAR borra la marca, NO devuelve la traccion por si sola.
    if (m.limpiar() && armadoLocal) cortadoPorBoton = false;

    // ---- DIRECCION ------------------------------------------------------
    // Se mueve aunque el motor este cortado: enderezar las ruedas con el carro
    // parado no es peligroso y ayuda a colocarlo entre rondas.
    int objetivo = m.centrar() ? servo.config().centro
                               : servo.anguloDesdePorcentaje(m.dir);
    escribirServoHW(servo.paso(objetivo, TICK_CONTROL_MS));

    // ---- TRACCION -------------------------------------------------------
    // DOS LLAVES: el boton (armadoLocal) y la Pi (F_ARMADO). Falta cualquiera
    // de las dos y el puente H se queda en alta impedancia.
    bool puedeMover = armadoLocal && m.armado() && !cortadoPorBoton &&
                      !failsafe && !m.parada();
    if (!puedeMover) {
      motor.cortar(ahora);
      habilitarPuente(false);
      escribirMotorHW(0);
    } else {
      habilitarPuente(true);
      int pedido = (int)m.vel * 255 / 100;
      int techo = (int)m.vmax;
      if (pedido >  techo) pedido =  techo;
      if (pedido < -techo) pedido = -techo;
      escribirMotorHW(motor.paso(pedido, ahora));
    }

    // ---- estado publicado y LED -----------------------------------------
    uint8_t e = 0;
    if (armadoLocal && m.armado()) e |= proto::E_ARMADO;
    if (pwmActual > 0)             e |= proto::E_MOTOR;
    if (failsafe)                  e |= proto::E_FAILSAFE;
    if (servo.saturado())          e |= proto::E_SERVO_TOPE;
    if (motor.bloqueada())         e |= proto::E_INV_BLOQUEADA;
    estadoBits = e;

    // sensoresPub la escribe tareaSensores en el OTRO nucleo: se lee bajo el
    // mismo spinlock, aunque sea un byte. Sin el, el compilador tiene derecho
    // a cachearla en un registro y el LED se quedaria mintiendo.
    portENTER_CRITICAL(&muxSensores);
    uint8_t estadoSens = sensoresPub.estado;
    portEXIT_CRITICAL(&muxSensores);
    bool calibrando = (estadoSens & proto::S_CALIBRANDO) != 0;
    if (calibrando)            led.patron(panel::Luz::CALIBRANDO);
    else if (cortadoPorBoton)  led.patron(panel::Luz::CORTADO);
    else if (!armadoLocal)     led.patron(panel::Luz::ESPERA);
    else if (failsafe)         led.patron(panel::Luz::SIN_ORDENES);
    else                       led.patron(panel::Luz::ARMADO);
    led.paso(ahora);
  }
}

// ======================== TAREA VIGILANTE =================================
// Nucleo 1, prioridad 6 (la mas alta del sistema). No conduce: vigila al que
// conduce. Si tareaControl deja de dar senales de vida — un bucle infinito, un
// I2C colgado, un deadlock — nadie mas bajaria el PWM, y el carro se iria
// contra el muro a velocidad de crucero con el programa "funcionando".
void tareaVigilante(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(50));
    uint32_t ahora = millis();
    if ((ahora - msUltimoControl) > VIGILANTE_MS) {
      habilitarPuente(false);
      ledcWrite(PIN_RPWM, 0);
      ledcWrite(PIN_LPWM, 0);
      pwmActual = 0;
      estadoBits |= proto::E_FAILSAFE;
      enviarLog("CTRL MUDO");
    }
  }
}

// ============================== SETUP =====================================
void setup() {
  // --- puente H: enables primero y en BAJO. Entre el reset y este punto los
  //     pines flotan; si el IBT-2 leyera un enable alto con un PWM basura, el
  //     carro daria un tiron en la mesa de jueces.
  pinMode(PIN_R_EN, OUTPUT);
  pinMode(PIN_L_EN, OUTPUT);
  habilitarPuente(false);

  ledcAttach(PIN_RPWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttach(PIN_LPWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(PIN_RPWM, 0);
  ledcWrite(PIN_LPWM, 0);

  ledcAttach(PIN_SERVO, SERVO_PWM_FREQ, SERVO_PWM_RES);
  servo.forzarCentro();
  escribirServoHW(servo.config().centro);

  boton.iniciar(PIN_BOTON);
  led.iniciar(PIN_LED);
  led.patron(panel::Luz::ESPERA);

  Serial.begin(BAUDIOS);
  Serial2.begin(BAUDIOS, SERIAL_8N1, PIN_RX2, PIN_TX2);

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);

  pinMode(PIN_INT_MPU, INPUT);            // push-pull: no necesita pull-up
  pinMode(PIN_INT_TCS, INPUT_PULLUP);     // open-drain: si lo necesita
  attachInterrupt(digitalPinToInterrupt(PIN_INT_MPU), isrMpu, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_TCS), isrTcs, FALLING);

  colaMando = xQueueCreate(1, sizeof(proto::Mando));

  // Reparto explicito de nucleos: nada que pueda bloquear (I2C, serial)
  // comparte nucleo con lo que mueve el motor.
  xTaskCreatePinnedToCore(tareaRx,         "rx",    4096, NULL, 5, NULL,       0);
  xTaskCreatePinnedToCore(tareaSensores,   "sens",  4096, NULL, 3, &hSensores, 0);
  xTaskCreatePinnedToCore(tareaTelemetria, "tele",  3072, NULL, 2, NULL,       0);
  xTaskCreatePinnedToCore(tareaControl,    "ctrl",  4096, NULL, 4, NULL,       1);
  xTaskCreatePinnedToCore(tareaVigilante,  "wdog",  2048, NULL, 6, NULL,       1);
}

// El lazo de Arduino queda vacio a proposito: si alguien mete codigo aqui,
// corre en el nucleo 1 con prioridad 1 y compite con el control.
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
