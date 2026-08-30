// ===========================================================================
//  WRO Future Engineers 2026  -  Firmware del ESP32
//  ------------------------------------------------------------------------
//  Responsabilidades del ESP32 (capa de tiempo real):
//    * PWM del puente IBT_2 (traccion) y del servo MG996R (direccion)
//    * Integracion del giroscopio MPU6050 -> rumbo (yaw)
//    * Deteccion de las lineas naranja/azul del tapete con el TCS34725
//    * Watchdog de seguridad: si la Raspberry deja de hablar, frena
//
//  Toda la vision, la estrategia y el servidor web viven en la Raspberry Pi 5.
//  La comunicacion es por USB (CDC o puente UART) a 115200 baudios.
//
//  PROTOCOLO  (lineas ASCII terminadas en '\n')
//  ------------------------------------------------------------------------
//  Pi -> ESP32
//    C <steer> <speed>    steer -1000..1000 (positivo = IZQUIERDA)
//                         speed -1000..1000 (positivo = ADELANTE)
//    S <0|1>              deshabilita / habilita la etapa de potencia
//    Z                    pone el rumbo (yaw) a 0. El bias del giroscopio se
//                         refina solo mientras el robot espera parado.
//    L                    pone a cero los contadores de lineas
//    P <nombre> <valor>   ajusta un parametro (ver applyParam)
//    G                    pide el volcado de todos los parametros
//    V                    ping / version
//
//  ESP32 -> Pi
//    T t=.. yaw=.. gz=.. am=.. line=.. no=.. nb=.. ls=.. seq=.. r=.. g=.. b=..
//      c=.. btn=.. arm=.. wd=..
//    A <texto>            acuse de recibo
//    E <texto>            error
//    # <texto>            log / depuracion
// ===========================================================================

#include <Wire.h>
#include "config.h"
#include "mpu6050.h"
#include "tcs34725.h"

// --------------------------------------------------------------------------
// Compatibilidad LEDC entre el core 2.x y el 3.x de Arduino-ESP32
// --------------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define LEDC_SETUP(pin, ch, freq, bits) ledcAttachChannel((pin), (freq), (bits), (ch))
  #define LEDC_WRITE(pin, ch, duty)       ledcWrite((pin), (duty))
#else
  #define LEDC_SETUP(pin, ch, freq, bits) do { ledcSetup((ch), (freq), (bits)); \
                                               ledcAttachPin((pin), (ch)); } while (0)
  #define LEDC_WRITE(pin, ch, duty)       ledcWrite((ch), (duty))
#endif

// --------------------------------------------------------------------------
// Parametros ajustables en caliente desde la Raspberry (comando P)
// --------------------------------------------------------------------------
struct Params {
  // Servo de direccion. Los tres pulsos definen el recorrido util.
  uint16_t servo_center_us = 1500;
  uint16_t servo_left_us   = 2000;   // pulso para steer = +1000 (izquierda)
  uint16_t servo_right_us  = 1000;   // pulso para steer = -1000 (derecha)
  uint16_t servo_slew_us   = 4000;   // us de pulso por segundo (limite de velocidad)
  uint8_t  steer_invert    = 0;

  // Traccion
  uint8_t  motor_invert    = 0;
  uint8_t  motor_min_pwm   = 40;     // vence la friccion estatica
  uint8_t  motor_max_pwm   = 255;
  uint16_t motor_slew      = 900;    // unidades de PWM por segundo (rampa)
  uint8_t  brake_active    = 1;      // 1 = frenado activo (ambos PWM a 0 y EN alto)
} P;

// --------------------------------------------------------------------------
// Estado global
// --------------------------------------------------------------------------
Mpu6050   imu;
Tcs34725  color;

int16_t   cmd_steer = 0;      // -1000..1000
int16_t   cmd_speed = 0;      // -1000..1000
bool      power_on  = false;  // etapa de potencia habilitada
bool      watchdog_trip = false;
uint32_t  last_cmd_ms = 0;

float     servo_us_now = 1500.0f;
float     pwm_now      = 0.0f;   // -255..255, con rampa aplicada

char      rxbuf[128];
uint8_t   rxlen = 0;

// Prototipos (el IDE de Arduino los genera solo, pero explicitarlos evita
// sorpresas al compilar con arduino-cli o PlatformIO).
void  setupActuators();
void  writeServoUs(float us);
float steerToUs(int16_t steer);
void  updateActuators(float dt);
bool  applyParam(const char *name, float v);
void  dumpParams();
void  handleLine(char *line);
void  pollSerial();
void  sendTelemetry();

// ==========================================================================
//  Actuadores
// ==========================================================================
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void setupActuators() {
  pinMode(PIN_R_EN, OUTPUT);
  pinMode(PIN_L_EN, OUTPUT);
  digitalWrite(PIN_R_EN, LOW);
  digitalWrite(PIN_L_EN, LOW);

  LEDC_SETUP(PIN_RPWM,  CH_RPWM,  MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  LEDC_SETUP(PIN_LPWM,  CH_LPWM,  MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  LEDC_SETUP(PIN_SERVO, CH_SERVO, SERVO_PWM_FREQ, SERVO_PWM_BITS);

  LEDC_WRITE(PIN_RPWM, CH_RPWM, 0);
  LEDC_WRITE(PIN_LPWM, CH_LPWM, 0);
  writeServoUs(P.servo_center_us);
}

void writeServoUs(float us) {
  us = clampf(us, 500.0f, 2400.0f);
  // periodo de 20000 us -> duty = us/20000 * 2^16
  uint32_t duty = (uint32_t)(us * 65535.0f / 20000.0f);
  LEDC_WRITE(PIN_SERVO, CH_SERVO, duty);
}

// Convierte el comando de direccion (-1000..1000) a microsegundos de pulso.
float steerToUs(int16_t steer) {
  float s = steer / 1000.0f;
  if (P.steer_invert) s = -s;
  s = clampf(s, -1.0f, 1.0f);
  if (s >= 0) return P.servo_center_us + s * ((float)P.servo_left_us  - P.servo_center_us);
  else        return P.servo_center_us + (-s) * ((float)P.servo_right_us - P.servo_center_us);
}

// Aplica rampas y escribe en el hardware. dt en segundos.
void updateActuators(float dt) {
  // ---- Direccion ----
  float target_us = steerToUs(cmd_steer);
  float max_step  = P.servo_slew_us * dt;
  float d = target_us - servo_us_now;
  if (d >  max_step) d =  max_step;
  if (d < -max_step) d = -max_step;
  servo_us_now += d;
  writeServoUs(servo_us_now);

  // ---- Traccion ----
  bool live = power_on && !watchdog_trip;
  float target_pwm = 0.0f;
  if (live && cmd_speed != 0) {
    float s = cmd_speed / 1000.0f;
    if (P.motor_invert) s = -s;
    float mag = fabsf(s);
    // Mapea |s| in (0,1] -> [min_pwm, max_pwm] para vencer la friccion estatica
    float pwm = P.motor_min_pwm + mag * ((float)P.motor_max_pwm - P.motor_min_pwm);
    target_pwm = (s >= 0) ? pwm : -pwm;
  }

  float step = P.motor_slew * dt;
  float dp = target_pwm - pwm_now;
  if (dp >  step) dp =  step;
  if (dp < -step) dp = -step;
  pwm_now += dp;
  if (fabsf(pwm_now) < 1.0f) pwm_now = 0.0f;

  if (!live) {
    digitalWrite(PIN_R_EN, P.brake_active ? HIGH : LOW);
    digitalWrite(PIN_L_EN, P.brake_active ? HIGH : LOW);
    LEDC_WRITE(PIN_RPWM, CH_RPWM, 0);
    LEDC_WRITE(PIN_LPWM, CH_LPWM, 0);
    pwm_now = 0.0f;
    return;
  }

  digitalWrite(PIN_R_EN, HIGH);
  digitalWrite(PIN_L_EN, HIGH);
  int p = (int)fabsf(pwm_now);
  if (p > 255) p = 255;
  if (pwm_now >= 0) { LEDC_WRITE(PIN_RPWM, CH_RPWM, p); LEDC_WRITE(PIN_LPWM, CH_LPWM, 0); }
  else              { LEDC_WRITE(PIN_RPWM, CH_RPWM, 0); LEDC_WRITE(PIN_LPWM, CH_LPWM, p); }
}

// ==========================================================================
//  Parametros
// ==========================================================================
bool applyParam(const char *name, float v) {
  #define PSET_U16(k, lo, hi) if (!strcmp(name, #k)) { P.k = (uint16_t)clampf(v, lo, hi); return true; }
  #define PSET_U8(k,  lo, hi) if (!strcmp(name, #k)) { P.k = (uint8_t) clampf(v, lo, hi); return true; }
  PSET_U16(servo_center_us, 500, 2400)
  PSET_U16(servo_left_us,   500, 2400)
  PSET_U16(servo_right_us,  500, 2400)
  PSET_U16(servo_slew_us,   200, 40000)
  PSET_U8 (steer_invert,      0, 1)
  PSET_U8 (motor_invert,      0, 1)
  PSET_U8 (motor_min_pwm,     0, 200)
  PSET_U8 (motor_max_pwm,    20, 255)
  PSET_U16(motor_slew,       50, 20000)
  PSET_U8 (brake_active,      0, 1)
  #undef PSET_U16
  #undef PSET_U8

  if (!strcmp(name, "th_orange_r"))  { color.th_orange_r = v; return true; }
  if (!strcmp(name, "th_orange_b"))  { color.th_orange_b = v; return true; }
  if (!strcmp(name, "th_blue_b"))    { color.th_blue_b   = v; return true; }
  if (!strcmp(name, "th_blue_r"))    { color.th_blue_r   = v; return true; }
  if (!strcmp(name, "th_clear_min")) { color.th_clear_min = (uint16_t)v; return true; }
  if (!strcmp(name, "confirm_n"))    { color.confirm_n    = (uint8_t)clampf(v,1,20); return true; }
  if (!strcmp(name, "refractory_ms")){ color.refractory_ms= (uint16_t)v; return true; }
  return false;
}

void dumpParams() {
  Serial.printf("A params servo_center_us=%u servo_left_us=%u servo_right_us=%u "
                "servo_slew_us=%u steer_invert=%u motor_invert=%u motor_min_pwm=%u "
                "motor_max_pwm=%u motor_slew=%u brake_active=%u "
                "th_orange_r=%.3f th_orange_b=%.3f th_blue_b=%.3f th_blue_r=%.3f "
                "th_clear_min=%u confirm_n=%u refractory_ms=%u\n",
                P.servo_center_us, P.servo_left_us, P.servo_right_us, P.servo_slew_us,
                P.steer_invert, P.motor_invert, P.motor_min_pwm, P.motor_max_pwm,
                P.motor_slew, P.brake_active,
                color.th_orange_r, color.th_orange_b, color.th_blue_b, color.th_blue_r,
                color.th_clear_min, color.confirm_n, color.refractory_ms);
}

// ==========================================================================
//  Protocolo serie
// ==========================================================================
void handleLine(char *line) {
  while (*line == ' ') line++;
  if (*line == 0) return;
  char c = *line++;

  switch (c) {
    case 'C': {                       // C <steer> <speed>
      int s = 0, v = 0;
      if (sscanf(line, "%d %d", &s, &v) == 2) {
        cmd_steer = (int16_t)constrain(s, -1000, 1000);
        cmd_speed = (int16_t)constrain(v, -1000, 1000);
        last_cmd_ms = millis();
        watchdog_trip = false;
      }
      break;
    }
    case 'S': {                       // S <0|1>
      int v = 0;
      if (sscanf(line, "%d", &v) == 1) {
        power_on = (v != 0);
        if (!power_on) { cmd_speed = 0; }
        last_cmd_ms = millis();
        watchdog_trip = false;
        Serial.printf("A power %d\n", (int)power_on);
      }
      break;
    }
    case 'Z':                         // reiniciar yaw + bias del giroscopio
      cmd_speed = 0;
      updateActuators(0.02f);
      imu.calibrate();
      Serial.printf("A zero bias=%.2f\n", imu.bias_z);
      break;
    case 'L':                         // reiniciar contadores de linea
      color.resetCounters();
      Serial.println("A lines 0");
      break;
    case 'P': {                       // P <nombre> <valor>
      char name[24]; float v = 0;
      if (sscanf(line, "%23s %f", name, &v) == 2) {
        if (applyParam(name, v)) Serial.printf("A set %s %.4f\n", name, v);
        else                     Serial.printf("E unknown param %s\n", name);
      }
      break;
    }
    case 'G':
      dumpParams();
      break;
    case 'V':
      Serial.printf("A wro-esp32 1.0 imu=%d tcs=%d\n", (int)imu.ok, (int)color.ok);
      break;
    default:
      Serial.printf("E cmd %c\n", c);
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxbuf[rxlen] = 0;
      if (rxlen) handleLine(rxbuf);
      rxlen = 0;
    } else if (rxlen < sizeof(rxbuf) - 1) {
      rxbuf[rxlen++] = ch;
    } else {
      rxlen = 0;   // linea demasiado larga: se descarta
    }
  }
}

void sendTelemetry() {
  int btn = 0;
#if PIN_BUTTON >= 0
  btn = (digitalRead(PIN_BUTTON) == LOW) ? 1 : 0;   // pull-up: pulsado = LOW
#endif
  Serial.printf("T t=%lu yaw=%.2f gz=%.2f am=%.3f line=%u no=%u nb=%u ls=%u seq=%lu "
                "r=%.3f g=%.3f b=%.3f c=%u btn=%d arm=%d wd=%d\n",
                (unsigned long)millis(), imu.yaw_deg, imu.gz_dps, imu.accel_mag,
                color.current, color.n_orange, color.n_blue, color.last_event,
                (unsigned long)color.seq,
                color.rn, color.gn, color.bn, color.cC,
                btn, (int)power_on, (int)watchdog_trip);
}

// ==========================================================================
//  setup / loop
// ==========================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

#if PIN_BUTTON >= 0
  pinMode(PIN_BUTTON, INPUT_PULLUP);
#endif
#if PIN_LED >= 0
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
#endif

  setupActuators();

  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ);
  Wire.setTimeOut(10);

  bool imu_ok = imu.begin();
  bool tcs_ok = color.begin();
  Serial.printf("# boot imu=%d tcs=%d\n", (int)imu_ok, (int)tcs_ok);

  if (imu_ok) {
    Serial.println("# calibrando giroscopio, no mover el robot...");
    imu.calibrate();
    Serial.printf("# bias_z=%.2f\n", imu.bias_z);
  }
  last_cmd_ms = millis();
}

void loop() {
  static uint32_t t_imu = 0, t_tel = 0, t_col = 0, t_acc = 0, t_led = 0;
  static uint32_t t_prev_us = micros();

  pollSerial();

  uint32_t now    = millis();
  uint32_t now_us = micros();
  float dt = (now_us - t_prev_us) * 1e-6f;
  if (dt < 0 || dt > 0.5f) dt = 0.005f;

  // ---- Watchdog: si la Pi deja de mandar comandos, se frena ----
  if (power_on && (now - last_cmd_ms) > CMD_TIMEOUT_MS) {
    watchdog_trip = true;
    cmd_speed = 0;
  }

  // ---- Giroscopio a IMU_HZ ----
  if (now - t_imu >= (1000 / IMU_HZ)) {
    float dt_imu = (now - t_imu) * 0.001f;
    t_imu = now;
    if (dt_imu > 0.2f) dt_imu = 1.0f / IMU_HZ;
    imu.update(dt_imu, !power_on && cmd_speed == 0);
  }
  if (now - t_acc >= 50) { t_acc = now; imu.updateAccel(); }

  // ---- Sensor de color ~80 Hz ----
  if (now - t_col >= 12) { t_col = now; color.update(now); }

  // ---- Actuadores en cada iteracion (rampas suaves) ----
  updateActuators(dt);
  t_prev_us = now_us;

  // ---- Telemetria ----
  if (now - t_tel >= (1000 / TELEMETRY_HZ)) { t_tel = now; sendTelemetry(); }

#if PIN_LED >= 0
  // LED: fijo si esta armado, parpadeo lento en espera, rapido si watchdog
  if (now - t_led >= (watchdog_trip ? 100u : (power_on ? 1000u : 400u))) {
    t_led = now;
    if (power_on && !watchdog_trip) digitalWrite(PIN_LED, HIGH);
    else digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
#endif
}
