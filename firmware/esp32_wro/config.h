// ---------------------------------------------------------------------------
// config.h  -  Pines y constantes de compilacion del ESP32
// WRO Future Engineers 2026
// ---------------------------------------------------------------------------
#pragma once

// ------------------------- Pines (segun tu cableado) -----------------------
#define PIN_RPWM        25   // IBT_2 RPWM  (PWM avance)
#define PIN_LPWM        26   // IBT_2 LPWM  (PWM retroceso)
#define PIN_R_EN        27   // IBT_2 R_EN  (habilitacion)
#define PIN_L_EN        33   // IBT_2 L_EN  (habilitacion)
#define PIN_SERVO       32   // MG996R direccion
#define PIN_SDA         21   // I2C  (MPU6050 + TCS34725)
#define PIN_SCL         22   // I2C

// Opcionales. Pon -1 para deshabilitar.
#define PIN_BUTTON       4   // Boton de arranque (a GND, usa pull-up interno)
#define PIN_LED          2   // LED de estado (LED integrado en muchas devkits)

// ------------------------- Canales LEDC ------------------------------------
#define CH_RPWM          0
#define CH_LPWM          1
#define CH_SERVO         2

#define MOTOR_PWM_FREQ   20000   // 20 kHz (fuera del rango audible)
#define MOTOR_PWM_BITS   8       // 0..255
#define SERVO_PWM_FREQ   50      // 50 Hz
#define SERVO_PWM_BITS   16      // 0..65535 -> ~0.3 us de resolucion

// ------------------------- Comunicacion ------------------------------------
#define SERIAL_BAUD      115200
#define TELEMETRY_HZ     50      // frecuencia de envio de telemetria a la Pi
#define CMD_TIMEOUT_MS   350     // watchdog: si no llega comando, se frena

// ------------------------- I2C ---------------------------------------------
#define I2C_FREQ         400000
#define MPU_ADDR         0x68    // 0x69 si AD0 va a VCC
#define TCS_ADDR         0x29

// ------------------------- Lazo de sensores --------------------------------
#define IMU_HZ           200     // integracion del giroscopio
#define GYRO_CAL_SAMPLES 1500    // muestras para estimar el bias en reposo
