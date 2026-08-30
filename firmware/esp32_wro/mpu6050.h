// ---------------------------------------------------------------------------
// mpu6050.h  -  Driver minimo por registros (solo necesita Wire.h)
//
// Se usa unicamente el giroscopio en Z para estimar el rumbo (yaw). El
// acelerometro se lee para detectar golpes / robot atascado.
// ---------------------------------------------------------------------------
#pragma once
#include <Wire.h>
#include "config.h"

class Mpu6050 {
public:
  bool  ok        = false;
  float yaw_deg   = 0.0f;   // rumbo integrado (se reinicia con el comando Z)
  float gz_dps    = 0.0f;   // velocidad angular en Z, grados/s (sin bias)
  float accel_mag = 1.0f;   // modulo de la aceleracion en g
  float bias_z    = 0.0f;
  bool  calibrated = false;
  bool  drift_ready = false;   // el bias se ha refinado en reposo

  bool begin() {
    // PWR_MGMT_1: salir de sleep, reloj = PLL con giro X
    if (!writeReg(0x6B, 0x01)) return false;
    delay(20);
    writeReg(0x1A, 0x03);   // CONFIG: DLPF 44 Hz  -> giro muestreado a 1 kHz
    writeReg(0x19, 0x04);   // SMPLRT_DIV: 1000/(1+4) = 200 Hz
    writeReg(0x1B, 0x08);   // GYRO_CONFIG: +-500 dps -> 65.5 LSB/(deg/s)
    writeReg(0x1C, 0x08);   // ACCEL_CONFIG: +-4 g    -> 8192 LSB/g
    delay(20);
    uint8_t who = 0;
    ok = readRegs(0x75, &who, 1);
    // El registro WHO_AM_I devuelve 0x68 (MPU6050), 0x70/0x72/0x73 en clones.
    ok = ok && (who != 0x00 && who != 0xFF);
    return ok;
  }

  // Estima el bias del giroscopio con el robot completamente quieto.
  // Se llama sola al arrancar y con el comando "Z" (solo si el robot no se mueve).
  void calibrate() {
    if (!ok) return;
    double acc = 0.0;
    int    n   = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
      int16_t gz;
      if (readGyroZRaw(gz)) { acc += gz; n++; }
      delayMicroseconds(700);
    }
    if (n > 100) {
      bias_z     = (float)(acc / n);
      calibrated = true;
    }
    yaw_deg = 0.0f;
  }

  // dt en segundos. `idle` = el robot esta parado y desarmado: en ese caso se
  // refina el bias sin bloquear nada, de modo que cuando llegue la orden de
  // arranque ya este listo y no haya que parar el firmware un segundo entero
  // (eso desbordaba el buffer serie y perdia comandos de la Raspberry).
  void update(float dt, bool idle = false) {
    if (!ok) return;
    int16_t raw;
    if (!readGyroZRaw(raw)) return;
    if (idle) {
      float d = (float)raw - bias_z;
      if (d > -200.0f && d < 200.0f) {      // solo si de verdad esta quieto
        bias_z += d * 0.002f;
        drift_ready = true;
      }
    }
    float dps = ((float)raw - bias_z) / 65.5f;      // +-500 dps
    // Zona muerta: elimina la deriva por ruido cuando el robot esta quieto.
    if (dps > -0.35f && dps < 0.35f) dps = 0.0f;
    gz_dps   = dps;
    yaw_deg += dps * dt;
    if (yaw_deg >  100000.0f) yaw_deg -= 100000.0f;
    if (yaw_deg < -100000.0f) yaw_deg += 100000.0f;
  }

  void updateAccel() {
    uint8_t b[6];
    if (!readRegs(0x3B, b, 6)) return;
    int16_t ax = (int16_t)((b[0] << 8) | b[1]);
    int16_t ay = (int16_t)((b[2] << 8) | b[3]);
    int16_t az = (int16_t)((b[4] << 8) | b[5]);
    float fx = ax / 8192.0f, fy = ay / 8192.0f, fz = az / 8192.0f;
    accel_mag = sqrtf(fx * fx + fy * fy + fz * fz);
  }

  void resetYaw() { yaw_deg = 0.0f; }

private:
  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)MPU_ADDR, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
  }
  bool readGyroZRaw(int16_t &out) {
    uint8_t b[2];
    if (!readRegs(0x47, b, 2)) return false;   // GYRO_ZOUT_H
    out = (int16_t)((b[0] << 8) | b[1]);
    return true;
  }
};
