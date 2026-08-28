// ===========================================================================
// sensores_i2c.h — MPU6050 y TCS34725 en el ESP32, los dos AUTODETECTADOS.
//
// Si el chip no contesta en el arranque, el objeto se queda con presente=false
// y nadie mas lo toca: el firmware funciona exactamente igual sin sensores.
// Puedes enchufarlos cuando quieras sin recompilar nada.
//
// Ambos comparten el bus I2C (SDA 21, SCL 22 por defecto) porque tienen
// direcciones distintas: 0x68/0x69 el MPU y 0x29 el TCS.
//
// Los pines INT estan soportados pero no son obligatorios:
//   - MPU  INT: se usa como "hay dato nuevo" para leer justo cuando toca en
//     vez de dar vueltas. Si no llega ningun flanco, se cae solo a sondeo.
//   - TCS  INT: se usa como "el color se salio de la ventana", que es
//     literalmente "acabo de pisar una linea".
// ===========================================================================
#ifndef SENSORES_I2C_H
#define SENSORES_I2C_H

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
class MPU6050 {
 public:
  static const uint8_t REG_PWR_MGMT_1 = 0x6B;
  static const uint8_t REG_SMPLRT_DIV = 0x19;
  static const uint8_t REG_CONFIG = 0x1A;
  static const uint8_t REG_GYRO_CONFIG = 0x1B;
  static const uint8_t REG_ACCEL_CONFIG = 0x1C;
  static const uint8_t REG_INT_ENABLE = 0x38;
  static const uint8_t REG_WHO_AM_I = 0x75;
  static const uint8_t REG_DATOS = 0x3B;

  bool presente = false;
  uint8_t direccion = 0;
  float yaw = 0.0f;          // grados, -180..180
  float giroZ = 0.0f;        // grados/s ya sin deriva
  float temp = 0.0f;
  bool calibrado = false;

  bool iniciar(TwoWire &bus, bool usarInt, int pinInt) {
    bus_ = &bus;
    for (uint8_t dir : {(uint8_t)0x68, (uint8_t)0x69}) {
      uint8_t quien = 0;
      if (!leer(dir, REG_WHO_AM_I, &quien, 1)) continue;
      // 0x68 = MPU6050; 0x70/0x71/0x73 = MPU6500/9250, compatibles de sobra
      if (quien == 0x68 || quien == 0x70 || quien == 0x71 || quien == 0x73) {
        direccion = dir;
        break;
      }
    }
    if (!direccion) return false;

    escribir(direccion, REG_PWR_MGMT_1, 0x01);   // reloj del giroscopo X
    delay(50);
    escribir(direccion, REG_SMPLRT_DIV, 0x04);   // 1 kHz / 5 = 200 Hz
    escribir(direccion, REG_CONFIG, 0x03);       // filtro paso bajo 44 Hz
    escribir(direccion, REG_GYRO_CONFIG, 0x00);  // +-250 dps
    escribir(direccion, REG_ACCEL_CONFIG, 0x00);
    escribir(direccion, REG_INT_ENABLE, usarInt ? 0x01 : 0x00);  // dato listo
    if (usarInt && pinInt >= 0) pinMode(pinInt, INPUT);
    presente = true;
    return true;
  }

  // Deriva del giroscopo CON EL CARRO QUIETO. Sin esto el rumbo se va solo
  // uno o dos grados por segundo y a la tercera recta ya no significa nada.
  bool calibrar(uint16_t muestras = 400) {
    if (!presente) return false;
    double suma = 0;
    uint16_t buenas = 0;
    for (uint16_t i = 0; i < muestras; i++) {
      float gz;
      if (leerGiroZ(&gz, nullptr)) { suma += gz; buenas++; }
      delay(3);
    }
    if (buenas < muestras / 2) return false;
    sesgo_ = (float)(suma / buenas);
    yaw = 0.0f;
    calibrado = true;
    return true;
  }

  void cero() { yaw = 0.0f; }

  // dt en segundos
  bool actualizar(float dt) {
    if (!presente) return false;
    float gz, t;
    if (!leerGiroZ(&gz, &t)) return false;
    giroZ = gz - sesgo_;
    temp = t;
    yaw += giroZ * dt;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return true;
  }

 private:
  bool leerGiroZ(float *gz, float *t) {
    uint8_t d[14];
    if (!leer(direccion, REG_DATOS, d, 14)) return false;
    int16_t crudoT = (int16_t)((d[6] << 8) | d[7]);
    int16_t crudoZ = (int16_t)((d[12] << 8) | d[13]);
    *gz = crudoZ / 131.0f;                    // +-250 dps -> 131 LSB/(grado/s)
    if (t) *t = crudoT / 340.0f + 36.53f;
    return true;
  }

  bool escribir(uint8_t dir, uint8_t reg, uint8_t val) {
    bus_->beginTransmission(dir);
    bus_->write(reg);
    bus_->write(val);
    return bus_->endTransmission() == 0;
  }

  bool leer(uint8_t dir, uint8_t reg, uint8_t *destino, uint8_t n) {
    bus_->beginTransmission(dir);
    bus_->write(reg);
    if (bus_->endTransmission(false) != 0) return false;
    if (bus_->requestFrom((int)dir, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; i++) destino[i] = bus_->read();
    return true;
  }

  TwoWire *bus_ = nullptr;
  float sesgo_ = 0.0f;
};

// ---------------------------------------------------------------------------
class TCS34725 {
 public:
  static const uint8_t DIR = 0x29;
  static const uint8_t CMD = 0x80;
  static const uint8_t REG_ENABLE = 0x00;
  static const uint8_t REG_ATIME = 0x01;
  static const uint8_t REG_CONTROL = 0x0F;
  static const uint8_t REG_ID = 0x12;
  static const uint8_t REG_CDATA = 0x14;

  bool presente = false;
  uint16_t c = 0, r = 0, g = 0, b = 0;

  bool iniciar(TwoWire &bus, int pinInt) {
    bus_ = &bus;
    uint8_t id = 0;
    if (!leer(REG_ID, &id, 1)) return false;
    // 0x44 = TCS34725, 0x4D = TCS34727, 0x10 = algunos clones
    if (id != 0x44 && id != 0x4D && id != 0x10) return false;

    // ATIME 0xEB = 50 ms de integracion: rapido, y a 30 km/h simulados de
    // carro pequeno una linea de 20 mm sigue durando varias lecturas.
    escribir(REG_ATIME, 0xEB);
    escribir(REG_CONTROL, 0x01);       // ganancia x4
    escribir(REG_ENABLE, 0x01);        // encender el oscilador
    delay(3);
    escribir(REG_ENABLE, 0x03);        // + habilitar el ADC
    delay(60);
    if (pinInt >= 0) pinMode(pinInt, INPUT_PULLUP);
    presente = true;
    return true;
  }

  bool leerColor() {
    if (!presente) return false;
    uint8_t d[8];
    if (!leer(REG_CDATA, d, 8)) return false;
    c = (uint16_t)(d[0] | (d[1] << 8));
    r = (uint16_t)(d[2] | (d[3] << 8));
    g = (uint16_t)(d[4] | (d[5] << 8));
    b = (uint16_t)(d[6] | (d[7] << 8));
    return true;
  }

 private:
  bool escribir(uint8_t reg, uint8_t val) {
    bus_->beginTransmission(DIR);
    bus_->write(CMD | reg);
    bus_->write(val);
    return bus_->endTransmission() == 0;
  }

  bool leer(uint8_t reg, uint8_t *destino, uint8_t n) {
    bus_->beginTransmission(DIR);
    bus_->write(CMD | reg);
    if (bus_->endTransmission(false) != 0) return false;
    if (bus_->requestFrom((int)DIR, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; i++) destino[i] = bus_->read();
    return true;
  }

  TwoWire *bus_ = nullptr;
};

#endif  // SENSORES_I2C_H
