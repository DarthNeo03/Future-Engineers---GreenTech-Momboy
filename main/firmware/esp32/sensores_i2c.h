// ===========================================================================
// sensores_i2c.h — MPU6050 (giroscopio) y TCS34725 (color) en el I2C del ESP32.
//
// Cableado real del carro:
//     SDA -> GPIO 21, SCL -> GPIO 22, bus a 400 kHz
//     MPU6050 en 0x68 (o 0x69 con AD0 a VCC), TCS34725 en 0x29 (fija)
//
// LECCION APRENDIDA (documentada en el proyecto viejo): sondear los sensores
// en setup() justo despues de Wire.begin() falla, porque el MPU y el TCS
// tardan en despertar y comparten alimentacion con el motor, que hunde el
// riel al arrancar. Por eso aqui:
//   - se espera >250 ms antes del primer sondeo,
//   - se reintenta cada 3 s mientras falte alguno,
//   - y hay reintento manual desde la Pi (CAL_REDETECTAR).
//
// LAS PATAS INT (lo que aporta cada una, que NO es lo mismo)
//
//   MPU6050 INT = DATA READY. Pulsa cada vez que hay una muestra nueva, a la
//   frecuencia configurada (200 Hz). Lo que gana el carro no es velocidad
//   sino EXACTITUD: el yaw se integra con el dt medido entre flancos, en
//   microsegundos, en vez de con el reloj de la tarea, que llega cuando el
//   planificador quiere. Ese jitter se integraba directamente en el rumbo.
//
//   TCS34725 INT = UMBRAL sobre el canal claro (AILT/AIHT + PERS). NO es data
//   ready: no hace que el sensor integre mas rapido. Lo que da es un flanco
//   por hardware, ENGANCHADO, en el instante en que el suelo se oscurece —o
//   sea, en el borde de la linea— con marca de tiempo tomada en el ISR. Aunque
//   la tarea llegue tarde, el cruce no se pierde y se sabe cuando ocurrio.
//
//   Para tener MAS MUESTRAS por linea hay que bajar ATIME (el tiempo de
//   integracion): 0xF6 son 24 ms y 0xFF son 2.4 ms. OJO: al cambiarlo cambian
//   todos los valores absolutos, asi que hay que repetir la calibracion del
//   TCS. El umbral de la interrupcion no sufre porque se expresa en % del
//   nivel del piso, que el firmware aprende solo.
//
// Este archivo usa Wire (Arduino), asi que NO se compila en el PC. La logica
// que si se puede romper (clasificador de lineas) vive aparte en lineas.h.
// ===========================================================================
#ifndef SENSORES_I2C_H
#define SENSORES_I2C_H

#include <Arduino.h>
#include <Wire.h>

namespace sens {

// ---------------------------------------------------------------- MPU6050 --
class Mpu6050 {
 public:
  bool presente = false;
  bool int_ok = false;      // llegan flancos DATA READY por la pata INT
  uint8_t direccion = 0x68;
  float yaw = 0.0f;         // grados, -180..180
  float gz = 0.0f;          // grados/s, ya sin sesgo
  bool calibrando = false;
  static const uint16_t HZ_MUESTREO = 200;   // 1 kHz / (SMPLRT_DIV + 1)

  bool detectar() {
    for (uint8_t d : {(uint8_t)0x68, (uint8_t)0x69}) {
      uint8_t quien = 0;
      if (!leer(d, 0x75, &quien, 1)) continue;
      // 0x68 = MPU6050; 0x70/0x71/0x73/0x98 = variantes compatibles
      if (quien == 0x68 || quien == 0x70 || quien == 0x71 ||
          quien == 0x73 || quien == 0x98) {
        direccion = d;
        if (!escribir(d, 0x6B, 0x01)) continue;   // despertar, reloj giro X
        delay(5);
        escribir(d, 0x19, 0x04);                  // 1 kHz / 5 = 200 Hz
        escribir(d, 0x1A, 0x03);                  // DLPF 44 Hz
        escribir(d, 0x1B, 0x00);                  // +-250 dps
        // --- pata INT: un pulso por muestra nueva ---------------------
        // INT_PIN_CFG: push-pull, activo ALTO, pulso de 50 us (sin enclavar)
        // y INT_RD_CLEAR=1, para que cualquier lectura limpie el estado y
        // no haya forma de quedarse con la interrupcion colgada.
        escribir(d, 0x37, 0x10);
        escribir(d, 0x38, 0x01);                  // INT_ENABLE: DATA_RDY_EN
        presente = true;
        return true;
      }
    }
    presente = false;
    return false;
  }

  // Leer el giro Z e integrar el yaw con el dt REAL entre muestras, en
  // microsegundos. Con la pata INT ese dt es el que mide el propio sensor
  // (flanco a flanco); sin ella, el de la tarea, que trae jitter.
  bool paso_us(uint32_t dt_us) {
    if (!presente || calibrando) return presente;
    uint8_t d[2];
    if (!leer(direccion, 0x47, d, 2)) {          // GYRO_ZOUT_H
      if (++fallos_ > 20) presente = false;      // se solto un cable
      return presente;
    }
    fallos_ = 0;
    int16_t crudo = (int16_t)((d[0] << 8) | d[1]);
    gz = crudo / 131.0f - sesgo_;
    // Un hueco enorme (tarea atascada, depuracion) no debe meter un salto de
    // rumbo: se recorta a 100 ms, veinte veces el periodo nominal.
    if (dt_us > 100000u) dt_us = 100000u;
    yaw += gz * (dt_us / 1000000.0f);
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw <= -180.0f) yaw += 360.0f;
    return true;
  }

  bool paso(uint32_t dt_ms) { return paso_us(dt_ms * 1000u); }

  // Medir el sesgo con el carro QUIETO. Bloquea ~1 s la tarea de sensores
  // (no la de control, que corre en otro nucleo). Pone el yaw a cero.
  void calibrar(uint16_t muestras = 400) {
    if (!presente) return;
    calibrando = true;
    float suma = 0;
    uint16_t ok = 0;
    for (uint16_t i = 0; i < muestras; i++) {
      uint8_t d[2];
      if (leer(direccion, 0x47, d, 2)) {
        suma += (int16_t)((d[0] << 8) | d[1]) / 131.0f;
        ok++;
      }
      delay(2);
    }
    if (ok > muestras / 2) sesgo_ = suma / ok;
    yaw = 0.0f;
    calibrando = false;
  }

  void ceroYaw() { yaw = 0.0f; }

 private:
  float sesgo_ = 0.0f;
  uint8_t fallos_ = 0;

  static bool escribir(uint8_t dir, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(dir);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  static bool leer(uint8_t dir, uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(dir);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)dir, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
  }
};

// --------------------------------------------------------------- TCS34725 --
class Tcs34725 {
 public:
  bool presente = false;
  bool int_ok = false;           // la pata INT esta cableada y responde
  uint16_t umbral_int = 0;       // por debajo de este claro, salta la INT
  uint16_t c = 0, r = 0, g = 0, b = 0;

  // atime: 0xFF=2.4ms, 0xF6=24ms, 0xEB=50ms. gain: 0=x1 1=x4 2=x16 3=x60.
  // Bajar ATIME multiplica las muestras por linea (que es lo que hace falta
  // para cruzarlas rapido) pero baja los valores absolutos: hay que repetir
  // la calibracion del TCS despues de tocarlo.
  bool detectar(uint8_t atime = 0xF6, uint8_t gain = 2) {
    uint8_t id = 0;
    if (!leer(0x12, &id, 1)) { presente = false; return false; }
    if (id != 0x44 && id != 0x4D && id != 0x10) { presente = false; return false; }
    escribir(0x00, 0x01);          // PON
    delay(3);
    configurar(atime, gain);
    escribir(0x00, 0x03);          // PON | AEN
    presente = true;
    int_ok = false;
    umbral_int = 0;
    return true;
  }

  void configurar(uint8_t atime, uint8_t gain) {
    atime_ = atime;
    escribir(0x01, atime);         // ATIME
    escribir(0x0F, gain & 0x03);   // CONTROL (ganancia)
  }

  // ---- interrupcion por umbral del canal claro -------------------------
  // Salta cuando el claro cae por debajo de 'umbral', o sea cuando el suelo
  // se oscurece: el borde de una linea. La ventana alta se deja en el maximo
  // para que solo interrumpa por abajo.
  void configurarInterrupcion(uint16_t umbral) {
    escribir(0x04, (uint8_t)(umbral & 0xFF));   // AILTL
    escribir(0x05, (uint8_t)(umbral >> 8));     // AILTH
    escribir(0x06, 0xFF);                       // AIHTL
    escribir(0x07, 0xFF);                       // AIHTH
    escribir(0x0C, 0x00);                       // PERS: cada ciclo cuenta
    escribir(0x00, 0x13);                       // PON | AEN | AIEN
    umbral_int = umbral;
    limpiarInterrupcion();
  }

  // Comando especial 0xE6: "clear channel interrupt clear".
  void limpiarInterrupcion() {
    Wire.beginTransmission(DIR);
    Wire.write(0xE6);
    Wire.endTransmission();
  }

  // Autotest del cableado: se pone una ventana imposible (todo queda fuera),
  // asi que la pata INT TIENE que activarse. Si no baja, es que no esta
  // conectada, y el firmware sigue funcionando por sondeo.
  bool probarInterrupcion(int pin) {
    if (!presente) return false;
    escribir(0x04, 0xFF); escribir(0x05, 0xFF);   // AILT = 0xFFFF
    escribir(0x06, 0x00); escribir(0x07, 0x00);   // AIHT = 0x0000
    escribir(0x0C, 0x00);
    escribir(0x00, 0x13);                         // PON | AEN | AIEN
    delay(periodoMs() * 2 + 10);
    int_ok = (digitalRead(pin) == LOW);           // open-drain, activo bajo
    limpiarInterrupcion();
    return int_ok;
  }

  // Periodo de integracion en ms (para saber cada cuanto leer).
  uint16_t periodoMs() const {
    uint16_t ciclos = 256 - atime_;
    uint32_t ms = (uint32_t)ciclos * 24 / 10;   // 2.4 ms por ciclo
    return ms < 3 ? 3 : (uint16_t)ms;
  }

  bool leerColor() {
    if (!presente) return false;
    uint8_t st = 0;
    if (!leer(0x13, &st, 1)) { fallo(); return presente; }
    if (!(st & 0x01)) return true;               // aun integrando
    uint8_t d[8];
    if (!leer(0x14, d, 8)) { fallo(); return presente; }
    fallos_ = 0;
    c = d[0] | (d[1] << 8);
    r = d[2] | (d[3] << 8);
    g = d[4] | (d[5] << 8);
    b = d[6] | (d[7] << 8);
    return true;
  }

 private:
  static const uint8_t DIR = 0x29;
  uint8_t atime_ = 0xF6;
  uint8_t fallos_ = 0;

  void fallo() { if (++fallos_ > 20) presente = false; }

  static bool escribir(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(DIR);
    Wire.write(0x80 | reg);        // bit de comando
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  static bool leer(uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(DIR);
    Wire.write(0x80 | (n > 1 ? 0x20 : 0x00) | reg);   // auto-incremento
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)DIR, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
  }
};

}  // namespace sens

#endif  // SENSORES_I2C_H
