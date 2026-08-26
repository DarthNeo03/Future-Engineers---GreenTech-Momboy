// ===========================================================================
// sensores.h — MPU6050 y TCS34725 colgados del ESP32.
//
// POR QUE AQUI Y NO EN LA RASPBERRY
// ---------------------------------
// La version anterior los tenia en el I2C de la Pi. Se han movido aqui por
// tres razones, en orden de peso:
//
//  1. MUESTREO DETERMINISTA. Es la decisiva, y la manda el sensor de color.
//     A 0,4 m/s una linea de 20 mm pasa por debajo del sensor en 50 ms. Un
//     hilo de Python en Linux, compitiendo con el lazo de vision a 30 fps y
//     con el servidor web, puede perder esa ventana sin avisar. Una tarea de
//     FreeRTOS con `vTaskDelayUntil` en un nucleo que no hace otra cosa, no.
//
//  2. CABLEADO. Los dos sensores estan fisicamente cerca del ESP32: el de
//     color tiene que ir a 10-15 mm del suelo y el ESP32 va abajo, junto al
//     puente H. Llevarlos a la Pi eran 30 cm de I2C pasando junto a los
//     cables del motor.
//
//  3. HAY SITIO DE SOBRA. El nucleo 0 solo atiende el UART y la telemetria.
//     Una tarea de sensores a 100 Hz no le quita nada al lazo de control, que
//     ademas vive en el nucleo 1.
//
// EL COSTE, QUE ES REAL: el yaw llega a la Pi con la latencia de la
// telemetria (50 ms). A 40 grados/s de giro son 2 grados de retraso, dentro
// de la tolerancia de 5 con la que se cierra el giro. Los cruces de linea NO
// tienen ese problema porque viajan como CONTADOR y no como evento: da igual
// cuando llegue la trama, el numero dice cuantos van.
//
// CABLEADO (los dos comparten bus; 0x29 y 0x68 no chocan):
//     SDA -> GPIO21     SCL -> GPIO22     VCC -> 3V3     GND -> GND
//
// La logica de decision (integrar el yaw, clasificar el color, detectar el
// cruce) esta separada del I2C a proposito, en funciones puras, para poder
// probarla con g++ en el PC desde tools/test_firmware.cpp.
// ===========================================================================
#ifndef SENSORES_H
#define SENSORES_H

#include <stdint.h>
#include <math.h>

namespace sens {

// --- MPU6050 ---------------------------------------------------------------
static const uint8_t MPU_DIR_A = 0x68;
static const uint8_t MPU_DIR_B = 0x69;
static const uint8_t MPU_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_SMPLRT_DIV = 0x19;
static const uint8_t MPU_CONFIG = 0x1A;
static const uint8_t MPU_GYRO_CONFIG = 0x1B;
static const uint8_t MPU_WHO_AM_I = 0x75;
static const uint8_t MPU_GYRO_ZOUT_H = 0x47;
static const float ESCALA_GIRO = 131.0f;      // LSB por grado/s a +-250 dps

// --- TCS34725 --------------------------------------------------------------
static const uint8_t TCS_DIR = 0x29;
static const uint8_t TCS_CMD = 0x80;
static const uint8_t TCS_CMD_AUTO = 0xA0;
static const uint8_t TCS_ENABLE = 0x00;
static const uint8_t TCS_ATIME = 0x01;
static const uint8_t TCS_CONTROL = 0x0F;
static const uint8_t TCS_ID = 0x12;
static const uint8_t TCS_CDATAL = 0x14;
static const uint8_t TCS_PON = 0x01;
static const uint8_t TCS_AEN = 0x02;

// ---------------------------------------------------------------------------
// Yaw: integracion del giroscopio
// ---------------------------------------------------------------------------
class Yaw {
 public:
  Yaw() : yaw_(0.0f), sesgo_(0.0f), calibrando_(false), n_cal_(0), suma_(0.0f) {}

  void empezarCalibracion(int muestras) {
    calibrando_ = true;
    n_cal_ = 0;
    objetivo_ = muestras > 0 ? muestras : 200;
    suma_ = 0.0f;
  }

  bool calibrando() const { return calibrando_; }

  // gz en cuentas crudas del MPU6050; dt en segundos.
  void paso(int16_t gz, float dt) {
    if (calibrando_) {
      suma_ += (float)gz;
      if (++n_cal_ >= objetivo_) {
        sesgo_ = suma_ / (float)n_cal_;
        calibrando_ = false;
        yaw_ = 0.0f;      // tras calibrar se empieza de cero, como la Pi
      }
      return;
    }
    // El SIGNO se aplica aqui: el navegador espera convenio de brujula, o sea
    // que el yaw AUMENTE al girar a la derecha. Segun como quede montado el
    // MPU en el chasis puede salir al reves; se corrige con `invertir` en vez
    // de recablear ni tocar la navegacion.
    const float grados_s = ((float)gz - sesgo_) / ESCALA_GIRO;
    yaw_ += (invertir ? -grados_s : grados_s) * dt;
    yaw_ = normalizar(yaw_);
  }

  void ponerCero() { yaw_ = 0.0f; }
  float grados() const { return yaw_; }
  int16_t decigrados() const {
    float d = yaw_ * 10.0f;
    if (d > 1800.0f) d = 1800.0f;
    if (d < -1800.0f) d = -1800.0f;
    return (int16_t)(d < 0 ? d - 0.5f : d + 0.5f);
  }

  static float normalizar(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
  }

  bool invertir = false;

 private:
  float yaw_;
  float sesgo_;
  bool calibrando_;
  int n_cal_, objetivo_;
  float suma_;
};

// ---------------------------------------------------------------------------
// Color del piso: clasificacion y deteccion de cruce
// ---------------------------------------------------------------------------
struct PerfilColor {
  float r, g, b, tol;
};

// Gemelo de la clasificacion de src/color_piso.py: se normaliza por la SUMA
// de los tres canales, no por los valores crudos, para que el brillo salga de
// la ecuacion y aguante que el LED envejezca o que el sensor quede algo mas
// alto de la cuenta.
inline uint8_t clasificar(uint16_t c, uint16_t r, uint16_t g, uint16_t b,
                          const PerfilColor *perfiles, uint8_t n,
                          uint16_t clear_min, uint16_t clear_max) {
  if (c < clear_min || c >= clear_max) return 0xFF;   // 0xFF = desconocido
  const float s = (float)r + (float)g + (float)b;
  if (s <= 0.0f) return 0xFF;
  const float rn = r / s, gn = g / s, bn = b / s;

  uint8_t mejor = 0xFF;
  float d_mejor = 1e9f;
  for (uint8_t i = 0; i < n; i++) {
    const float dr = rn - perfiles[i].r;
    const float dg = gn - perfiles[i].g;
    const float db = bn - perfiles[i].b;
    const float d = sqrtf(dr * dr + dg * dg + db * db);
    if (d < d_mejor && d <= perfiles[i].tol) {
      d_mejor = d;
      mejor = i;
    }
  }
  return mejor;
}

// Detecta el cruce de una linea a partir de la secuencia de colores.
// indice 0 = blanco (por convenio del orden de `perfiles`), 0xFF = desconocido.
class DetectorLinea {
 public:
  DetectorLinea()
      : dentro_(SIN_LINEA), n_(0), t_entrada_(0), t_ultimo_(0),
        contador_(0), ultimo_color_(0) {}

  static const uint8_t SIN_LINEA = 0xFF;

  // Un paso del detector. `color` es el indice del perfil: 0 = blanco,
  // 0xFF = desconocido, el resto son lineas.
  // Devuelve true en el instante en que se completa un cruce.
  //
  // Se emite al SALIR de la linea, no al entrar: asi se sabe cuantas muestras
  // duro y se pueden tirar los destellos de una sola, que son ruido. Una
  // linea de verdad da 2 o mas con 24 ms de integracion.
  bool paso(uint8_t color, uint32_t ahora_ms) {
    const bool es_linea = (color != 0 && color != SIN_LINEA);

    if (es_linea) {
      if (dentro_ == color) {
        if (n_ < 255) n_++;
      } else {
        dentro_ = color;
        t_entrada_ = ahora_ms;
        n_ = 1;
      }
      return false;
    }

    if (dentro_ == SIN_LINEA) return false;    // ya estabamos fuera

    // Se acaba de salir: se decide con lo que quedo guardado y se limpia.
    const uint8_t color_salido = dentro_;
    const uint8_t muestras = n_;
    const uint32_t t0 = t_entrada_;
    dentro_ = SIN_LINEA;
    n_ = 0;

    if (muestras < muestras_min_) return false;
    if (contador_ > 0 && (uint32_t)(t0 - t_ultimo_) < separacion_ms_) return false;

    t_ultimo_ = t0;
    contador_++;
    ultimo_color_ = color_salido;
    return true;
  }

  uint8_t contador() const { return contador_; }
  uint8_t ultimoColor() const { return ultimo_color_; }
  uint8_t muestrasActuales() const { return n_; }

  void reiniciar() {
    contador_ = 0;
    ultimo_color_ = 0;
    dentro_ = SIN_LINEA;
    n_ = 0;
    t_ultimo_ = 0;
  }

  uint8_t muestras_min_ = 2;
  uint32_t separacion_ms_ = 350;

 private:
  uint8_t dentro_;        // indice del color en el que estamos, o SIN_LINEA
  uint8_t n_;             // muestras seguidas de ese color
  uint32_t t_entrada_;
  uint32_t t_ultimo_;
  uint8_t contador_;
  uint8_t ultimo_color_;
};

}  // namespace sens

#endif  // SENSORES_H
