// ---------------------------------------------------------------------------
// tcs34725.h  -  Driver minimo + detector de lineas naranja / azul del tapete
//
// El tapete es blanco; las lineas de esquina son naranja CMYK(0,60,100,0) y
// azul CMYK(100,80,0,0), de 20 mm de ancho.  El sensor se monta mirando al
// suelo a 5-15 mm con una falda que tape la luz lateral.
//
// Estrategia: se normaliza R,G,B contra la suma (r+g+b) para independizarse
// del nivel de iluminacion, y ademas se mantiene una referencia lenta de
// "blanco" que el robot actualiza solo mientras ve tapete blanco. Esto compensa
// diferencias de luz entre sedes sin que el equipo toque nada (regla 9.9).
// ---------------------------------------------------------------------------
#pragma once
#include <Wire.h>
#include "config.h"

#define LINE_NONE   0
#define LINE_ORANGE 1
#define LINE_BLUE   2

class Tcs34725 {
public:
  bool  ok = false;

  // ---- Lecturas crudas y normalizadas (visibles en la telemetria) ----
  uint16_t cR = 0, cG = 0, cB = 0, cC = 0;
  float    rn = 0.33f, gn = 0.33f, bn = 0.33f;

  // ---- Parametros ajustables desde la Pi (comando P) ----
  float th_orange_r = 0.44f;  // r normalizado minimo para naranja
  float th_orange_b = 0.22f;  // b normalizado maximo para naranja
  float th_blue_b   = 0.42f;  // b normalizado minimo para azul
  float th_blue_r   = 0.26f;  // r normalizado maximo para azul
  uint16_t th_clear_min = 60; // C minimo: por debajo se considera lectura invalida
  uint8_t  confirm_n    = 2;  // muestras consecutivas para validar un evento
  uint16_t refractory_ms = 220; // tiempo minimo entre dos eventos de linea

  // ---- Estado del detector ----
  uint8_t  current   = LINE_NONE;  // color visto ahora mismo
  uint8_t  last_event= LINE_NONE;  // ultimo evento confirmado
  uint16_t n_orange  = 0;
  uint16_t n_blue    = 0;
  uint32_t seq       = 0;          // se incrementa en cada evento nuevo

  bool begin() {
    uint8_t id = 0;
    if (!readRegs(0x12, &id, 1)) return false;
    // 0x44 = TCS34725/TCS34721 ; 0x4D = TCS34727/TCS34723
    if (id != 0x44 && id != 0x4D && id != 0x10) return false;
    writeReg(0x01, 0xFB);   // ATIME: 5 ciclos -> 12 ms de integracion (~83 Hz)
    writeReg(0x0F, 0x01);   // CONTROL: ganancia 4x
    writeReg(0x00, 0x01);   // ENABLE: PON
    delay(3);
    writeReg(0x00, 0x03);   // ENABLE: PON | AEN
    delay(15);
    ok = true;
    return true;
  }

  void update(uint32_t now_ms) {
    if (!ok) return;
    uint8_t b[8];
    // 0xA0 = bit de comando (0x80) + auto-incremento (0x20); 0x14 = CDATAL
    if (!readRegs(0x14, b, 8)) return;
    cC = (uint16_t)(b[0] | (b[1] << 8));
    cR = (uint16_t)(b[2] | (b[3] << 8));
    cG = (uint16_t)(b[4] | (b[5] << 8));
    cB = (uint16_t)(b[6] | (b[7] << 8));

    uint32_t sum = (uint32_t)cR + cG + cB;
    if (cC < th_clear_min || sum < 30) { current = LINE_NONE; run_ = 0; return; }

    // Normalizacion contra la referencia de blanco aprendida
    float r = (float)cR / wr_, g = (float)cG / wg_, bl = (float)cB / wb_;
    float s = r + g + bl;
    if (s <= 0.0001f) { current = LINE_NONE; run_ = 0; return; }
    rn = r / s; gn = g / s; bn = bl / s;

    uint8_t seen = LINE_NONE;
    if (rn >= th_orange_r && bn <= th_orange_b)      seen = LINE_ORANGE;
    else if (bn >= th_blue_b && rn <= th_blue_r)     seen = LINE_BLUE;

    if (seen == LINE_NONE) {
      // Tapete blanco: refresca muy despacio la referencia de blanco.
      if (rn > 0.26f && rn < 0.42f && bn > 0.24f && bn < 0.42f && cC > 200) {
        wr_ += ((float)cR - wr_) * 0.002f;
        wg_ += ((float)cG - wg_) * 0.002f;
        wb_ += ((float)cB - wb_) * 0.002f;
        if (wr_ < 1) wr_ = 1;  if (wg_ < 1) wg_ = 1;  if (wb_ < 1) wb_ = 1;
      }
      run_ = 0;
      current = LINE_NONE;
      return;
    }

    current = seen;
    if (seen == run_color_) run_++;
    else { run_color_ = seen; run_ = 1; }

    if (run_ >= confirm_n && (now_ms - last_ms_) > refractory_ms) {
      last_ms_    = now_ms;
      last_event  = seen;
      seq++;
      if (seen == LINE_ORANGE) n_orange++; else n_blue++;
    }
  }

  void resetCounters() {
    n_orange = 0; n_blue = 0; seq = 0; last_event = LINE_NONE;
  }

private:
  float    wr_ = 100.0f, wg_ = 100.0f, wb_ = 100.0f;  // referencia de blanco
  uint8_t  run_ = 0, run_color_ = LINE_NONE;
  uint32_t last_ms_ = 0;

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCS_ADDR);
    Wire.write(0x80 | reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(TCS_ADDR);
    Wire.write(0xA0 | reg);            // comando + auto-incremento
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)TCS_ADDR, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
  }
};
