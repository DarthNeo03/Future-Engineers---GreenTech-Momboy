// ===========================================================================
// lineas.h — Clasificador de las lineas del piso (naranja / azul) del TCS34725.
//
// C++ puro (sin Arduino) a proposito: se puede compilar con g++ en el PC y
// probar sin el carro (tools de piloto).
//
// COMO CLASIFICA
// El TCS34725 entrega C (canal claro) y R, G, B crudos. Los valores absolutos
// cambian con la luz del pabellon, pero los RATIOS r = R*255/C y b = B*255/C
// casi no. Sobre piso blanco r ~ b ~ 85 (un tercio cada canal). Sobre la linea
// naranja el ratio rojo sube y el azul cae; sobre la azul, al reves:
//
//     naranja: r >= naranja_r_min  &&  b <= naranja_b_max
//     azul:    b >= azul_b_min     &&  r <= azul_r_max
//
// y ademas C >= c_min (si hay muy poca luz es sombra o el sensor tapado).
//
// COMO CUENTA
// Una linea de ~20 mm a 0.5 m/s pasa bajo el sensor en ~40 ms: 1-2 lecturas.
// Por eso el cruce se LATCHEA aqui, a la frecuencia del sensor, y viaja como
// contador (4 bits, envuelve): la Pi solo mira si el contador avanzo.
//   - muestras_min lecturas seguidas de la misma clase => cruce confirmado
//   - refractario_ms sin admitir OTRO cruce del mismo color (la misma linea
//     no puede contar dos veces por una lectura ruidosa en el borde).
// ===========================================================================
#ifndef LINEAS_H
#define LINEAS_H

#include <stdint.h>

namespace lin {

struct Config {
  uint16_t c_min          = 80;    // por debajo, sombra/borde: no clasificar
  uint8_t  naranja_r_min  = 120;   // ratio rojo (0..255) minimo del naranja
  uint8_t  naranja_b_max  = 60;    // ratio azul maximo del naranja
  uint8_t  azul_b_min     = 110;   // ratio azul minimo del azul
  uint8_t  azul_r_max     = 70;    // ratio rojo maximo del azul
  uint8_t  muestras_min   = 1;     // lecturas seguidas para confirmar
  uint16_t refractario_ms = 300;   // sin repetir el mismo color
};

static const uint8_t NADA = 0;
static const uint8_t NARANJA = 1;
static const uint8_t AZUL = 2;

class Clasificador {
 public:
  Clasificador() { reiniciar(); }

  void configurar(const Config &c) { cfg_ = c; }
  const Config &config() const { return cfg_; }

  void reiniciar() {
    clase_ = NADA;
    seguidas_ = 0;
    cntNaranja_ = 0;
    cntAzul_ = 0;
    msNaranja_ = 0;
    msAzul_ = 0;
    latch_ = NADA;
  }

  // Clase instantanea de una lectura, sin memoria.
  uint8_t clasificar(uint16_t c, uint16_t r, uint16_t g, uint16_t b) const {
    (void)g;
    if (c < cfg_.c_min || c == 0) return NADA;
    const uint32_t rr = (uint32_t)r * 255u / c;
    const uint32_t rb = (uint32_t)b * 255u / c;
    if (rr >= cfg_.naranja_r_min && rb <= cfg_.naranja_b_max) return NARANJA;
    if (rb >= cfg_.azul_b_min && rr <= cfg_.azul_r_max) return AZUL;
    return NADA;
  }

  // Alimentar una lectura nueva. Devuelve la clase instantanea.
  // Los contadores de cruce avanzan solos; leerlos con contadores().
  uint8_t paso(uint16_t c, uint16_t r, uint16_t g, uint16_t b, uint32_t ahora_ms) {
    const uint8_t cl = clasificar(c, r, g, b);

    if (cl == clase_ && cl != NADA) {
      if (seguidas_ < 255) seguidas_++;
    } else if (cl != NADA) {
      clase_ = cl;
      seguidas_ = 1;
    } else {
      clase_ = NADA;
      seguidas_ = 0;
      latch_ = NADA;             // salimos de la linea: se puede latchear otra
    }

    if (clase_ != NADA && seguidas_ >= cfg_.muestras_min && latch_ != clase_) {
      // Cruce confirmado. El refractario evita contar la MISMA linea dos
      // veces si el borde parpadea entre linea y piso.
      if (clase_ == NARANJA) {
        if ((uint32_t)(ahora_ms - msNaranja_) >= cfg_.refractario_ms) {
          cntNaranja_ = (uint8_t)((cntNaranja_ + 1) & 0x0F);
          msNaranja_ = ahora_ms;
        }
      } else {
        if ((uint32_t)(ahora_ms - msAzul_) >= cfg_.refractario_ms) {
          cntAzul_ = (uint8_t)((cntAzul_ + 1) & 0x0F);
          msAzul_ = ahora_ms;
        }
      }
      latch_ = clase_;
    }
    return clase_;
  }

  uint8_t clase() const { return clase_; }
  bool sobreLinea() const { return clase_ != NADA; }

  // naranja en los 4 bits bajos, azul en los 4 altos (formato de la trama).
  uint8_t contadores() const {
    return (uint8_t)((cntNaranja_ & 0x0F) | ((cntAzul_ & 0x0F) << 4));
  }

 private:
  Config cfg_;
  uint8_t clase_, seguidas_, latch_;
  uint8_t cntNaranja_, cntAzul_;
  uint32_t msNaranja_, msAzul_;
};

}  // namespace lin

#endif  // LINEAS_H
