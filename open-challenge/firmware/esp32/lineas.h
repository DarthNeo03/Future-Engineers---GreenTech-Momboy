// ===========================================================================
// lineas.h — Clasificador de las lineas del piso (naranja / azul) del TCS34725.
//
// C++ puro (sin Arduino) a proposito: se puede compilar con g++ en el PC y
// probar sin el carro (tools de piloto).
//
// COMO CLASIFICA
// El TCS34725 entrega C (canal claro) y R, G, B crudos. Los valores absolutos
// cambian con la luz del pabellon, pero los RATIOS r = R*255/C y b = B*255/C
// casi no. Sobre piso blanco r ~ b ~ 85 (un tercio cada canal).
//
// Lo que MANDA es la DIFERENCIA entre los dos ratios, no su valor absoluto:
//
//     dif = b - r      ~0 en el blanco, muy positivo en azul, muy negativo
//                      en naranja
//
// Medido en la pista del equipo sobre la linea azul: C=678 R=186 B=284, o sea
// r=70 y b=107. El azul solo saca 22 puntos al blanco en su propio canal (107
// contra ~85) -- margen tan fino que con un umbral absoluto de 110 la linea
// azul NO se detectaba -- pero saca 37 en la diferencia (b-r = +37 contra ~0).
// Por eso la diferencia es el criterio principal y los umbrales absolutos se
// quedan como reja de seguridad, no como discriminador:
//
//     naranja: (r - b) >= naranja_dif_min  &&  r >= naranja_r_min  &&  b <= naranja_b_max
//     azul:    (b - r) >= azul_dif_min     &&  b >= azul_b_min     &&  r <= azul_r_max
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
  uint8_t  naranja_dif_min = 30;   // (r-b) minimo del naranja  <- discriminador
  uint8_t  azul_dif_min    = 18;   // (b-r) minimo del azul     <- discriminador
  uint8_t  naranja_r_min  = 110;   // rejas de seguridad, no discriminadores:
  uint8_t  naranja_b_max  = 90;    //   holgadas a proposito, para que sea la
  uint8_t  azul_b_min     = 95;    //   diferencia la que decida
  uint8_t  azul_r_max     = 95;
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
    const int32_t rr = (int32_t)((uint32_t)r * 255u / c);
    const int32_t rb = (int32_t)((uint32_t)b * 255u / c);
    const int32_t dif = rb - rr;        // >0 azulado, <0 anaranjado
    if (-dif >= (int32_t)cfg_.naranja_dif_min &&
        rr >= (int32_t)cfg_.naranja_r_min &&
        rb <= (int32_t)cfg_.naranja_b_max) return NARANJA;
    if (dif >= (int32_t)cfg_.azul_dif_min &&
        rb >= (int32_t)cfg_.azul_b_min &&
        rr <= (int32_t)cfg_.azul_r_max) return AZUL;
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
