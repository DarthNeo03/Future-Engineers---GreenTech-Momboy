// ===========================================================================
// lineas.h — Clasificador de las lineas del suelo para el TCS34725.
//
// C++ puro (sin Arduino) para poder probarlo con g++ antes de montar el
// sensor: tools/test_firmware.cpp lo somete a colores reales del tapete WRO.
//
// EN RELATIVO, NO EN ABSOLUTO. Los umbrales fijos de RGB fallan en cuanto
// cambia la luz del pabellon. Aqui se toma primero una muestra del PISO BLANCO
// (boton "calibrar color") y despues solo importa CUANTO se aleja el color
// medido de ese blanco:
//     naranja -> sube el rojo y baja el azul
//     azul    -> sube el azul y baja el rojo
// Asi el mismo umbral vale con luz de tubo, de LED o de ventana.
//
// HISTERESIS: hay que ver N lecturas seguidas del mismo color para declarar
// que se cruzo una linea, y M seguidas de "nada" para soltarla. Sin eso, un
// reflejo en el borde de la linea cuenta dos cruces y el contador de vueltas
// se va al carajo.
// ===========================================================================
#ifndef LINEAS_H
#define LINEAS_H

#include <stdint.h>

namespace lineas {

static const uint8_t NINGUNA = 0;
static const uint8_t NARANJA = 1;
static const uint8_t AZUL    = 2;

struct Config {
  uint16_t luz_min = 300;      // por debajo de esto el sensor esta a oscuras
  uint16_t luz_max = 65000;    // por encima esta cegado (saturado)
  int16_t  umbral = 22;        // cuanto hay que alejarse del blanco (0-255)
  uint8_t  confirmar = 2;      // lecturas seguidas para declarar una linea
  uint8_t  soltar = 3;         // lecturas seguidas de nada para soltarla
};

class Detector {
 public:
  Detector() { reiniciar(); }

  void configurar(const Config &c) {
    cfg_ = c;
    if (cfg_.confirmar < 1) cfg_.confirmar = 1;
    if (cfg_.soltar < 1) cfg_.soltar = 1;
    if (cfg_.umbral < 3) cfg_.umbral = 3;
  }
  const Config &config() const { return cfg_; }

  void reiniciar() {
    estado_ = NINGUNA;
    candidato_ = NINGUNA;
    repeticiones_ = 0;
    rn_ = gn_ = bn_ = 0;
    luz_ = 0;
    calibrado_ = false;
    r0_ = g0_ = b0_ = 85;      // blanco teorico si nadie calibra
  }

  // Guarda el color que hay ahora mismo como "piso blanco".
  void calibrarBlanco() {
    r0_ = rn_; g0_ = gn_; b0_ = bn_;
    calibrado_ = true;
  }
  bool calibrado() const { return calibrado_; }

  // Devuelve true si el estado CAMBIO con esta lectura (es decir, hay evento).
  bool actualizar(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    luz_ = c;
    normalizar(r, g, b, c);

    uint8_t visto = NINGUNA;
    if (c >= cfg_.luz_min && c <= cfg_.luz_max) {
      const int16_t dr = (int16_t)rn_ - (int16_t)r0_;
      const int16_t db = (int16_t)bn_ - (int16_t)b0_;
      if (dr >= cfg_.umbral && db <= -cfg_.umbral / 2)      visto = NARANJA;
      else if (db >= cfg_.umbral && dr <= -cfg_.umbral / 2) visto = AZUL;
    }

    if (visto == candidato_) {
      if (repeticiones_ < 255) repeticiones_++;
    } else {
      candidato_ = visto;
      repeticiones_ = 1;
    }

    const uint8_t hacen_falta = (visto == NINGUNA) ? cfg_.soltar : cfg_.confirmar;
    if (candidato_ != estado_ && repeticiones_ >= hacen_falta) {
      estado_ = candidato_;
      return true;
    }
    return false;
  }

  uint8_t estado() const { return estado_; }
  uint8_t r() const { return rn_; }
  uint8_t g() const { return gn_; }
  uint8_t b() const { return bn_; }
  uint8_t luzComprimida() const { return (uint8_t)(luz_ >> 8); }

 private:
  void normalizar(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    // Dividir por el canal claro quita la intensidad y deja solo el tono:
    // la misma linea naranja a la sombra y al sol da casi los mismos numeros.
    uint32_t den = c ? c : 1;
    uint32_t rr = (uint32_t)r * 255u / den;
    uint32_t gg = (uint32_t)g * 255u / den;
    uint32_t bb = (uint32_t)b * 255u / den;
    rn_ = (uint8_t)(rr > 255 ? 255 : rr);
    gn_ = (uint8_t)(gg > 255 ? 255 : gg);
    bn_ = (uint8_t)(bb > 255 ? 255 : bb);
  }

  Config cfg_;
  uint8_t estado_, candidato_, repeticiones_;
  uint8_t rn_, gn_, bn_;
  uint8_t r0_, g0_, b0_;
  uint16_t luz_;
  bool calibrado_;
};

}  // namespace lineas

#endif  // LINEAS_H
