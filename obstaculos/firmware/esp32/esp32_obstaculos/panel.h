// ===========================================================================
// panel.h — El unico boton y el unico LED que hay en competencia.
//
// REGLAMENTO 2026, PUNTO 9.11: "Una vez encendido el vehiculo, este debe
// permanecer en un estado de espera, esperando a que se presione el boton de
// inicio. Solo se permite UN boton de inicio."
//
// De ahi salen dos decisiones que no son de gusto, son de regla:
//
//   1. El mismo boton arma y desarma (como un cronometro). No hay dos
//      botones, ni un interruptor de modo: eso seria "meter datos al programa
//      mediante la configuracion de interruptores" (9.9) y descalifica.
//
//   2. El boton cuelga del ESP32, no de la Pi, y el CORTE se hace AQUI, en el
//      tick siguiente, sin preguntarle a nadie. Si el boton fuese por USB a
//      la Pi, pararlo dependeria de que Python este vivo, que es justo lo que
//      no se puede suponer cuando hay que pararlo.
//
// El nivel del boton tambien viaja a la Pi en la trama de sensores, pero como
// NIVEL y como CONTADOR DE PULSACIONES, no como evento: si se pierde una
// trama la Pi ve el contador saltar y se entera igual.
//
// PATRONES DEL LED (GPIO 2, el azul de a bordo). En la pista no hay pantalla
// ni web: este LED es lo unico que dice que esta pasando.
//   apagado ............ sin alimentacion
//   latido lento ....... esperando el boton de inicio (estado de espera)
//   triple destello .... calibrando el giroscopio: NO MOVER EL CARRO
//   fijo ............... armado y corriendo
//   parpadeo rapido .... armado pero la Pi lleva rato muda (failsafe)
//   doble destello ..... cortado por el boton, esperando rearme
// ===========================================================================
#ifndef PANEL_H
#define PANEL_H

#include <Arduino.h>

namespace panel {

enum class Luz : uint8_t {
  APAGADO,
  ESPERA,        // latido lento
  CALIBRANDO,    // triple destello
  ARMADO,        // fijo
  SIN_ORDENES,   // parpadeo rapido
  CORTADO        // doble destello
};

// ------------------------------------------------------------------ BOTON --
// Antirrebote por tiempo, no por contador de muestras: el tick de control ya
// es de 10 ms y un MS_REBOTE de 40 ms son cuatro ticks, suficiente para el
// pulsador tactil mas sucio sin que se note al pulsar.
class Boton {
 public:
  static const uint32_t MS_REBOTE = 40;

  void iniciar(int pin) {
    pin_ = pin;
    pinMode(pin_, INPUT_PULLUP);     // pulsador entre el pin y GND
    crudo_ = estable_ = (digitalRead(pin_) == LOW);
    t_cambio_ = millis();
  }

  // Llamar en cada tick de control. Devuelve true en el FLANCO de pulsacion.
  bool paso(uint32_t ahora_ms) {
    bool leido = (digitalRead(pin_) == LOW);
    if (leido != crudo_) { crudo_ = leido; t_cambio_ = ahora_ms; }
    bool flanco = false;
    if ((ahora_ms - t_cambio_) >= MS_REBOTE && estable_ != crudo_) {
      estable_ = crudo_;
      if (estable_) { flanco = true; pulsaciones_++; }
    }
    return flanco;
  }

  bool pisado() const { return estable_; }
  uint8_t pulsaciones() const { return pulsaciones_; }

 private:
  int pin_ = -1;
  bool crudo_ = false, estable_ = false;
  uint32_t t_cambio_ = 0;
  uint8_t pulsaciones_ = 0;
};

// -------------------------------------------------------------------- LED --
// Sin delay(), sin tarea propia: un patron es una mascara de 16 casillas de
// 60 ms que se recorre con millis(). Cambiar de patron es cambiar la mascara,
// asi que nunca hay dos parpadeos peleandose por el pin.
class Led {
 public:
  static const uint32_t MS_CASILLA = 60;

  void iniciar(int pin) {
    pin_ = pin;
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
  }

  void patron(Luz l) { luz_ = l; }

  void paso(uint32_t ahora_ms) {
    if (pin_ < 0) return;
    uint16_t mascara = 0x0000;
    switch (luz_) {
      case Luz::APAGADO:     mascara = 0x0000; break;
      case Luz::ESPERA:      mascara = 0x0003; break;  // ..............XX
      case Luz::CALIBRANDO:  mascara = 0x0155; break;  // X.X.X.X.X (triple+)
      case Luz::ARMADO:      mascara = 0xFFFF; break;  // fijo
      case Luz::SIN_ORDENES: mascara = 0x5555; break;  // rapido alterno
      case Luz::CORTADO:     mascara = 0x0005; break;  // X.X...........
    }
    uint8_t casilla = (uint8_t)((ahora_ms / MS_CASILLA) & 0x0F);
    digitalWrite(pin_, (mascara >> casilla) & 1 ? HIGH : LOW);
  }

 private:
  int pin_ = -1;
  Luz luz_ = Luz::APAGADO;
};

}  // namespace panel

#endif  // PANEL_H
