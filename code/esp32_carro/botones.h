// ===========================================================================
// botones.h — Los dos pulsadores de competencia, colgados del ESP32.
//
// El reglamento WRO Future Engineers exige que la ronda EMPIECE con una sola
// accion sobre el robot ya colocado en la pista: nada de teclado, ni pantalla,
// ni web, ni cable. Los pulsadores van al ESP32 (que es quien tiene los pines
// libres y el bucle de 100 Hz) y su estado viaja a la Pi en la trama de
// sensores; la Pi decide QUE significan.
//
//   ARRANQUE  el "start" del reglamento. Lo interpreta la Pi.
//   PARO      la parada de emergencia. La interpreta la Pi... y ADEMAS corta
//             aqui mismo (ver mas abajo).
//
// CABLEADO (dos cables por pulsador, ninguna resistencia):
//
//     GPIO13 ---[ pulsador ARRANQUE ]--- GND
//     GPIO23 ---[ pulsador PARO     ]--- GND
//
// con el pull-up interno: en reposo el pin lee ALTO y al pulsar cae a BAJO.
// Los pines son constantes del firmware, como los del motor: no se configuran
// desde la Pi. Se han elegido dos que no son de arranque (strapping) ni de
// entrada-sola, para que un pulsador pisado al encender no impida el boot.
//
// POR QUE NIVEL Y NO CONTADOR (al reves que los cruces de linea)
// Una linea se cruza en 40 ms a velocidad de carrera: si no se latchea y se
// cuenta, se pierde. Un dedo, en cambio, aguanta el pulsador 100-300 ms, o
// sea 4-12 tramas de sensores: el nivel llega de sobra. Y hay una razon de
// seguridad que decide el empate: un contador guarda pulsaciones pendientes,
// asi que un corte del serial con una pulsacion sin entregar podria arrancar
// el carro AL RECONECTAR, solo. Con nivel, lo que se pierde no se ejecuta:
// el fallo es "no pasa nada", que delante de un juez es el fallo bueno.
//
// EL PARO CORTA AQUI, NO EN LA PI
// El resto del sistema tiene la emergencia a un viaje de ida y vuelta por el
// serial (~50 ms) y depende de que el programa de la Pi este sano. Teniendo
// el pulsador en el ESP32 seria absurdo no usarlo: al confirmarse la
// pulsacion se enclava un corte local que para el motor en el siguiente tick
// de 10 ms, aunque la Pi este colgada mandando "adelante". El enclavamiento
// se suelta cuando la Pi acusa recibo mandando un mando DESARMADO, que es
// justo lo que hace al enterarse del boton. Asi el operador no tiene que
// hacer nada raro para volver a armar: pulsa ARRANQUE y ya.
// ===========================================================================
#ifndef BOTONES_H
#define BOTONES_H

#include <Arduino.h>
#include <stdint.h>

namespace bot {

// Antirrebote. Los contactos de un pulsador barato rebotan entre 1 y 10 ms;
// 30 ms se los come todos y sigue sin notarse al pulsar.
static const uint16_t ANTIRREBOTE_MS = 30;

// --------------------------------------------------------------------------
// Un pulsador con antirrebote. No decide nada: solo dice si esta pisado.
class Pulsador {
 public:
  void iniciar(uint8_t pin) {
    pin_ = pin;
    pinMode(pin_, INPUT_PULLUP);
    crudo_ = estable_ = false;      // en reposo el pin esta ALTO = suelto
    msCambio_ = millis();
  }

  // Llamar a ritmo fijo (el tick de control, 100 Hz). Devuelve el nivel ya
  // estable: true = pisado.
  bool paso(uint32_t ahora) {
    // pull-up: pisado = pin a BAJO
    const bool leido = (digitalRead(pin_) == LOW);
    if (leido != crudo_) {
      crudo_ = leido;
      msCambio_ = ahora;
    } else if (crudo_ != estable_ && (ahora - msCambio_) >= ANTIRREBOTE_MS) {
      estable_ = crudo_;
    }
    return estable_;
  }

  bool pisado() const { return estable_; }

 private:
  uint8_t pin_ = 255;
  bool crudo_ = false, estable_ = false;
  uint32_t msCambio_ = 0;
};

// --------------------------------------------------------------------------
// Los dos pulsadores juntos, mas el enclavamiento del corte local de PARO.
class Panel {
 public:
  void iniciar(uint8_t pinArranque, uint8_t pinParo) {
    arranque_.iniciar(pinArranque);
    paro_.iniciar(pinParo);
    corte_ = false;
    paroPrevio_ = false;
  }

  // Tick de 100 Hz. 'piDesarmada' = el ultimo mando recibido viene sin armar,
  // o sea que la Pi ya se entero y no hace falta seguir cortando por nuestra
  // cuenta. Devuelve true si el motor tiene que estar cortado por el boton.
  bool paso(uint32_t ahora, bool piDesarmada) {
    arranque_.paso(ahora);
    const bool paro = paro_.paso(ahora);
    if (paro && !paroPrevio_) corte_ = true;    // flanco de pulsacion: enclava
    paroPrevio_ = paro;
    // Se suelta cuando la Pi acusa recibo (manda desarmado) y ademas el dedo
    // ya no esta encima: si no, soltarlo con el boton pisado dejaria el carro
    // listo para salir en cuanto la Pi rearme.
    if (corte_ && piDesarmada && !paro) corte_ = false;
    return corte_;
  }

  bool arranquePisado() const { return arranque_.pisado(); }
  bool paroPisado() const { return paro_.pisado(); }
  bool cortando() const { return corte_; }

 private:
  Pulsador arranque_, paro_;
  bool corte_ = false;
  bool paroPrevio_ = false;
};

}  // namespace bot

#endif  // BOTONES_H
