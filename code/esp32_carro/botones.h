// ===========================================================================
// botones.h — El pulsador de competencia y el LED de estado del ESP32.
//
// El reglamento WRO Future Engineers exige que la ronda EMPIECE con una sola
// accion sobre el robot ya colocado en la pista: nada de teclado, ni pantalla,
// ni web, ni cable. UN SOLO PULSADOR hace las dos cosas, como el start/stop de
// un cronometro:
//
//     pulsacion con el carro parado  ->  ARMA y arranca la ronda
//     pulsacion con el carro armado  ->  DESARMA y para
//
// El pulsador cuelga del ESP32 (que tiene los pines libres y el bucle de
// 100 Hz) y su estado viaja a la Pi en la trama de sensores; la Pi es quien
// decide, porque es quien sabe si el carro estaba armado.
//
// CABLEADO (dos cables, ninguna resistencia):
//
//     GPIO13 ---[ pulsador ]--- GND
//
// con el pull-up interno: en reposo el pin lee ALTO y al pulsar cae a BAJO.
// El pin es una constante del firmware, como los del motor: no se configura
// desde la Pi. El 13 no es pin de arranque (strapping) ni de entrada-sola,
// asi que un pulsador pisado al encender no impide el boot.
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
// LA EMERGENCIA CORTA AQUI, NO EN LA PI
// Con un solo boton, el ESP32 no sabe que quiere decir una pulsacion... salvo
// en el caso que importa: si el carro esta ARMADO, lo unico que puede
// significar es PARAR. Nadie pulsa para armar un carro que ya va andando. Asi
// que en ese caso se enclava un corte local y el motor se para en el
// siguiente tick de 10 ms, sin esperar el viaje de ida y vuelta por el serial
// (~50 ms) y aunque el programa de la Pi este colgado mandando "adelante".
// El enclavamiento se suelta cuando la Pi acusa recibo (manda un mando
// desarmado) y ademas el dedo ya no esta encima. Con el carro parado la
// pulsacion no corta nada: es la de arrancar, y esa la interpreta la Pi.
//
// EL LED DE ESTADO (el azul de a bordo, GPIO2 en casi todas las placas)
// Sin web es lo unico que dice desde fuera que esta pasando. Cuatro patrones
// que se distinguen de un vistazo a dos metros:
//
//     fijo encendido      ARMADO: el carro puede salir corriendo AHORA
//     un destello corto   listo y desarmado, con la Pi hablando
//     parpadeo rapido     sin ordenes de la Pi (failsafe): programa caido,
//                         cable suelto o Pi todavia arrancando
//     dos destellos       cortado por el pulsador (emergencia enclavada)
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
// El pulsador mas el enclavamiento del corte local.
class Panel {
 public:
  void iniciar(uint8_t pin) {
    puls_.iniciar(pin);
    corte_ = false;
    previo_ = false;
  }

  // Tick de 100 Hz. 'piArmada' = el ultimo mando recibido viene armado, o sea
  // que el carro puede estar moviendose. Devuelve true si el motor tiene que
  // estar cortado por el boton.
  bool paso(uint32_t ahora, bool piArmada) {
    const bool pisado = puls_.paso(ahora);
    // Flanco de pulsacion CON EL CARRO ARMADO: solo puede querer decir parar.
    if (pisado && !previo_ && piArmada) corte_ = true;
    previo_ = pisado;
    // Se suelta cuando la Pi acusa recibo (desarma) y ademas el dedo ya no
    // esta encima: soltarlo con el boton pisado dejaria el carro listo para
    // salir en cuanto la Pi rearmara.
    if (corte_ && !piArmada && !pisado) corte_ = false;
    return corte_;
  }

  bool pisado() const { return puls_.pisado(); }
  bool cortando() const { return corte_; }

 private:
  Pulsador puls_;
  bool corte_ = false;
  bool previo_ = false;
};

// --------------------------------------------------------------------------
// LED de estado. Cada patron son 16 ranuras de 100 ms (ciclo de 1.6 s): el
// bit N dice si el LED esta encendido en la ranura N. Se lee del reloj, sin
// estado propio, asi que da igual quien lo llame y cada cuanto.
static const uint16_t LED_ARMADO  = 0xFFFF;   // fijo encendido
static const uint16_t LED_LISTO   = 0x0001;   // un destello de 100 ms
static const uint16_t LED_SIN_PI  = 0x5555;   // parpadeo rapido (5 Hz)
static const uint16_t LED_CORTE   = 0x0005;   // dos destellos y pausa

inline bool ledEncendido(uint16_t patron, uint32_t ahora) {
  return ((patron >> ((ahora / 100u) % 16u)) & 1u) != 0;
}

}  // namespace bot

#endif  // BOTONES_H
