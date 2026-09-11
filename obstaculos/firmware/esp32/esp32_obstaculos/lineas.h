// ===========================================================================
// lineas.h — Clasificador de las lineas naranja y azul del piso.
//
// EL SENSOR NO SABE DE COLORES ABSOLUTOS. El TCS34725 entrega cuatro cuentas
// crudas (C, R, G, B) que dependen de la luz de la sala, de la altura a la
// que quedo el sensor y de la ganancia. Un umbral absoluto calibrado en el
// taller no sobrevive al gimnasio de la competencia.
//
// LO QUE SI ES ESTABLE es la PROPORCION entre canales, normalizada por el
// claro. El tapete blanco da r ~ g ~ b ~ 0.33. Segun el reglamento 2026
// (13.9) la linea naranja es CMYK(0,60,100,0) y la azul CMYK(100,80,0,0):
//
//     naranja  ->  r sube mucho, b se hunde       ->  r - b grande y positivo
//     azul     ->  b sube, r se hunde             ->  r - b grande y negativo
//     blanco   ->  r - b ~ 0
//
// Asi que el discriminante es UN numero, (r-b) normalizado, con el claro como
// puerta de entrada: si el piso no se ha oscurecido, no hay linea que valer.
// El nivel de blanco se APRENDE solo mientras el carro rueda por tapete,
// que es lo que hace que esto aguante cambios de iluminacion.
//
// POR QUE HAY HISTERESIS Y UN MINIMO DE PERMANENCIA
// Sin ellos, el borde de la linea genera una rafaga de entradas y salidas y
// el contador de vueltas se dispara varias veces por cruce. Un cruce cuenta
// UNA vez: al entrar, y no se vuelve a admitir hasta que el sensor haya visto
// tapete limpio durante MS_REARME. A 1 m/s una linea de 20 mm dura 20 ms.
//
// C++ puro: se compila y se prueba en el PC con vectores grabados del carro.
// ===========================================================================
#ifndef LINEAS_H
#define LINEAS_H

#include <stdint.h>

namespace lin {

// Clases, iguales a proto::LINEA_*
static const uint8_t NADA    = 0;
static const uint8_t NARANJA = 1;
static const uint8_t AZUL    = 2;

struct Config {
  // Puerta de claro: se considera "piso oscurecido" por debajo de este
  // porcentaje del blanco aprendido. Las lineas de color reflejan bastante
  // menos que el tapete blanco, pero mucho mas que el muro negro.
  uint8_t pct_entrada = 78;    // entra a linea por debajo del 78 % del blanco
  uint8_t pct_salida  = 88;    // sale por encima del 88 % (histeresis)
  // Discriminante minimo de color, en milesimas de (r-b)/(r+g+b).
  int16_t sep_min = 60;        // 0.060; por debajo es gris, no color
  uint16_t ms_minimo = 8;      // permanencia minima para admitir un cruce
  uint16_t ms_rearme = 120;    // tapete limpio antes de admitir otro cruce
};

class Clasificador {
 public:
  // Salidas publicas, leidas por la tarea de telemetria.
  uint8_t clase = NADA;          // clase confirmada, ahora mismo
  uint8_t cruces_naranja = 0;
  uint8_t cruces_azul = 0;
  uint16_t blanco = 1;           // nivel de claro del tapete, aprendido
  int16_t separacion = 0;        // ultimo discriminante, para depurar

  void configurar(const Config &c) { cfg_ = c; }
  const Config &config() const { return cfg_; }

  // OJO: NO se reinicia `blanco`. Este reinicio lo pide la Pi al pulsar el
  // boton (CAL_CERO_LINEAS) y lo que quiere es poner los contadores a cero,
  // no volver a dejar el sensor ciego diez segundos justo al arrancar la
  // ronda. El nivel de blanco es una propiedad de la sala, no de la carrera.
  void reiniciar() {
    clase = NADA;
    cruces_naranja = 0;
    cruces_azul = 0;
    candidata_ = NADA;
    ms_candidata_ = 0;
    ms_limpio_ = 1000;
  }

  // Umbral de claro para la INT del TCS, en % del blanco aprendido. Al ser
  // relativo no se estropea cuando se cambia el tiempo de integracion.
  uint16_t umbralInterrupcion(uint8_t pct) const {
    uint32_t u = (uint32_t)blanco * pct / 100u;
    if (u < 1) u = 1;
    if (u > 65000) u = 65000;
    return (uint16_t)u;
  }

  // Alimentar con una muestra cruda del TCS. dt_ms = tiempo desde la anterior.
  // Devuelve la clase del cruce RECIEN confirmado (NARANJA/AZUL) o NADA.
  uint8_t paso(uint16_t c, uint16_t r, uint16_t g, uint16_t b, uint16_t dt_ms) {
    if (dt_ms == 0) dt_ms = 1;
    uint32_t suma = (uint32_t)r + g + b;
    separacion = suma ? (int16_t)(((int32_t)r - (int32_t)b) * 1000 / (int32_t)suma) : 0;

    // --- aprendizaje del blanco --------------------------------------
    // LA PRIMERA MUESTRA SE ADOPTA ENTERA. Antes `blanco` arrancaba en 1 y
    // subia con una media movil de 1/64: desde 1 hasta el nivel real del
    // tapete son varios cientos de muestras, o sea del orden de diez segundos
    // en los que `c` esta muy por encima de `blanco` y la puerta de claro
    // nunca se abre. El carro empieza la ronda ciego a las lineas. Tomando la
    // primera lectura como punto de partida, el filtro solo tiene que seguir
    // la deriva de luz de la sala, que es para lo que esta.
    if (!inicializado_) {
      inicializado_ = true;
      if (c > 0) blanco = c;
    }
    // Y solo se aprende cuando NO estamos sobre color: si no, una linea larga
    // se convertiria en el nuevo "blanco" y el sensor se quedaria ciego.
    bool color = (separacion > cfg_.sep_min) || (separacion < -cfg_.sep_min);
    if (!color && c > blanco / 2) {
      // Media movil muy lenta (1/64): sigue la deriva de luz de la sala,
      // no los cambios de un frame.
      blanco = (uint16_t)(((uint32_t)blanco * 63 + c) / 64);
      if (blanco < 1) blanco = 1;
    }

    // --- puerta de claro con histeresis ------------------------------
    uint32_t pct = (uint32_t)c * 100u / (blanco ? blanco : 1);
    bool oscuro = sobre_ ? (pct < cfg_.pct_salida) : (pct < cfg_.pct_entrada);
    sobre_ = oscuro;

    // --- clase instantanea -------------------------------------------
    uint8_t ahora = NADA;
    if (oscuro && color)
      ahora = (separacion > 0) ? NARANJA : AZUL;

    // --- permanencia y rearme ----------------------------------------
    if (ahora == NADA) {
      ms_limpio_ = (ms_limpio_ + dt_ms > 60000) ? 60000 : ms_limpio_ + dt_ms;
      candidata_ = NADA;
      ms_candidata_ = 0;
      clase = NADA;
      return NADA;
    }

    // PERMANENCIA CONTADA EN MILISEGUNDOS, NO EN MUESTRAS.
    //
    // Antes la primera lectura de una clase nueva abria candidata y salia sin
    // mirar el umbral, o sea que hacian falta DOS lecturas seguidas para
    // contar un cruce. Con 12 ms de integracion y una linea de 20 mm caben
    // una o dos lecturas dentro de la linea: exigir dos es exigir el caso
    // bueno siempre, y el dia que solo cae una la linea no se cuenta. Eso no
    // era lo que el filtro queria decir — `ms_minimo` esta en milisegundos —
    // y ademas la ventana de integracion YA es el promediado: una lectura de
    // 12 ms es luz recogida durante 12 ms, no una muestra instantanea que
    // pueda ser ruido.
    if (ahora != candidata_) {          // empieza una candidata nueva
      candidata_ = ahora;
      ms_candidata_ = dt_ms;
    } else {
      ms_candidata_ += dt_ms;
    }
    clase = candidata_;

    // Un cruce solo cuenta si (a) la clase se sostuvo lo suficiente y (b)
    // antes hubo tapete limpio de sobra. Lo segundo es lo que evita contar
    // tres veces la misma linea al pasarla en diagonal.
    if (ms_candidata_ >= cfg_.ms_minimo && ms_limpio_ >= cfg_.ms_rearme) {
      ms_limpio_ = 0;
      if (candidata_ == NARANJA) cruces_naranja++;
      else                       cruces_azul++;
      return candidata_;
    }
    return NADA;
  }

  bool sobreLinea() const { return sobre_; }

 private:
  Config cfg_;
  uint8_t  candidata_ = NADA;
  uint16_t ms_candidata_ = 0;
  uint16_t ms_limpio_ = 1000;
  bool     sobre_ = false;
  bool     inicializado_ = false;
};

}  // namespace lin

#endif  // LINEAS_H
