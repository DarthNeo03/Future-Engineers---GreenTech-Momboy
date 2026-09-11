// ===========================================================================
// seguridad.h — Ultima linea de defensa del hardware.
//
// Todo lo que puede romper una pieza vive aqui, en C++ puro y sin Arduino,
// para poder probarlo con g++ en el PC antes de arriesgar el servo.
//
// TRES REGLAS QUE NADA PUEDE SALTARSE, ni un bug de la Pi ni una trama con
// CRC correcto pero contenido absurdo:
//
//   1. El servo nunca recibe un angulo fuera de [TOPE_MIN, TOPE_MAX]. Esos
//      dos numeros son constantes de COMPILACION; la configuracion que manda
//      la Pi en caliente solo puede ESTRECHAR el rango, jamas ampliarlo.
//   2. El servo nunca salta de golpe: hay limite de grados por segundo. Un
//      MG996R yendo de 65 a 135 grados en un tick arranca la cremallera.
//   3. El motor nunca invierte el giro sin pasar por cero y esperar. Invertir
//      un IBT-2 a plena marcha es un pico de corriente que mete un bajon de
//      tension capaz de reiniciar la Pi.
//
// En el reto de obstaculos la regla 3 importa mas que en el reto abierto,
// porque la maniobra de recuperacion por lado incorrecto SI usa reversa: es
// el unico sitio del programa donde el signo de la velocidad cambia.
// ===========================================================================
#ifndef SEGURIDAD_H
#define SEGURIDAD_H

#include <stdint.h>

namespace seg {

// ---------------------------------------------------------------------------
// TOPES FISICOS ABSOLUTOS DEL MECANISMO DE DIRECCION.
// Medidos a mano con el carro montado y el servo desconectado de la barra.
// Cambiar esto solo despues de volver a medir.
static const int SERVO_TOPE_MIN = 50;
static const int SERVO_TOPE_MAX = 145;
// ---------------------------------------------------------------------------

inline int lim(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline int signo(int v) { return (v > 0) - (v < 0); }

// =============================================================== DIRECCION ==
struct ConfigServo {
  int centro       = 100;   // grados que dejan las ruedas rectas
  int izquierda    = 65;    // tope util a la izquierda (dir = -100)
  int derecha      = 135;   // tope util a la derecha   (dir = +100)
  int gradosPorSeg = 320;   // velocidad maxima de barrido
};

class ControlServo {
 public:
  ControlServo() {
    configurar(ConfigServo());
    actual_ = cfg_.centro;
    saturado_ = false;
  }

  // Acepta configuracion nueva pero la mete a la fuerza dentro de los topes
  // de compilacion, y garantiza izquierda < centro < derecha.
  void configurar(const ConfigServo &c) {
    cfg_.izquierda = lim(c.izquierda, SERVO_TOPE_MIN, SERVO_TOPE_MAX);
    cfg_.derecha   = lim(c.derecha,   SERVO_TOPE_MIN, SERVO_TOPE_MAX);
    if (cfg_.izquierda > cfg_.derecha) {          // vinieron al reves
      int t = cfg_.izquierda; cfg_.izquierda = cfg_.derecha; cfg_.derecha = t;
    }
    cfg_.centro = lim(c.centro, cfg_.izquierda, cfg_.derecha);
    cfg_.gradosPorSeg = lim(c.gradosPorSeg, 20, 2000);
  }

  const ConfigServo &config() const { return cfg_; }

  // Porcentaje con signo -> grados. -100 cae EXACTAMENTE en el tope izquierdo
  // y +100 en el derecho: por construccion no hay forma de pedir mas.
  int anguloDesdePorcentaje(int dirPct) {
    saturado_ = (dirPct > 100 || dirPct < -100);
    dirPct = lim(dirPct, -100, 100);
    long ang;
    if (dirPct >= 0)
      ang = cfg_.centro + (long)dirPct * (cfg_.derecha - cfg_.centro) / 100;
    else
      ang = cfg_.centro + (long)dirPct * (cfg_.centro - cfg_.izquierda) / 100;
    int a = lim((int)ang, cfg_.izquierda, cfg_.derecha);
    return lim(a, SERVO_TOPE_MIN, SERVO_TOPE_MAX);   // doble red, a proposito
  }

  // Avanza hacia 'objetivo' respetando la velocidad maxima. dt en ms.
  int paso(int objetivo, uint32_t dt_ms) {
    objetivo = lim(objetivo, cfg_.izquierda, cfg_.derecha);
    objetivo = lim(objetivo, SERVO_TOPE_MIN, SERVO_TOPE_MAX);
    if (dt_ms == 0) dt_ms = 1;
    int maxPaso = (int)((uint32_t)cfg_.gradosPorSeg * dt_ms / 1000u);
    if (maxPaso < 1) maxPaso = 1;
    int d = objetivo - actual_;
    if (d >  maxPaso) d =  maxPaso;
    if (d < -maxPaso) d = -maxPaso;
    actual_ += d;
    actual_ = lim(actual_, SERVO_TOPE_MIN, SERVO_TOPE_MAX);   // triple red
    return actual_;
  }

  void forzarCentro() { actual_ = cfg_.centro; }
  int  actual()   const { return actual_; }
  bool saturado() const { return saturado_; }

 private:
  ConfigServo cfg_;
  int  actual_;
  bool saturado_;
};

// ================================================================ TRACCION ==
struct ConfigMotor {
  int rampaPorTick = 10;             // cuentas de PWM por tick de 10 ms
  int pwmMinArranque = 0;            // 0 = desactivado; vence la friccion
  uint32_t msFrenoAntesDeInvertir = 150;
};

class ControlMotor {
 public:
  ControlMotor()
      : actual_(0), tCero_(0), ultimoSigno_(0), enCero_(true),
        bloqueada_(false) {}

  void configurar(const ConfigMotor &c) {
    cfg_ = c;
    cfg_.rampaPorTick   = lim(cfg_.rampaPorTick, 1, 255);
    cfg_.pwmMinArranque = lim(cfg_.pwmMinArranque, 0, 255);
  }

  // pedido: PWM con signo -255..255 (ya recortado a vmax por quien llama).
  // ahora_ms: millis(). Devuelve el PWM con signo a escribir en el puente H.
  int paso(int pedido, uint32_t ahora_ms) {
    pedido = lim(pedido, -255, 255);

    // --- proteccion de inversion -------------------------------------
    // Si se pide el sentido contrario al que gira el eje, primero se baja a
    // cero y se espera msFrenoAntesDeInvertir. El motor tiene inercia: el
    // puente H no debe ver dos sentidos en menos de ese tiempo.
    if (pedido != 0 && ultimoSigno_ != 0 && signo(pedido) != ultimoSigno_) {
      bloqueada_ = true;
      pedido = 0;
    } else {
      bloqueada_ = false;
    }
    if (bloqueada_ && enCero_ &&
        (ahora_ms - tCero_) >= cfg_.msFrenoAntesDeInvertir) {
      ultimoSigno_ = 0;       // ya se puede invertir en el proximo tick
      bloqueada_ = false;
    }

    // --- rampa -------------------------------------------------------
    int d = pedido - actual_;
    if (d >  cfg_.rampaPorTick) d =  cfg_.rampaPorTick;
    if (d < -cfg_.rampaPorTick) d = -cfg_.rampaPorTick;
    actual_ += d;
    actual_ = lim(actual_, -255, 255);

    // --- minimo de arranque ------------------------------------------
    // Un PWM de 20 no mueve 500 rpm con la carga del carro: solo calienta el
    // driver y hace zumbar el motor. Si se pide algo, se pide algo util.
    int salida = actual_;
    if (cfg_.pwmMinArranque > 0 && salida != 0 &&
        abs(salida) < cfg_.pwmMinArranque)
      salida = signo(salida) * cfg_.pwmMinArranque;

    // --- memoria del sentido -----------------------------------------
    if (actual_ == 0) {
      if (!enCero_) { enCero_ = true; tCero_ = ahora_ms; }
    } else {
      enCero_ = false;
      ultimoSigno_ = signo(actual_);
    }
    return salida;
  }

  // Parada de emergencia: cero INMEDIATO, sin rampa. Se usa en el failsafe,
  // al desarmar y cuando el boton corta. Aqui la rampa no protege nada: el
  // motor se frena solo contra la inercia del carro, que es poca.
  void cortar(uint32_t ahora_ms) {
    actual_ = 0;
    enCero_ = true;
    tCero_ = ahora_ms;
    bloqueada_ = false;
  }

  int  actual()    const { return actual_; }
  bool bloqueada() const { return bloqueada_; }

 private:
  static int abs(int v) { return v < 0 ? -v : v; }

  ConfigMotor cfg_;
  int      actual_;
  uint32_t tCero_;
  int      ultimoSigno_;
  bool     enCero_;
  bool     bloqueada_;
};

}  // namespace seg

#endif  // SEGURIDAD_H
