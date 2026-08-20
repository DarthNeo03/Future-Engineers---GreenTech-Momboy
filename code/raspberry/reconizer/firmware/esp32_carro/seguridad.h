// ===========================================================================
// seguridad.h — Ultima linea de defensa del hardware.
//
// Todo lo que puede romper una pieza vive aqui, en C++ puro y sin Arduino,
// para poder probarlo con g++ en el PC (tools/test_firmware.cpp).
//
// Tres reglas que NADA puede saltarse, ni un bug de la Pi ni una trama con
// CRC correcto pero contenido absurdo:
//
//   1. El servo nunca recibe un angulo fuera de [TOPE_MIN, TOPE_MAX]. Esos dos
//      numeros son constantes de compilacion; la configuracion en caliente solo
//      puede ESTRECHAR el rango, jamas ampliarlo.
//   2. El servo nunca salta de golpe: hay limite de grados por segundo. Un
//      MG996R yendo de 65 a 135 en un tick arranca la cremallera.
//   3. El motor nunca invierte el giro sin pasar por cero y esperar. Invertir
//      un puente H a plena marcha es un pico de corriente que, ademas, el
//      reglamento WRO limita a 4 A.
// ===========================================================================
#ifndef SEGURIDAD_H
#define SEGURIDAD_H

#include <stdint.h>

namespace seg {

// ---------------------------------------------------------------------------
// TOPES FISICOS ABSOLUTOS DEL MECANISMO DE DIRECCION.
// Medidos a mano con el carro montado. Cambiar esto solo despues de volver a
// medir con el servo desconectado de la barra.
static const int SERVO_TOPE_MIN = 50;
static const int SERVO_TOPE_MAX = 145;
// ---------------------------------------------------------------------------

inline int lim(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct ConfigServo {
  int centro      = 100;   // recto
  int izquierda   = 65;    // tope util a la izquierda (dir = -100)
  int derecha     = 135;   // tope util a la derecha  (dir = +100)
  int gradosPorSeg = 320;  // velocidad maxima de barrido
};

class ControlServo {
 public:
  ControlServo() { configurar(ConfigServo()); actual_ = cfg_.centro; saturado_ = false; }

  // Acepta configuracion nueva pero la mete a la fuerza dentro de los topes de
  // compilacion y garantiza izquierda < centro < derecha.
  void configurar(const ConfigServo &c) {
    cfg_.izquierda = lim(c.izquierda, SERVO_TOPE_MIN, SERVO_TOPE_MAX);
    cfg_.derecha   = lim(c.derecha,   SERVO_TOPE_MIN, SERVO_TOPE_MAX);
    if (cfg_.izquierda > cfg_.derecha) {           // invertidos: los ordena
      int t = cfg_.izquierda; cfg_.izquierda = cfg_.derecha; cfg_.derecha = t;
    }
    cfg_.centro = lim(c.centro, cfg_.izquierda, cfg_.derecha);
    cfg_.gradosPorSeg = lim(c.gradosPorSeg, 20, 2000);
  }

  const ConfigServo &config() const { return cfg_; }

  // Porcentaje con signo -> grados. -100 cae exactamente en el tope izquierdo
  // y +100 en el derecho, asi que por construccion no hay forma de pedir mas.
  int anguloDesdePorcentaje(int dirPct) {
    saturado_ = (dirPct > 100 || dirPct < -100);
    dirPct = lim(dirPct, -100, 100);
    long ang;
    if (dirPct >= 0) ang = cfg_.centro + (long)dirPct * (cfg_.derecha - cfg_.centro) / 100;
    else             ang = cfg_.centro + (long)dirPct * (cfg_.centro - cfg_.izquierda) / 100;
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

// ---------------------------------------------------------------------------
struct ConfigMotor {
  int rampaPorTick = 10;            // cuentas de PWM que puede cambiar por tick
  int pwmMinArranque = 0;           // 0 = desactivado
  uint32_t msFrenoAntesDeInvertir = 150;
};

inline int signo(int v) { return (v > 0) - (v < 0); }

class ControlMotor {
 public:
  ControlMotor()
      : actual_(0), tCero_(0), ultimoSigno_(0), enCero_(true), bloqueada_(false) {}

  void configurar(const ConfigMotor &c) {
    cfg_ = c;
    cfg_.rampaPorTick = lim(cfg_.rampaPorTick, 1, 255);
    cfg_.pwmMinArranque = lim(cfg_.pwmMinArranque, 0, 255);
  }

  // pedido: PWM con signo -255..255 (ya recortado a vmax por quien llama).
  // Devuelve el PWM con signo que hay que escribir en el puente H.
  //
  // La proteccion se mide contra el ULTIMO SENTIDO REAL de giro, no contra el
  // PWM instantaneo: en cuanto el PWM llega a cero el motor sigue girando por
  // inercia unas decimas, y es justo ahi donde invertir el puente H mete el
  // pico de corriente. Por eso hace falta el reloj, no basta con "actual == 0".
  int paso(int pedido, uint32_t ahora_ms) {
    pedido = lim(pedido, -255, 255);
    if (cfg_.pwmMinArranque > 0 && pedido != 0) {
      int mag = pedido < 0 ? -pedido : pedido;
      if (mag < cfg_.pwmMinArranque)
        pedido = pedido < 0 ? -cfg_.pwmMinArranque : cfg_.pwmMinArranque;
    }

    bloqueada_ = false;
    const int sp = signo(pedido);
    if (sp != 0 && ultimoSigno_ != 0 && sp != ultimoSigno_) {
      if (actual_ != 0) {
        pedido = 0;                                   // 1) frenar hasta cero
        bloqueada_ = true;
      } else if ((uint32_t)(ahora_ms - tCero_) < cfg_.msFrenoAntesDeInvertir) {
        pedido = 0;                                   // 2) esperar quieto
        bloqueada_ = true;
      } else {
        ultimoSigno_ = 0;                             // 3) permitido invertir
      }
    }

    int d = pedido - actual_;
    if (d >  cfg_.rampaPorTick) d =  cfg_.rampaPorTick;
    if (d < -cfg_.rampaPorTick) d = -cfg_.rampaPorTick;
    actual_ = lim(actual_ + d, -255, 255);

    if (actual_ == 0) {
      if (!enCero_) { enCero_ = true; tCero_ = ahora_ms; }
    } else {
      enCero_ = false;
      ultimoSigno_ = signo(actual_);
    }
    return actual_;
  }

  // Parada de emergencia: corta sin rampa. Mantiene el ultimo sentido, asi que
  // despues de un corte de emergencia la inversion sigue exigiendo su pausa.
  void cortar(uint32_t ahora_ms) {
    actual_ = 0;
    enCero_ = true;
    tCero_ = ahora_ms;
    bloqueada_ = false;
  }

  int  actual() const { return actual_; }
  bool inversionBloqueada() const { return bloqueada_; }

 private:
  ConfigMotor cfg_;
  int actual_;
  uint32_t tCero_;
  int ultimoSigno_;
  bool enCero_;
  bool bloqueada_;
};

// ---------------------------------------------------------------------------
// Convierte el % de velocidad de la Pi a PWM respetando el tope vmax.
inline int pwmDesdePorcentaje(int velPct, uint8_t vmax) {
  velPct = lim(velPct, -100, 100);
  return (int)((long)velPct * (long)vmax / 100L);
}

}  // namespace seg

#endif  // SEGURIDAD_H
