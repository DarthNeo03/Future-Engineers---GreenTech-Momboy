// ===========================================================================
// protocolo.h — Trama binaria Raspberry Pi 5 <-> ESP32 (reto de OBSTACULOS).
//
// GEMELO EXACTO de raspberry-pi/src/protocolo.py. Si tocas uno, toca el otro:
// tools/selftest.py cruza vectores de los dos y falla si se separan.
//
//      A5 5A | LEN | TIPO | payload (LEN bytes) | CRC8
//
// POR QUE BINARIO Y NO TEXTO
// A 115200 baudios un byte cuesta ~87 us. La trama de mando ocupa 11 bytes,
// o sea ~0.96 ms de linea. En ASCII ("V=35,D=-18\n") serian tres o cuatro
// veces mas, y ademas habria que parsear numeros dentro del lazo de control.
// Con el mando a 50 Hz el enlace queda al 5 % de ocupacion: no hay cola, no
// hay latencia acumulada, y el failsafe del ESP32 mide silencio de verdad y
// no congestion del puerto.
//
// POR QUE CRC Y NO SUMA DE COMPROBACION
// El cable del motor va a 30 cm del cable serial. Un checksum de suma no
// detecta dos bits cambiados dentro del mismo byte; el CRC-8/ATM si. Una
// trama de direccion corrupta que pase el filtro significa volante al tope.
//
// C++ puro (solo stdint/string.h) a proposito: se compila con g++ en el PC y
// se prueba sin subir nada al ESP32.
// ===========================================================================
#ifndef PROTOCOLO_H
#define PROTOCOLO_H

#include <stdint.h>
#include <string.h>

namespace proto {

static const uint8_t SYNC1 = 0xA5;
static const uint8_t SYNC2 = 0x5A;
static const uint8_t MAX_PAYLOAD = 24;   // la de sensores usa 16
static const uint8_t VERSION_PROTOCOLO = 1;

// ----------------------------------------------------------- tipos de trama
static const uint8_t TIPO_MANDO  = 0x01;   // Pi -> ESP32,  6 bytes
static const uint8_t TIPO_PING   = 0x02;   // Pi -> ESP32,  1 byte
static const uint8_t TIPO_CONFIG = 0x03;   // Pi -> ESP32,  7 bytes
static const uint8_t TIPO_CAL    = 0x06;   // Pi -> ESP32,  1 byte
static const uint8_t TIPO_TELE   = 0x81;   // ESP32 -> Pi,  8 bytes
static const uint8_t TIPO_LOG    = 0x82;   // ESP32 -> Pi,  texto
static const uint8_t TIPO_PONG   = 0x83;   // ESP32 -> Pi,  1 byte
static const uint8_t TIPO_SENS   = 0x84;   // ESP32 -> Pi, 16 bytes

// -------------------------------------------------------- banderas de mando
static const uint8_t F_ARMADO  = 0x01;   // sin esto el motor no gira, punto
static const uint8_t F_PARADA  = 0x02;   // parada inmediata
static const uint8_t F_CENTRAR = 0x04;   // servo al centro, ignora dir
static const uint8_t F_LIMPIAR = 0x08;   // rearmar tras un corte por boton

// --------------------------------------------------- bits de la telemetria
static const uint8_t E_ARMADO        = 0x01;
static const uint8_t E_MOTOR         = 0x02;
static const uint8_t E_FAILSAFE      = 0x04;   // la Pi lleva FAILSAFE_MS muda
static const uint8_t E_SERVO_TOPE    = 0x08;   // se pidio mas de lo que hay
static const uint8_t E_INV_BLOQUEADA = 0x10;   // inversion esperando el cero

// ------------------------------------------- bits de la trama de sensores
static const uint8_t S_MPU_OK      = 0x01;
static const uint8_t S_TCS_OK      = 0x02;
static const uint8_t S_CALIBRANDO  = 0x04;   // yaw congelado: NO MOVER
static const uint8_t S_SOBRE_LINEA = 0x08;
static const uint8_t S_MPU_INT     = 0x10;   // la pata INT del MPU da flancos
static const uint8_t S_TCS_INT     = 0x20;   // la pata INT del TCS responde
// Los bits 0x40 y 0x80 llevan la clase de linea que hay bajo el carro.
static const uint8_t LINEA_NADA    = 0;
static const uint8_t LINEA_NARANJA = 1;
static const uint8_t LINEA_AZUL    = 2;

// ------------------------------------------------- bits del byte de boton
static const uint8_t B_NIVEL = 0x01;   // pulsador pisado, ya con antirrebote
static const uint8_t B_CORTE = 0x02;   // el ESP32 corto la traccion el solo

// ------------------------------------------------ comandos de calibracion
static const uint8_t CAL_SESGO_GIRO  = 1;   // medir deriva, carro QUIETO
static const uint8_t CAL_CERO_YAW    = 2;   // poner el rumbo actual a cero
static const uint8_t CAL_REDETECTAR  = 3;   // volver a sondear el bus I2C
static const uint8_t CAL_CERO_LINEAS = 4;   // reiniciar contadores de linea

// ---------------------------------------------------------------------------
// CRC-8/ATM (poly 0x07, init 0x00). Sin tabla: 256 bytes de flash no valen la
// pena para 12 bytes de trama, y asi el header no arrastra estado global.
inline uint8_t crc8(const uint8_t *datos, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) {
    c ^= datos[i];
    for (uint8_t k = 0; k < 8; k++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// Escribe la trama completa en 'salida' y devuelve cuantos bytes ocupa
// (0 si el payload no cabe). 'salida' necesita 5 + MAX_PAYLOAD bytes.
inline uint8_t empaquetar(uint8_t tipo, const uint8_t *payload, uint8_t n,
                          uint8_t *salida) {
  if (n > MAX_PAYLOAD) return 0;
  salida[0] = SYNC1;
  salida[1] = SYNC2;
  salida[2] = n;
  salida[3] = tipo;
  if (n) memcpy(salida + 4, payload, n);
  salida[4 + n] = crc8(salida + 2, (uint8_t)(n + 2));
  return (uint8_t)(n + 5);
}

// ===========================================================================
// MANDO — lo unico que la Pi manda para conducir.
//
// vel y dir van en PORCENTAJE CON SIGNO, no en PWM ni en grados. El firmware
// es el unico que conoce los topes mecanicos del servo y el PWM del puente H:
// asi, si cambia el servo o la relacion de la cremallera, se toca UN archivo
// y el codigo de la Pi ni se entera.
// ===========================================================================
struct Mando {
  uint8_t seq;
  uint8_t flags;
  int8_t  vel;    // -100..100  (+ adelante)
  int8_t  dir;    // -100..100  (+ derecha)
  uint8_t vmax;   // techo de PWM 0..255 que impone la Pi
  uint8_t aux;    // bit0 = modo lento (maniobra fina), resto reservado

  bool armado()  const { return (flags & F_ARMADO)  != 0; }
  bool parada()  const { return (flags & F_PARADA)  != 0; }
  bool centrar() const { return (flags & F_CENTRAR) != 0; }
  bool limpiar() const { return (flags & F_LIMPIAR) != 0; }
};

inline bool decodificarMando(const uint8_t *p, uint8_t n, Mando &m) {
  if (n < 6) return false;
  m.seq   = p[0];
  m.flags = p[1];
  m.vel   = (int8_t)p[2];
  m.dir   = (int8_t)p[3];
  m.vmax  = p[4];
  m.aux   = p[5];
  // Recorte defensivo: aunque la Pi mande basura DENTRO de una trama con CRC
  // correcto (un bug de software, no ruido de linea), aqui ya no pasa de 100.
  if (m.vel >  100) m.vel =  100;
  if (m.vel < -100) m.vel = -100;
  if (m.dir >  100) m.dir =  100;
  if (m.dir < -100) m.dir = -100;
  return true;
}

// ------------------------------------------------------------- TELEMETRIA
struct Telemetria {
  uint8_t  seq_eco;          // devuelve el seq del ultimo mando valido
  uint8_t  estado;           // bits E_*
  uint8_t  pwm;              // PWM absoluto que se esta escribiendo
  uint8_t  angulo;           // grados reales del servo
  uint16_t ms_desde_mando;   // cuanto lleva callada la Pi
  uint8_t  tramas_malas;     // CRC fallidos acumulados, saturado en 255
  uint8_t  version;
};

inline uint8_t empaquetarTelemetria(const Telemetria &t, uint8_t *salida) {
  uint8_t p[8];
  p[0] = t.seq_eco;
  p[1] = t.estado;
  p[2] = t.pwm;
  p[3] = t.angulo;
  p[4] = (uint8_t)(t.ms_desde_mando & 0xFF);   // little endian, como Python
  p[5] = (uint8_t)(t.ms_desde_mando >> 8);
  p[6] = t.tramas_malas;
  p[7] = t.version;
  return empaquetar(TIPO_TELE, p, 8, salida);
}

// --------------------------------------------------------------- SENSORES
// Los contadores de linea son ACUMULADOS, no eventos: si se pierde una trama
// la Pi no pierde el cruce, solo lo ve un ciclo mas tarde. Esa decision es la
// que permite contar vueltas sin ACK ni retransmisiones.
struct Sensores {
  uint8_t  estado;           // bits S_* + clase de linea en los 2 bits altos
  int16_t  yaw_d10;          // rumbo en decimas de grado (-1800..1800)
  int16_t  gz_d10;           // velocidad angular en decimas de grado/s
  uint16_t claro;            // canal C del TCS (nivel de luz del piso)
  uint8_t  cruces_naranja;
  uint8_t  cruces_azul;
  uint8_t  botones;          // bits B_*
  uint8_t  pulsaciones;      // pulsaciones completas, contador que envuelve
  uint8_t  version;
  // LOS DOS NUMEROS CON LOS QUE EL CLASIFICADOR DE LINEAS DECIDE. Sin ellos,
  // "el carro no ve las lineas" no se puede diagnosticar desde la Pi: se ve
  // el claro, pero no contra que se esta comparando ni cuanto color hay.
  // Con los dos, la respuesta es una resta: |separacion| por debajo de
  // sep_entrada es que no llega color (altura del sensor, lente sucia), y
  // claro por debajo del suelo de luz es que no hay tapete debajo.
  uint16_t blanco;           // nivel de claro del tapete, aprendido
  int16_t  separacion;       // (r-b)/(r+g+b) en milesimas
};

inline uint8_t empaquetarSensores(const Sensores &s, uint8_t *salida) {
  uint8_t p[16];
  p[0]  = s.estado;
  p[1]  = (uint8_t)(s.yaw_d10 & 0xFF);
  p[2]  = (uint8_t)((s.yaw_d10 >> 8) & 0xFF);
  p[3]  = (uint8_t)(s.gz_d10 & 0xFF);
  p[4]  = (uint8_t)((s.gz_d10 >> 8) & 0xFF);
  p[5]  = (uint8_t)(s.claro & 0xFF);
  p[6]  = (uint8_t)(s.claro >> 8);
  p[7]  = s.cruces_naranja;
  p[8]  = s.cruces_azul;
  p[9]  = s.botones;
  p[10] = s.pulsaciones;
  p[11] = s.version;
  // Los cuatro bytes de diagnostico van AL FINAL a proposito: una Pi con
  // codigo viejo lee los doce primeros y no se entera de que hay mas.
  p[12] = (uint8_t)(s.blanco & 0xFF);
  p[13] = (uint8_t)(s.blanco >> 8);
  p[14] = (uint8_t)(s.separacion & 0xFF);
  p[15] = (uint8_t)((s.separacion >> 8) & 0xFF);
  return empaquetar(TIPO_SENS, p, 16, salida);
}

// ===========================================================================
// LECTOR — maquina de estados que saca tramas de un flujo de bytes.
//
// Se alimenta byte a byte (no bloquea, no reserva memoria) para poder
// llamarla desde la tarea de RX sin importar como lleguen los bytes partidos.
// Si el SYNC se desalinea por ruido vuelve sola: nunca se queda colgada
// esperando un LEN que no existe.
// ===========================================================================
class Lector {
 public:
  void reiniciar() { paso_ = 0; n_ = 0; i_ = 0; }

  // Devuelve true cuando tipo/payload/n quedan cargados con una trama valida.
  // 'malas' cuenta los CRC fallidos, para que la telemetria los delate.
  bool alimentar(uint8_t b, uint8_t &tipo, uint8_t *payload, uint8_t &n,
                 uint32_t &malas) {
    switch (paso_) {
      case 0:
        if (b == SYNC1) paso_ = 1;
        return false;
      case 1:
        // Un A5 repetido no debe perder el sincronismo: sigue esperando 5A.
        paso_ = (b == SYNC2) ? 2 : (b == SYNC1 ? 1 : 0);
        return false;
      case 2:
        if (b > MAX_PAYLOAD) { paso_ = 0; malas++; return false; }
        n_ = b; i_ = 0; paso_ = 3;
        return false;
      case 3:
        tipo_ = b;
        paso_ = (n_ == 0) ? 5 : 4;
        return false;
      case 4:
        buf_[i_++] = b;
        if (i_ >= n_) paso_ = 5;
        return false;
      default: {
        uint8_t cab[2] = {n_, tipo_};
        uint8_t c = crc8(cab, 2);
        // CRC continuado sobre el payload, sin copiar a ningun temporal.
        for (uint8_t k = 0; k < n_; k++) {
          c ^= buf_[k];
          for (uint8_t j = 0; j < 8; j++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
        }
        paso_ = 0;
        if (c != b) { malas++; return false; }
        tipo = tipo_;
        n = n_;
        if (n_) memcpy(payload, buf_, n_);
        return true;
      }
    }
  }

 private:
  uint8_t paso_ = 0, n_ = 0, i_ = 0, tipo_ = 0;
  uint8_t buf_[MAX_PAYLOAD];
};

}  // namespace proto

#endif  // PROTOCOLO_H
