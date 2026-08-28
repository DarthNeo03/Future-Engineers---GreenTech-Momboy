// ===========================================================================
// protocolo.h — Trama binaria ESP32 <-> Raspberry Pi.
//
// GEMELO EXACTO de src/protocolo.py. Si tocas uno, toca el otro:
// tools/selftest_robot.py cruza vectores generados aqui con los de Python y
// falla si se separan.
//
//   A5 5A | LEN | TIPO | payload (LEN bytes) | CRC8
//
// C++ puro (solo stdint/string.h) a proposito: asi se compila tambien con g++
// en el PC y se puede probar sin subir nada al ESP32.
// ===========================================================================
#ifndef PROTOCOLO_H
#define PROTOCOLO_H

#include <stdint.h>
#include <string.h>

namespace proto {

static const uint8_t SYNC1 = 0xA5;
static const uint8_t SYNC2 = 0x5A;
static const uint8_t MAX_PAYLOAD = 16;
static const uint8_t VERSION_PROTOCOLO = 1;

// Tipos de trama
static const uint8_t TIPO_MANDO  = 0x01;   // Pi -> ESP32, 6 bytes
static const uint8_t TIPO_PING   = 0x02;   // Pi -> ESP32, 1 byte
static const uint8_t TIPO_CONFIG = 0x03;   // Pi -> ESP32, 6 bytes
static const uint8_t TIPO_TELE   = 0x81;   // ESP32 -> Pi, 8 bytes
static const uint8_t TIPO_LOG    = 0x82;   // ESP32 -> Pi, texto
static const uint8_t TIPO_PONG   = 0x83;   // ESP32 -> Pi, 1 byte
static const uint8_t TIPO_IMU    = 0x84;   // ESP32 -> Pi, 6 bytes (rumbo)
static const uint8_t TIPO_COLOR  = 0x85;   // ESP32 -> Pi, 6 bytes (evento linea)
static const uint8_t TIPO_SENSORES = 0x86; // ESP32 -> Pi, 4 bytes (que hay)

// Ordenes hacia los sensores, en el byte 'aux' del mando
static const uint8_t AUX_CERO_YAW    = 0x01;
static const uint8_t AUX_CALIB_IMU   = 0x02;
static const uint8_t AUX_CALIB_COLOR = 0x04;

// Lineas del suelo
static const uint8_t LINEA_NINGUNA = 0;
static const uint8_t LINEA_NARANJA = 1;
static const uint8_t LINEA_AZUL    = 2;

// Bits de 'presentes'
static const uint8_t S_MPU = 0x01;
static const uint8_t S_TCS = 0x02;

// Banderas del mando
static const uint8_t F_ARMADO  = 0x01;
static const uint8_t F_PARADA  = 0x02;
static const uint8_t F_CENTRAR = 0x04;
static const uint8_t F_LIMPIAR = 0x08;

// Bits de estado de la telemetria
static const uint8_t E_ARMADO        = 0x01;
static const uint8_t E_MOTOR         = 0x02;
static const uint8_t E_FAILSAFE      = 0x04;
static const uint8_t E_SERVO_TOPE    = 0x08;
static const uint8_t E_INV_BLOQUEADA = 0x10;

// --------------------------------------------------------------------------
// CRC-8/ATM (poly 0x07, init 0x00). Sin tabla: 256 bytes de flash no valen la
// pena para 11 bytes por trama, y asi el header no lleva estado global.
inline uint8_t crc8(const uint8_t *datos, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) {
    c ^= datos[i];
    for (uint8_t k = 0; k < 8; k++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// Escribe la trama completa en 'salida'. Devuelve cuantos bytes ocupa
// (0 si el payload no cabe). 'salida' debe tener al menos 5+MAX_PAYLOAD bytes.
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

// --------------------------------------------------------------------------
// Mando ya decodificado. vel y dir van en % con signo: el firmware es el unico
// que conoce los grados del servo y el PWM del puente H.
struct Mando {
  uint8_t seq;
  uint8_t flags;
  int8_t  vel;        // -100..100
  int8_t  dir;        // -100..100
  uint8_t vmax;       // 0..255
  uint8_t aux;

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
  // Recorte defensivo: aunque la Pi mande basura dentro de una trama con CRC
  // correcto (bug de software, no ruido), aqui ya no puede pasar de +-100.
  if (m.vel >  100) m.vel =  100;
  if (m.vel < -100) m.vel = -100;
  if (m.dir >  100) m.dir =  100;
  if (m.dir < -100) m.dir = -100;
  return true;
}

struct Telemetria {
  uint8_t  seq_eco;
  uint8_t  estado;
  uint8_t  pwm;
  uint8_t  angulo;
  uint16_t ms_desde_mando;
  uint8_t  tramas_malas;
  uint8_t  version;
};

// --------------------------------------------------------------------------
// Rumbo ya integrado por el ESP32. Se manda masticado, no crudo: el MPU escupe
// cientos de muestras por segundo y mandarlas todas saturaria el serial para
// nada. Aqui se integra a 200 Hz y se publican 6 bytes cuando toca.
inline uint8_t empaquetarIMU(float yaw, float giroZ, bool calibrado,
                             uint8_t temp, uint8_t *salida) {
  int32_t y = (int32_t)(yaw * 10.0f);
  if (y >  1800) y =  1800;
  if (y < -1800) y = -1800;
  int32_t g = (int32_t)(giroZ * 10.0f);
  if (g >  32000) g =  32000;
  if (g < -32000) g = -32000;
  uint8_t p[6];
  p[0] = (uint8_t)(y & 0xFF);   p[1] = (uint8_t)((y >> 8) & 0xFF);
  p[2] = (uint8_t)(g & 0xFF);   p[3] = (uint8_t)((g >> 8) & 0xFF);
  p[4] = calibrado ? 1 : 0;     p[5] = temp;
  return empaquetar(TIPO_IMU, p, 6, salida);
}

// Evento de linea. SOLO se manda cuando cambia: cruzar una linea son dos
// tramas de 11 bytes por esquina, no un chorro continuo.
inline uint8_t empaquetarColor(uint8_t linea, uint8_t r, uint8_t g, uint8_t b,
                               uint8_t luz, uint8_t *salida) {
  uint8_t p[6] = { linea, r, g, b, luz, 0 };
  return empaquetar(TIPO_COLOR, p, 6, salida);
}

inline uint8_t empaquetarSensores(uint8_t presentes, uint8_t hzImu,
                                  uint8_t hzColor, uint8_t *salida) {
  uint8_t p[4] = { presentes, hzImu, hzColor, 0 };
  return empaquetar(TIPO_SENSORES, p, 4, salida);
}

inline uint8_t empaquetarTelemetria(const Telemetria &t, uint8_t *salida) {
  uint8_t p[8];
  p[0] = t.seq_eco;
  p[1] = t.estado;
  p[2] = t.pwm;
  p[3] = t.angulo;
  p[4] = (uint8_t)(t.ms_desde_mando & 0xFF);        // little endian
  p[5] = (uint8_t)(t.ms_desde_mando >> 8);
  p[6] = t.tramas_malas;
  p[7] = t.version;
  return empaquetar(TIPO_TELE, p, 8, salida);
}

// --------------------------------------------------------------------------
// Lector con reintento hacia atras.
//
// Una maquina de estados "de una pasada" tiene un agujero: si llega una trama
// truncada, se come los bytes de la SIGUIENTE trama creyendo que son su
// payload, y pierde las dos. Aqui los bytes se acumulan en un buffer y se
// reescanea: si el CRC falla, se avanza UN byte y se vuelve a buscar el sync,
// asi que una trama buena escondida detras de basura se recupera igual.
//
// Uso:
//     while (serial.available()) {
//       lector.alimentar(serial.read());
//       while (lector.siguiente()) { ...usar tipo()/payload()... }
//     }
//
// Sin memoria dinamica: el buffer es fijo de 64 bytes (3 tramas).
class Lector {
 public:
  static const uint8_t MAX_TRAMA = 5 + MAX_PAYLOAD;   // 21 bytes
  static const uint8_t CAP = 64;

  Lector() { reiniciar(); framesOk = 0; crcMalos = 0; descartados = 0; }

  void reiniciar() { n_ = 0; len_ = 0; tipo_ = 0; }

  void alimentar(uint8_t b) {
    if (n_ >= CAP) {          // no deberia pasar si se drena con siguiente()
      descartar(1);
      descartados++;
    }
    buf_[n_++] = b;
  }

  // Saca la siguiente trama valida del buffer. Llamar en bucle.
  bool siguiente() {
    for (;;) {
      // tirar lo que no puede ser el arranque de una trama
      while (n_ >= 1 && buf_[0] != SYNC1) { descartar(1); descartados++; }
      if (n_ >= 2 && buf_[1] != SYNC2) {
        descartar(1); descartados++;
        continue;
      }
      if (n_ < 5) return false;                  // falta cabecera completa

      const uint8_t len = buf_[2];
      if (len > MAX_PAYLOAD) { descartar(1); descartados++; continue; }

      const uint8_t total = (uint8_t)(len + 5);
      if (n_ < total) return false;              // falta cuerpo, esperar bytes

      if (crc8(buf_ + 2, (uint8_t)(len + 2)) == buf_[total - 1]) {
        tipo_ = buf_[3];
        len_ = len;
        for (uint8_t i = 0; i < len; i++) payload_[i] = buf_[4 + i];
        descartar(total);
        framesOk++;
        return true;
      }
      crcMalos++;
      descartar(1);            // un byte y a reescanear: la trama buena puede
                               // estar justo detras
    }
  }

  uint8_t tipo() const { return tipo_; }
  uint8_t len()  const { return len_; }
  const uint8_t *payload() const { return payload_; }
  uint8_t pendientes() const { return n_; }

  uint32_t framesOk, crcMalos, descartados;

 private:
  void descartar(uint8_t k) {
    if (k >= n_) { n_ = 0; return; }
    memmove(buf_, buf_ + k, (size_t)(n_ - k));
    n_ = (uint8_t)(n_ - k);
  }

  uint8_t buf_[CAP];
  uint8_t payload_[MAX_PAYLOAD];
  uint8_t n_, len_, tipo_;
};

}  // namespace proto

#endif  // PROTOCOLO_H
