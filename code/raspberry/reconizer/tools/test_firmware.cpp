// ===========================================================================
// test_firmware.cpp — Prueba en el PC la logica del ESP32, sin ESP32.
//
//   g++ -std=c++17 -O2 -I firmware/esp32_carro tools/test_firmware.cpp -o /tmp/tfw
//   /tmp/tfw            -> ejecuta las pruebas
//   /tmp/tfw vectores   -> imprime vectores para cruzarlos con Python
//
// protocolo.h y seguridad.h son C++ puro justamente para esto: lo que decide
// si una pieza se rompe se prueba antes de subirlo a la placa.
// ===========================================================================
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "protocolo.h"
#include "seguridad.h"

static int fallos = 0, oks = 0;
static void check(bool c, const char *nombre, const char *detalle = "") {
  if (c) { oks++; printf("  ok   %s\n", nombre); }
  else   { fallos++; printf("  FALLA %s  %s\n", nombre, detalle); }
}

// ---------------------------------------------------------------------------
static void testCrc() {
  printf("\n[C1] CRC8\n");
  check(proto::crc8((const uint8_t *)"", 0) == 0x00, "crc de vacio = 0x00");
  const uint8_t uno[1] = { 0x00 };
  check(proto::crc8(uno, 1) == 0x00, "crc de {0x00} = 0x00");
  const uint8_t abc[3] = { 'A', 'B', 'C' };
  uint8_t c = proto::crc8(abc, 3);
  printf("       crc8(\"ABC\") = 0x%02X\n", c);
  const uint8_t seq[9] = { '1','2','3','4','5','6','7','8','9' };
  uint8_t chk = proto::crc8(seq, 9);
  // CRC-8/ATM (poly 0x07, init 0) del clasico "123456789" es 0xF4
  check(chk == 0xF4, "crc8(\"123456789\") = 0xF4 (CRC-8/ATM)",
        (std::string("da 0x") + std::to_string((int)chk)).c_str());
}

static void testTrama() {
  printf("\n[C2] Empaquetado y lectura\n");
  uint8_t payload[6] = { 7, proto::F_ARMADO, (uint8_t)(int8_t)-40, 55, 180, 0 };
  uint8_t buf[32];
  uint8_t n = proto::empaquetar(proto::TIPO_MANDO, payload, 6, buf);
  check(n == 11, "una trama de mando ocupa 11 bytes");
  check(buf[0] == 0xA5 && buf[1] == 0x5A, "empieza por A5 5A");
  check(buf[2] == 6 && buf[3] == proto::TIPO_MANDO, "LEN y TIPO en su sitio");

  proto::Lector L;
  bool completa = false;
  for (uint8_t i = 0; i < n; i++) { L.alimentar(buf[i]); completa = L.siguiente(); }
  check(completa, "el ultimo byte completa la trama");
  check(L.tipo() == proto::TIPO_MANDO && L.len() == 6, "tipo y len leidos");

  proto::Mando m{};
  check(proto::decodificarMando(L.payload(), L.len(), m), "decodifica");
  check(m.seq == 7 && m.vel == -40 && m.dir == 55 && m.vmax == 180,
        "los campos sobreviven el viaje");
  check(m.armado() && !m.parada(), "banderas");

  // CRC corrompido
  buf[6] ^= 0xFF;
  proto::Lector L2;
  completa = false;
  for (uint8_t i = 0; i < n; i++) { L2.alimentar(buf[i]); completa |= L2.siguiente(); }
  check(!completa, "una trama con un byte cambiado NO se acepta");
  check(L2.crcMalos == 1, "y se cuenta como CRC malo");
}

static void testResync() {
  printf("\n[C3] Resincronizacion con ruido\n");
  uint8_t payload[6] = { 1, 0, 50, (uint8_t)(int8_t)-50, 200, 0 };
  uint8_t trama[32];
  uint8_t n = proto::empaquetar(proto::TIPO_MANDO, payload, 6, trama);

  std::vector<uint8_t> flujo;
  // basura, media trama cortada, basura, y al final una trama entera
  const uint8_t basura[] = { 0x00, 0xFF, 0xA5, 0xA5, 0x13, 0x5A, 0x7F };
  for (uint8_t b : basura) flujo.push_back(b);
  for (uint8_t i = 0; i < 6; i++) flujo.push_back(trama[i]);   // trama cortada
  flujo.push_back(0xEE);
  for (uint8_t i = 0; i < n; i++) flujo.push_back(trama[i]);

  proto::Lector L;
  int completas = 0;
  for (uint8_t b : flujo) { L.alimentar(b); while (L.siguiente()) completas++; }
  check(completas == 1, "una trama truncada NO se traga la buena que viene detras",
        (std::string("dio ") + std::to_string(completas)).c_str());

  // 200 tramas seguidas sin ruido
  proto::Lector L3;
  int c3 = 0;
  for (int k = 0; k < 200; k++) {
    payload[0] = (uint8_t)k;
    uint8_t t2[32];
    uint8_t n2 = proto::empaquetar(proto::TIPO_MANDO, payload, 6, t2);
    for (uint8_t i = 0; i < n2; i++) { L3.alimentar(t2[i]); while (L3.siguiente()) c3++; }
  }
  check(c3 == 200, "200 tramas seguidas = 200 lecturas");
  check(L3.crcMalos == 0 && L3.descartados == 0, "sin descartes en flujo limpio");

  // Arranque a mitad de trama (la Pi abre el puerto con el ESP32 ya hablando)
  proto::Lector L4;
  int c4 = 0;
  for (uint8_t i = 5; i < n; i++) { L4.alimentar(trama[i]); while (L4.siguiente()) c4++; }
  for (int k = 0; k < 3; k++)
    for (uint8_t i = 0; i < n; i++) { L4.alimentar(trama[i]); while (L4.siguiente()) c4++; }
  check(c4 == 3, "engancha desde la siguiente trama entera al abrir el puerto",
        (std::string("dio ") + std::to_string(c4)).c_str());

  // Un byte de ruido metido entre dos tramas buenas: solo se pierde una
  proto::Lector L5;
  int c5 = 0;
  for (uint8_t i = 0; i < n; i++) { L5.alimentar(trama[i]); while (L5.siguiente()) c5++; }
  L5.alimentar(0xA5);                       // ruido que finge ser un sync
  for (int k = 0; k < 2; k++)
    for (uint8_t i = 0; i < n; i++) { L5.alimentar(trama[i]); while (L5.siguiente()) c5++; }
  check(c5 == 3, "un byte de ruido intercalado no rompe el flujo",
        (std::string("dio ") + std::to_string(c5)).c_str());
}

static void testServoLimites() {
  printf("\n[C4] Limites del servo (la parte que rompe piezas)\n");
  seg::ControlServo s;
  seg::ConfigServo c;
  c.centro = 100; c.izquierda = 65; c.derecha = 135; c.gradosPorSeg = 100000;
  s.configurar(c);

  check(s.anguloDesdePorcentaje(0) == 100, "0% = centro");
  check(s.anguloDesdePorcentaje(100) == 135, "+100% = tope derecho exacto");
  check(s.anguloDesdePorcentaje(-100) == 65, "-100% = tope izquierdo exacto");
  check(s.anguloDesdePorcentaje(50) == 117, "+50% a mitad de camino");

  check(s.anguloDesdePorcentaje(500) == 135, "un 500% se recorta al tope");
  check(s.anguloDesdePorcentaje(-500) == 65, "un -500% se recorta al tope");
  check(s.saturado(), "y marca saturacion para la telemetria");

  // Una configuracion que intenta ampliar el rango fisico
  seg::ConfigServo malo;
  malo.centro = 200; malo.izquierda = 0; malo.derecha = 250;
  s.configurar(malo);
  check(s.config().izquierda >= seg::SERVO_TOPE_MIN, "config no baja del tope min",
        (std::string("izq=") + std::to_string(s.config().izquierda)).c_str());
  check(s.config().derecha <= seg::SERVO_TOPE_MAX, "config no sube del tope max",
        (std::string("der=") + std::to_string(s.config().derecha)).c_str());
  check(s.config().centro >= s.config().izquierda &&
        s.config().centro <= s.config().derecha, "el centro queda entre los topes");

  int peor = 0;
  for (int pct = -1000; pct <= 1000; pct += 7) {
    int a = s.anguloDesdePorcentaje(pct);
    if (a < seg::SERVO_TOPE_MIN || a > seg::SERVO_TOPE_MAX) peor = a;
  }
  check(peor == 0, "ningun porcentaje, ni absurdo, sale del rango fisico");

  // Config con izquierda y derecha intercambiadas
  seg::ConfigServo cruzado;
  cruzado.centro = 100; cruzado.izquierda = 135; cruzado.derecha = 65;
  s.configurar(cruzado);
  check(s.config().izquierda < s.config().derecha, "ordena izquierda/derecha cruzadas");
}

static void testServoVelocidad() {
  printf("\n[C5] Rampa del servo\n");
  seg::ControlServo s;
  seg::ConfigServo c;
  c.centro = 100; c.izquierda = 65; c.derecha = 135; c.gradosPorSeg = 300;
  s.configurar(c);
  s.forzarCentro();

  int a = s.paso(135, 10);        // 300 grados/s * 10 ms = 3 grados
  check(a == 103, "en 10 ms se mueve 3 grados, no 35",
        (std::string("a=") + std::to_string(a)).c_str());

  int pasos = 0;
  while (s.actual() < 135 && pasos < 1000) { s.paso(135, 10); pasos++; }
  check(s.actual() == 135, "acaba llegando al tope");
  check(pasos >= 10 && pasos <= 13, "tarda ~117 ms (35 grados a 300 g/s)",
        (std::string("pasos=") + std::to_string(pasos)).c_str());

  // Latigazo de extremo a extremo pedido en un solo tick
  s.forzarCentro();
  int antes = s.actual();
  int desp = s.paso(65, 10);
  check(antes - desp <= 3, "un salto extremo a extremo se sirve a 3 grados por tick");
}

static void testMotor() {
  printf("\n[C6] Rampa e inversion del motor\n");
  seg::ControlMotor m;
  seg::ConfigMotor c;
  c.rampaPorTick = 10; c.msFrenoAntesDeInvertir = 150;
  m.configurar(c);

  check(m.paso(255, 0) == 10, "arranque suave: 10 cuentas en el primer tick");
  uint32_t t = 0;
  while (m.actual() < 200 && t < 10000) { t += 10; m.paso(200, t); }
  check(m.actual() == 200, "llega al pedido");
  check(t >= 190 && t <= 220, "tarda ~200 ms en llegar a 200",
        (std::string("t=") + std::to_string(t)).c_str());

  // Ahora pedimos reversa a fondo estando a 200 hacia adelante
  bool salto_directo = false;
  int previo = m.actual();
  for (uint32_t k = 0; k < 100; k++) {
    t += 10;
    int v = m.paso(-200, t);
    if ((previo > 0 && v < 0) || (previo < 0 && v > 0)) salto_directo = true;
    previo = v;
  }
  check(!salto_directo, "nunca salta de positivo a negativo sin pasar por cero");
  check(m.actual() < 0, "termina en reversa");

  // La pausa de proteccion existe de verdad
  seg::ControlMotor m2;
  m2.configurar(c);
  uint32_t t2 = 0;
  while (m2.actual() < 100) { t2 += 10; m2.paso(100, t2); }
  uint32_t tCero = 0, tNegativo = 0;
  for (uint32_t k = 0; k < 100; k++) {
    t2 += 10;
    int v = m2.paso(-100, t2);
    if (v == 0 && tCero == 0) tCero = t2;
    if (v < 0) { tNegativo = t2; break; }
  }
  check(tCero != 0 && tNegativo != 0, "pasa por cero y luego invierte");
  check(tNegativo - tCero >= c.msFrenoAntesDeInvertir,
        "se queda quieto los 150 ms de proteccion antes de invertir",
        (std::string("espero ") + std::to_string(tNegativo - tCero) + " ms").c_str());

  // Si ya llevaba mucho rato parado, no hace falta esperar otra vez
  seg::ControlMotor m3;
  m3.configurar(c);
  uint32_t t3 = 0;
  while (m3.actual() < 100) { t3 += 10; m3.paso(100, t3); }
  while (m3.actual() != 0)  { t3 += 10; m3.paso(0, t3); }
  t3 += 5000;                                  // cinco segundos detenido
  check(m3.paso(-100, t3) < 0, "tras estar parado un buen rato invierte al momento");

  // Parada dura
  m2.cortar(t2);
  check(m2.actual() == 0, "cortar() deja el PWM en cero al instante");
  check(m2.paso(100, t2 + 10) == 0,
        "y despues de un corte de emergencia la inversion sigue protegida");
}

static void testPwmDesdePorcentaje() {
  printf("\n[C7] Tope de velocidad (vmax)\n");
  check(seg::pwmDesdePorcentaje(100, 120) == 120, "100% con vmax 120 = 120");
  check(seg::pwmDesdePorcentaje(50, 200) == 100, "50% con vmax 200 = 100");
  check(seg::pwmDesdePorcentaje(-100, 90) == -90, "-100% con vmax 90 = -90");
  check(seg::pwmDesdePorcentaje(1000, 100) == 100, "un 1000% sigue topado en vmax");
  int peor = 0;
  for (int p = -300; p <= 300; p++)
    for (int v = 0; v <= 255; v += 17) {
      int r = seg::pwmDesdePorcentaje(p, (uint8_t)v);
      if (r > v || r < -v) peor = r;
    }
  check(peor == 0, "jamas se supera vmax, con ningun porcentaje");
}

// ---------------------------------------------------------------------------
// Vectores para cruzar con Python
static void imprimirVectores() {
  struct Caso { int seq, flags, vel, dir, vmax; };
  Caso casos[] = {
    {0, 0, 0, 0, 0}, {1, 1, 100, -100, 255}, {200, 3, -100, 100, 128},
    {7, 5, -40, 55, 180}, {255, 15, 1, -1, 1}, {42, 9, -7, 33, 77},
  };
  for (Caso &c : casos) {
    uint8_t p[6] = { (uint8_t)c.seq, (uint8_t)c.flags, (uint8_t)(int8_t)c.vel,
                     (uint8_t)(int8_t)c.dir, (uint8_t)c.vmax, 0 };
    uint8_t buf[32];
    uint8_t n = proto::empaquetar(proto::TIPO_MANDO, p, 6, buf);
    printf("MANDO %d %d %d %d %d ", c.seq, c.flags, c.vel, c.dir, c.vmax);
    for (uint8_t i = 0; i < n; i++) printf("%02X", buf[i]);
    printf("\n");
  }
  proto::Telemetria t;
  int tv[][7] = { {0,0,0,0,0,0,0}, {9,7,200,135,1234,3,2}, {255,31,255,50,65535,255,2} };
  for (auto &v : tv) {
    t.seq_eco = v[0]; t.estado = v[1]; t.pwm = v[2]; t.angulo = v[3];
    t.ms_desde_mando = (uint16_t)v[4]; t.tramas_malas = v[5]; t.version = v[6];
    uint8_t buf[32];
    uint8_t n = proto::empaquetarTelemetria(t, buf);
    printf("TELE %d %d %d %d %d %d %d ", v[0],v[1],v[2],v[3],v[4],v[5],v[6]);
    for (uint8_t i = 0; i < n; i++) printf("%02X", buf[i]);
    printf("\n");
  }
  // Vectores del servo: porcentaje -> grados con la config real del carro
  seg::ControlServo s;
  seg::ConfigServo c; c.centro = 100; c.izquierda = 65; c.derecha = 135;
  s.configurar(c);
  printf("SERVO");
  for (int pct = -120; pct <= 120; pct += 10) printf(" %d", s.anguloDesdePorcentaje(pct));
  printf("\n");
}

int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "vectores") == 0) { imprimirVectores(); return 0; }
  printf("Pruebas de la logica del firmware (host)\n");
  testCrc();
  testTrama();
  testResync();
  testServoLimites();
  testServoVelocidad();
  testMotor();
  testPwmDesdePorcentaje();
  printf("\n%d pruebas ok, %d fallos\n", oks, fallos);
  return fallos ? 1 : 0;
}
