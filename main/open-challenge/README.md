# Piloto WRO 2026 — OPEN CHALLENGE

Tres vueltas a la pista vacia. Este programa NO mira pilares rojos ni verdes:
el reto de obstaculos vive aparte, en `main/obstacle-challenge/`, y empezo
siendo una copia de este.

Es la version que pasa el Open Challenge, dejada a proposito pequeña. Lo que se
le quito respecto del programa combinado: el esquive, la estrategia de seguir
pared, el metodo viejo de deteccion de muro por pixel negro y la camara
trasera. Lo que se le añadio: que el conteo de esquinas funcione de verdad.

```
Raspberry Pi 5 (vision + decisiones)  <-USB->  ESP32 (motor, servo, MPU6050, TCS34725)
```

El firmware del ESP32 vive en `main/firmware/esp32_carro/` y es COMPARTIDO con
el programa de obstaculos: el mismo binario sirve para los dos retos.

---

## Puesta en marcha

**Raspberry Pi 5** (o PC Windows para probar sin carro):

```bash
python3 -m venv .venv && source .venv/bin/activate    # en Windows: .venv\Scripts\activate
pip install -r requirements.txt
python tools/selftest.py            # 58 pruebas, sin hardware
python main.py                      # camara + ESP32 + web
python main.py --simulado           # sin ESP32 (pruebas en el PC)
python main.py --imagen foto.jpg    # sin camara, sobre una foto
python main.py --vmax 90            # tope de PWM solo para esta prueba
```

**ESP32**: abrir `../firmware/esp32_carro/esp32_carro.ino` en el IDE de Arduino
(los 6 archivos en la misma carpeta) y subir. Sin librerias externas. Al subir
firmware nuevo, subirlo ANTES de correr `main.py`.

Web de depuracion: **http://carrito.local:8080/** (o `http://<ip>:8080/`).
El carro **arranca desarmado**. En competencia se arranca con el PULSADOR del
GPIO 13 (una pulsacion arma y arranca la ronda, otra desarma y para); desde la
web, el boton ARMAR hace lo mismo y hay pulsaciones virtuales para ensayar la
secuencia sin cablear nada.

---

## Que hace en el Open Challenge (reglas 2026)

1. **Sentido de la ronda**: en 2026 las 3 vueltas van TODAS en la misma
   direccion (ya no hay media vuelta) y esta prohibido meterle datos al robot
   antes del start, asi que el carro lo deduce solo:
   - por la **primera linea** de esquina que cruza (TCS bajo el carro o
     camara): naranja primero = horario, azul primero = antihorario
     (`lineas.naranja_es_horario` por si un tapete viniera al reves);
   - por la camara ANTES de cruzar (ve cual linea esta mas cerca);
   - de respaldo, por el lado del primer giro.
   Tambien se puede forzar desde la web para pruebas.
2. **Mantenerse en el carril**: centrado (compara el espacio libre a izquierda
   y derecha, en mm reales). El giroscopio corrige el rumbo en recta y clava
   los giros de 90.
3. **Esquinas**: PRE_GIRO (frena + deja pasar las ruedas traseras + **giro
   abierto** tipo camion si hay sitio) -> GIRO de 90 por giroscopio -> RECTO.
   Disparadores: pasillo cerrandose, muro interno que desaparece, o linea de
   esquina cruzada.
4. **Conteo**: cada esquina tiene un par de lineas naranja+azul y el par cuenta
   UNA esquina. Una esquina se ABRE con su primera linea y la del otro color la
   CIERRA durante `lineas.cierre_max_ms`, tarde o temprano: nunca abre otra.
   Eso es lo que impide que una azul que llega tarde -y llega tarde a menudo,
   porque es mas oscura y el sensor se la salta- sume una esquina de mentira y
   el carro se crea una vuelta por delante. La MISMA linea repetida es rebote y
   se ignora durante el refractario. 4 esquinas = vuelta.
5. **Parada final**: tras la esquina 12 avanza `carrera.parada_ms` para meter
   el carro ENTERO en la seccion de meta y se detiene solo (bono del
   reglamento). Tope de 3 minutos.

**Aqui no hay pilares.** Ese reto es `main/obstacle-challenge/`.

---

## El arreglo de las paredes con brillo (lo importante)

El programa viejo buscaba "el pixel negro mas bajo" por columna. Cuando la
pared brillaba dejaba de ser negra: la linea de contacto saltaba a las sillas
del fondo y el carro creia que tenia via libre (capturas 514/515).

El metodo nuevo (`muro.metodo = piso`) usa dos ideas geometricas:

1. **Horizonte**: con la camara a 125 mm y 7.5 grados, TODO lo que es pista
   queda por debajo de una fila fija de la imagen. Sillas, mesas y publico se
   recortan por geometria antes de mirar un solo color.
2. **Primera transicion piso -> no-piso, subiendo desde abajo**: solo se
   supone que el PISO parece piso (blanco o linea naranja/azul). La pared
   puede brillar o ser gris: mientras no parezca piso blanco, se encuentra su
   base igual. Y como se toma la PRIMERA transicion, nada del fondo puede
   adelantarse.

Encima se ajustan **rectas** al contorno (en mm reales) que se fusionan a
traves de los huecos por brillo, y sus intersecciones dan las **esquinas**
("saliente" = canto del muro interno, "rincon" = esquina del externo).

El metodo viejo ya no esta: se quito al separar los programas. `tools/selftest.py`
conserva el caso sintetico de la pared con brillo, que es donde fallaba.

### Cuanto piso se ve por cada mitad

Ademas de la distancia al muro, el perfil publica `piso_izq` y `piso_der`: la
fraccion de pixeles de piso de cada mitad de la imagen. Parece redundante con
`izq`/`der` y no lo es. Aquellas miden A QUE DISTANCIA esta el muro, y metido en
un rincon el muro esta igual de cerca por los dos lados: empatan y no dicen
nada. Contar pixeles blancos si responde a la pregunta util, que es por donde
queda sitio para salir, y por eso es lo que decide el lado del escape.

Se mide sobre TODA la franja util, sin recortar por el horizonte. El recorte del
horizonte solo se aplica a la BUSQUEDA DEL MURO, donde hace falta por
geometria.

---

## Distancias reales (calibracion geometrica)

Todo el perfil trabaja en **milimetros sobre el suelo**, no en pixeles:

- Medir y poner `geometria.alto_cam_mm` (125) e `inclinacion_deg` (7.5).
- Pestaña **Calibrar**: poner un objeto a distancia conocida del morro,
  escribir la distancia, pulsar el boton y tocar en el video donde el objeto
  toca el piso -> se resuelve la focal `fy`. Lo mismo de lado para `fx`.
- `ancho_carro_mm` + `margen_ruedas_mm` dibujan el **corredor** por donde van
  a pasar las ruedas (la camara no las ve); el "pasillo" es lo que de verdad
  hay dentro de ese corredor, en mm.

Gracias a eso los umbrales (`girar_bajo_mm`, `parar_bajo_mm`...) son
distancias fisicas y valen igual en pista ancha (1000 mm) o angosta (600 mm).

---

## La web (todo en caliente, nada requiere reiniciar)

- **Carrera**: video anotado, ARMAR/PARAR, modo, sentido, conteo, ajustes
  rapidos y telemetria.
- **Manual**: joystick tactil (y flechas/WASD). Hombre muerto: si el joystick
  deja de refrescar 400 ms, el carro se para. Para rescates tras choque.
- **Colores**: igual que el calibrador viejo pero en el movil: clic sobre el
  objeto (con acumular para cara iluminada+sombra), sliders HSV y filtros,
  vista de mascara y de piso, 20 perfiles rotativos.
- **Ajustes**: TODOS los parametros, generados del esquema con su descripcion
  (añadir un parametro en `src/params.py` lo hace aparecer solo). 20 perfiles
  rotativos ("casa", "pabellon"...).
- **Calibrar**: focales por clic, giroscopio (calibrar con el carro QUIETO),
  y TCS: poner el sensor sobre blanco/naranja/azul y pulsar el boton; los
  umbrales se calculan y viajan al ESP32.
- **Sistema**: enlace, contadores de tramas, log, reintento de sensores I2C.

## Parametros que se tocan en pista

| Parametro | Que hace |
|---|---|
| `limites.vmax` | Tope duro de PWM. El freno de mano de todas las pruebas. |
| `navegacion.kp` / `kd` | PD del centrado: kp si corrige lento, kd si oscila. |
| `navegacion.girar_bajo_mm` | Pasillo con el que asume esquina y gira. |
| `navegacion.ttc_min_s` | Freno por tiempo-hasta-el-muro (anti-inercia). El que mas se toca. |
| `navegacion.retardo_giro_ms` | Espera del pre-giro (ruedas traseras pasan el canto interno). |
| `navegacion.apertura_pct` | Cuanto se abre (contra-direccion) antes de cortar la esquina. |
| `carrera.parada_ms` | Cuanto avanza tras la ultima esquina antes de pararse en meta. |
| `muro.k_transicion` | Filas no-piso seguidas para creer el muro (sube si hay muros fantasma). |
| `lineas.cierre_max_ms` | Cuanto se espera la segunda linea del par. Solo bajalo si dos esquinas de verdad quedan muy seguidas. |
| `tcs.naranja_dif_min` / `azul_dif_min` | Los discriminadores del sensor de piso. Son cocientes, asi que no cambian con la luz ni con el tiempo de integracion: son los que hay que tocar. |

## Estructura

```
open-challenge/
├── main.py                 arranque; --simulado, --imagen, --vmax, --puerto
├── piloto.sh               demonio de competencia (sin web)
├── config/                 params.json y colors.json (se crean solos; 20 perfiles c/u)
├── src/
│   ├── geometria.py        pixeles <-> mm sobre el suelo; horizonte; corredor
│   ├── muro.py             perfil de contacto robusto + rectas + esquinas
│   ├── vision.py           mascaras HSV y deteccion de objetos (del reconizer)
│   ├── color_config.py     perfiles de color (mismo formato que el reconizer)
│   ├── params.py           esquema autodocumentado de parametros + perfiles
│   ├── lineas.py           sentido / esquinas / vueltas (TCS + camara + giros)
│   ├── navegacion.py       RECTO / PRE_GIRO / GIRO / ESCAPE
│   ├── carrera.py          director de la ronda (3 vueltas y parada en meta)
│   ├── botones.py          el pulsador de competencia (cuelga del ESP32)
│   ├── protocolo.py        trama binaria v3 (gemela de protocolo.h)
│   ├── enlace.py           hilo serie; sensores del ESP32 -> eventos
│   ├── robot.py            el nucleo que une todo
│   ├── dibujo.py           overlay del video
│   ├── servidor.py         http.server + MJPEG
│   └── web/index.html      la interfaz
└── tools/selftest.py       58 pruebas sin hardware
```

## Notas practicas (heredadas a golpes)

- **Congela exposicion y balance de blancos antes de calibrar colores**
  (`camara.exposicion` / `balance_blancos`; -1 = automatico).
- El giroscopio se calibra solo al detectarse (con el carro quieto en la
  preparacion) y hay boton para repetirlo. Sin calibrar deriva 1-3 grados/s.
- Los cruces de linea del TCS viajan como CONTADORES: perder tramas no pierde
  cruces.
- Regla 9.9 del reglamento: en competencia NO se puede calibrar despues de la
  revision tecnica. Calibrar colores/TCS ANTES de entregar el carro.
- En el Open Challenge esta prohibido tocar el muro perimetral exterior:
  `parar_bajo_mm` y el escape existen para eso; mejor conservador.
- **El TCS integra 12 ms** (`tcs.atime` 251) y el ESP32 lo sondea a 200 Hz, no
  una vez por periodo de integracion: sondear al mismo ritmo al que el chip
  integra hace que los dos relojes deriven y se pierda un ciclo de cada dos, y
  entonces 24 ms de integracion daban hasta 48 ms reales entre muestras. Con
  eso una linea de 2 cm cabia entera en el hueco sin dejar ni una lectura. Al
  cambiar `atime` hay que volver a muestrear BLANCO en la pestaña Calibrar: las
  cuentas absolutas se parten por dos (`tcs.c_min`), aunque los ratios y sus
  diferencias -que son los que deciden el color- no cambian.
- **El TCS no veia la linea AZUL** y costo caro entenderlo. Sobre azul el sensor
  da C=678 R=186 B=284, o sea ratios r=70 y b=107 contra ~85 del blanco: 22
  puntos de margen en su propio canal, que con un umbral absoluto de 110 no
  bastaban. En la DIFERENCIA b-r saca 37 contra ~0. Ademas el perfil guardado
  tenia `c_min` sacado del 25 % de un blanco brillante, y como la linea azul
  absorbe luz, se descartaba antes de mirar siquiera su color.
