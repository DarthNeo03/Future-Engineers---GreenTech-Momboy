# Carro WRO 2026 — Futuros Ingenieros

Raspberry Pi 5 (visión y decisiones) → ESP32 (hardware) → puente H → motor.
Mismo código en Windows 11 y en Raspbian, Python 3.11.

```
reconizer/
├── main.py                     ← el programa principal
├── config/
│   ├── colors.json             ← las últimas 5 calibraciones de color
│   └── robot.json              ← puerto, límites, ganancias, red
├── src/
│   ├── color_config.py   perfiles de color (guardado atómico)
│   ├── camera.py         cámara USB igual en los dos sistemas
│   ├── vision.py         máscaras HSV y detección de objetos
│   ├── protocolo.py      trama binaria hacia el ESP32
│   ├── enlace.py         hilo serie con autodetección de puerto
│   ├── imu.py            MPU6050 opcional por I2C
│   ├── sensores.py       rumbo y color: del ESP32, de la Pi, o de ninguno
│   ├── navegacion.py     perfil del muro, 3 estrategias, esquinas y escape
│   ├── vueltas.py        contador de vueltas fusionando 3 fuentes
│   ├── robot.py          el núcleo que lo une todo
│   ├── robot_config.py   robot.json
│   └── servidor.py       carrito.local: vídeo + control desde el móvil
├── tools/
│   ├── calibrador.py     interfaz de calibración HSV
│   ├── panel.py          panel de pruebas de escritorio
│   ├── selftest.py       51 pruebas de visión, sin cámara
│   ├── selftest_robot.py 189 pruebas del sistema, sin carro
│   ├── test_firmware.cpp 59 pruebas de la lógica del ESP32, sin ESP32
│   └── carrito_wifi.sh   AP, mDNS, UART, I2C y servicio de arranque
├── docs/
│   └── metodos_navegacion.html   análisis de los 5 métodos
└── firmware/esp32_carro/
    ├── esp32_carro.ino   firmware (sin WiFi, con sensores)
    ├── protocolo.h       gemelo en C++ de protocolo.py
    ├── seguridad.h       límites del servo y del motor
    ├── lineas.h          clasificador naranja/azul del TCS34725
    └── sensores_i2c.h    MPU6050 y TCS34725 autodetectados
```

---

## Puesta en marcha

**Raspberry Pi 5**

```bash
sudo bash tools/carrito_wifi.sh instalar   # avahi, UART, I2C, hostname carrito
sudo reboot
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 tools/selftest.py && python3 tools/selftest_robot.py
sudo bash tools/carrito_wifi.sh ap         # crea la red Carrito-WRO
python3 main.py --sin-panel
```

**Windows 11** (desarrollo, sin carro)

```bat
.venv\Scripts\activate
pip install -r requirements.txt
python tools\selftest.py
python tools\selftest_robot.py
python main.py --simulado --imagen capturas\pista.png
```

**ESP32**: abre `firmware/esp32_carro/esp32_carro.ino` en el IDE de Arduino
(los cinco archivos tienen que estar en la misma carpeta) y súbelo. Ya no hace
falta ninguna librería externa: fuera WiFiManager, WebSockets y ESPmDNS.

**Cableado del serie** (cruzado, ambos a 3,3 V, sin divisor):

| Raspberry Pi 5 | ESP32 |
|---|---|
| GPIO14 / TXD (pin 8) | GPIO16 (RX2) |
| GPIO15 / RXD (pin 10) | GPIO17 (TX2) |
| GND (pin 6) | GND |

También funciona por USB sin tocar nada: el firmware escucha las dos bocas y
contesta por la que recibió la última trama válida.

---

## Cómo se usa

Enciende, entra a **http://carrito.local:8080/** desde el móvil (o
`http://192.168.50.1:8080/` si el mDNS no resuelve) y verás lo que ve el carro
con el muro detectado, el perfil de distancia, los huecos por los que cabe, por
dónde pasan las ruedas y la decisión que está tomando. Desde ahí armas, paras,
cambias la velocidad máxima, la mezcla de navegaciones, el número de vueltas y
las ganancias, en caliente.

El panel de escritorio (`main.py` lo abre solo si hay pantalla) muestra lo mismo
más los parámetros finos que en el móvil estorban.

**El carro arranca siempre DESARMADO.** Hay que pulsar ARMAR para que el motor
pueda moverse. `Escape` en el panel y el botón rojo en la web son parada de
emergencia.

Para limitar la velocidad de esta prueba: el slider *vmax* (0-255) es el tope de
PWM que el ESP32 no supera nunca, y *crucero* / *en giro* son porcentajes de ese
tope. `python3 main.py --vmax 90` lo fija desde la línea de órdenes.

---

## Cómo no chocar con los muros

Todo sale del mismo sitio: el **perfil de contacto muro–piso**, columna por
columna. Para cada columna se busca el píxel negro más bajo, que es donde el
muro toca el suelo: cuanto más abajo, más cerca. Sale un perfil de distancia de
ancho completo, como un LIDAR pobre. Encima de eso van tres estrategias que se
pueden **mezclar con pesos** desde la web o el panel:

| Estrategia | Qué hace | Cuándo brilla |
|---|---|---|
| **Centrado** | Compara el espacio libre de la banda izquierda y la derecha y gira hacia la despejada con un PD | Es la más tolerante a una calibración imperfecta. Por defecto |
| **Seguir pared externa** | Mantiene la pared exterior a una distancia fija con un PD | Trayectorias limpias y repetibles. En `auto` deduce sola cuál es la externa |
| **Hueco pasable** | Busca los tramos por los que el carro **cabe de verdad**, contando el ancho de las ruedas a la distancia del obstáculo | Reto de obstáculos. Es lo que propusiste tú |

Los pesos se suman: `centrado 1.0 + hueco 0.5` es una combinación válida y se
ve el aporte de cada una en la telemetría.

### Lo que se arregló de la primera prueba

**La inercia.** Antes solo se miraba la distancia: cuando el umbral saltaba, el
carro ya llevaba la velocidad encima. Ahora hay dos capas:

- **Envolvente de velocidad** por distancia, como antes.
- **Tiempo hasta el muro**: se mide la *velocidad de cierre* del pasillo (cuánto
  se reduce por segundo) y se calcula cuántos segundos faltan para llegar. Si
  bajan de `ttc_min` (1.1 s por defecto) se frena **aunque la distancia todavía
  parezca aceptable**. Es lo que hace que llegue a la esquina ya frenado.
- **Estado `pre_giro`**: al detectar la esquina no se gira de golpe. Primero se
  va recto y frenando durante `retardo_giro_ms`, y solo entonces se gira. Ese
  retardo es además lo que deja pasar las ruedas traseras por la esquina interna.

**El escape.** Retroceder a ciegas era el problema: en una esquina hay otro muro
detrás que la cámara no ve, y el carro se quedaba encajado empujando. Ahora:

1. **Primero, giro hacia adelante** alejándose del muro externo (si se conoce el
   sentido de la vuelta, hacia el lado del muro interno). Casi siempre resuelve.
2. Si en `escape_evaluar_ms` (700 ms) el espacio **no mejora**, prueba marcha atrás.
3. Si atrás tampoco mejora — el caso de la esquina de atrás — vuelve a adelante,
   y a la tercera cambia de lado. Cuenta los cambios y avisa de `ATASCADO`.

**El giro tardío del seguimiento de pared.** No era cuestión de ganancias: en la
esquina no hay pared que seguir. La solución es el disparador nuevo.

### La esquina del muro interno

El muro externo siempre se ve; el interno **se acaba** en cada esquina. Ese final
es un **escalón brusco en el perfil**: delante hay muro cerca, y de golpe la
columna siguiente ve piso o el muro de enfrente, mucho más lejos.

```
libre  ────────────╮
                   │  ← escalón: aquí se acabó el muro interno
     ──────────────╯
        columnas de la imagen  →
```

Ese escalón llega **bastante antes** de que el muro de enfrente esté encima, que
es justo el aviso que faltaba. Cuando salta, el carro entra en `pre_giro` y luego
gira con `dir_giro_abierto` (65 % por defecto, no a tope) para que las ruedas
traseras no barran la esquina.

### El sentido de la vuelta

Yendo en sentido horario el centro de la pista queda a la derecha, así que el
**muro interno está a la derecha** y los giros son a la derecha. El programa lo
deduce por votos de tres fuentes:

- el signo de los giros que ya ha hecho,
- el lado donde aparece el escalón del muro interno,
- el orden de las líneas del suelo (naranja y luego azul = horario, configurable).

Con el sentido conocido, `lado_pared: auto` sigue sola la pared externa y el
escape sabe hacia dónde salir. Tras la media vuelta todo se invierte solo.

### El ancho de carril se calibra solo

En competencia abierta el carril cambia de una pista a otra, así que los
umbrales fijos no valen. En recta, con poca dirección y el frente despejado, la
**suma del espacio libre de las dos bandas laterales es prácticamente constante**
y no depende de por dónde vayas dentro del carril: esa suma *es* el ancho del
carril en las unidades del perfil.

Con ella se derivan `parar_bajo`, `girar_bajo` y `frenar_bajo`, acotados a
valores sensatos. En la web aparece `(auto)` al lado del umbral cuando está
mandando la medida. Se puede desactivar con `autocalibrar_carril`.

### Parámetros que vas a tocar en la pista

| Parámetro | Qué hace |
|---|---|
| `ttc_min` | Segundos hasta el muro por debajo de los cuales frena. **Súbelo si sigue llegando rápido a las esquinas** |
| `interno_libre` | Cuánto tiene que despejarse la banda interna para dar la esquina por buena |
| `retardo_giro_ms` | Espera entre detectar la esquina y girar. Súbelo si las ruedas traseras rozan |
| `dir_giro_abierto` | Cuánto vuelca la dirección en la esquina. Bájalo para un giro más abierto |
| `margen_hueco` | Cuánto más ancho que el carro se exige un hueco (1.15 = 15 % de margen) |
| `y_horizonte` | Fila del horizonte. Escala la perspectiva del ancho del carro. **Reajústalo si mueves la cámara** |
| `kp` / `kd` | PD del centrado. Sube `kp` si corrige lento, sube `kd` si oscila |
| `px_min_columna` | Píxeles negros mínimos en una columna para creerse que hay muro |

## Vueltas, y media vuelta

En la pista hay una línea naranja y una azul en cada esquina. El contador fusiona
**tres fuentes**, porque una sola falla:

1. **La cámara** ve las líneas del suelo (colores ya calibrados). Falla si la luz
   cambia o la línea queda fuera del recorte de abajo.
2. **El TCS34725** las ve por contacto. Falla si el carro pasa por el borde.
3. **Los giros de 90°** que cuenta la navegación. No falla casi nunca, pero no
   distingue una esquina de un esquive brusco.

Una esquina se da por buena cuando se completa un **par** de líneas (naranja +
azul en cualquier orden dentro de una ventana) o cuando termina un giro. Ver la
misma esquina por los tres caminos suma **una**. Cuatro esquinas = una vuelta.

Al llegar al objetivo (3 por defecto, ajustable en la web y el panel) pide la
media vuelta, y al terminarla cuenta otras tantas en sentido contrario. Hay dos
maniobras, elegibles en caliente:

- **`recta_3t`** — tres tiempos en la recta: adelante girando, atrás girando al
  revés, adelante otra vez, con el yaw controlando los 180°. Lo más seguro en un
  carril estrecho, ~4 s.
- **`esquina`** — espera a llegar a una esquina, donde sobra sitio, y encadena
  dos giros. Más rápido y elegante, depende de detectar bien la esquina.

Ojo con un detalle que costó encontrar: para medir 180° **no sirve** la diferencia
angular contra el rumbo inicial, porque se envuelve y al pasar de 180 empieza a
bajar. Hay que acumular el giro paso a paso; así está hecho.

## Los sensores

Hoy el **MPU6050** y el **TCS34725** van al ESP32, con sus pines INT. El firmware
los **autodetecta**: si el chip no contesta al arrancar, no se usa y todo sigue
igual. Puedes enchufarlos cuando quieras sin recompilar.

```
ESP32          MPU6050 / TCS34725
GPIO21  ──►    SDA
GPIO22  ──►    SCL
GPIO34  ◄──    INT del MPU6050
GPIO35  ◄──    INT del TCS34725
3V3 / GND      alimentación
```

**Qué manda el ESP32 y qué no.** No manda muestras crudas: eso saturaría el
serial para nada.

- **Rumbo**: el ESP32 integra el giroscopio a 200 Hz y publica el yaw **ya en
  grados** a 50 Hz, o antes si se movió más de 0.4°. La Pi recibe un número, no
  un chorro de lecturas.
- **Color**: solo se manda **cuando cambia**. Cruzar una línea son dos tramas de
  11 bytes por esquina, no un flujo continuo.

**Y si mañana los pasas a la Raspberry**, solo cambias una palabra en
`robot.json`:

```json
"sensores": { "origen_rumbo": "pi", "origen_color": "camara" }
```

En `auto` gana el ESP32 si reporta que tiene el chip; si no, se prueba el I2C de
la Pi; si tampoco, se sigue sin él. El cambio es **en caliente**: si desenchufas
el sensor a mitad de prueba, a los pocos segundos se cae a la alternativa sin
que nadie reinicie nada.

El clasificador de líneas trabaja **en relativo**, no con umbrales fijos de RGB:
primero toma una muestra del piso blanco (botón *Calibrar color*) y después solo
importa cuánto se aleja el color medido de ese blanco. Así el mismo umbral vale
con luz de tubo, de LED o de ventana — probado, sin calibrar da falso positivo
con luz cálida y calibrando no.

## El giroscopio (opcional de verdad)

Si prefieres el MPU6050 en el **I2C de la Raspberry** en vez del ESP32, pon
`"origen_rumbo": "pi"` y cablea:

```
VCC -> pin 1 (3V3)     SDA -> pin 3 (GPIO2)
GND -> pin 6 (GND)     SCL -> pin 5 (GPIO3)
```

Comprueba con `i2cdetect -y 1` (debe salir `68` o `69`). Si no está, si el bus
no existe (Windows) o si se suelta un cable a mitad de carrera, el programa
sigue igual, solo que sin ayuda de rumbo — está probado.

**Calibra el giroscopio con el carro quieto** (botón *Calibrar* en el panel o en
la web) cada vez que enciendas. Sin eso el yaw se va solo 1-3 grados por segundo
y a la tercera recta el rumbo objetivo ya no significa nada.

---

## El enlace con el ESP32

Trama de 11 bytes, binaria, con CRC:

```
A5 5A | LEN | TIPO | payload | CRC8
```

`vel` y `dir` viajan en **porcentaje con signo**, no en unidades de hardware: la
Pi no sabe nada de grados de servo ni de PWM. El mapeo lo hace el firmware, que
es quien conoce los límites físicos. Por eso un bug en la Pi no puede pedir un
ángulo imposible: no existe forma de expresarlo en el protocolo.

Por qué binario y no texto: un `A95\n` obliga a parsear en el ESP32 mientras el
motor espera, y un byte de ruido convierte `A95` en `A9` (el servo se va a 9
grados). Con LEN y CRC, una trama con ruido simplemente no existe.

`protocolo.py` y `protocolo.h` son gemelos y `selftest_robot.py` cruza vectores
generados por el binario de C++ contra los de Python: si alguien toca uno solo,
la prueba falla.

---

## Seguridad: quién reacciona antes

1. **El navegador** frena cuando ve que el pasillo se cierra.
2. Si el lazo de visión se atasca **>250 ms**, el hilo del enlace manda
   velocidad 0. No repite la última orden: repetir una orden vieja es
   exactamente lo que hace que un carro siga a fondo contra la pared.
3. Si el serial se calla **>300 ms**, el ESP32 corta el motor y **centra el
   servo** (el carro se detiene recto, no torcido).
4. Si la tarea de control del ESP32 se cuelga **>200 ms**, una tarea vigilante
   de prioridad máxima corta el PWM directamente sobre el hardware.
5. Ctrl+C, cerrar la ventana o perder la cámara mandan parada de emergencia.

### La última línea de defensa del servo

En `firmware/esp32_carro/seguridad.h`, `SERVO_TOPE_MIN = 50` y
`SERVO_TOPE_MAX = 145` son constantes de compilación. La configuración en
caliente solo puede **estrechar** ese rango, jamás ampliarlo. El límite se
aplica **cuatro veces**: al convertir el porcentaje a grados, al recortar contra
izquierda/derecha, dentro de la rampa, y otra vez justo antes de escribir el
LEDC. La prueba `test_firmware.cpp` barre porcentajes de -1000 a +1000 y
comprueba que ninguno se sale.

Además el servo tiene **límite de grados por segundo** (320 por defecto): un
MG996R yendo de 65 a 135 de golpe arranca la cremallera.

Y el motor **nunca invierte el giro sin pasar por cero y esperar 150 ms**.
Invertir un puente H a plena marcha es un pico de corriente que, además, el
reglamento WRO limita a 4 A.

---

## Calibrar los colores

```bash
python3 tools/calibrador.py                     # cámara 0
python3 tools/calibrador.py --imagen foto.png   # sobre una foto
```

1. Elige el color: `rojo`, `verde`, `negro` (los que se usan) y `magenta`,
   `naranja`, `azul` (estacionamiento y líneas del piso, listos para cuando
   toquen).
2. **Clic izquierdo sobre el objeto**: calcula el rango HSV solo. Con *acumular*
   activado cada clic amplía la muestra — haz clic en la cara iluminada y en la
   sombreada del mismo pilar.
3. Afina con los sliders; cada uno tiene su casilla para escribir el valor exacto.
4. Nombre y **Guardar perfil**. Se conservan las 5 últimas.

Los perfiles guardados antes de que existieran los colores nuevos los heredan
solos, sin pisar lo que ya tenías calibrado.

**Congela la exposición y el balance de blancos de la cámara** antes de
calibrar. Si siguen en automático, al girar hacia una pared blanca la cámara
reajusta y tu calibración HSV deja de valer.

---

## Reglamento que condiciona todo esto

Futuros Ingenieros 2026: tapete 3200×3200 mm con pista interior de 3000×3000 mm,
piso blanco, muros exteriores e interiores de 100 mm de alto y negros por
dentro, 7 señales verdes y 7 rojas de 5×5×10 cm. Robot máximo 30×20×30 cm,
hardware y software libres.

---

## Qué sigue

El análisis de los cinco métodos de navegación, con los tres mejores a fondo y
en qué orden implementarlos, está en `docs/metodos_navegacion.html`. Resumen:

1. **Medir primero** con lo que ya hay: da vueltas con el centrado y mira en
   `carrito.local` el ancho de carril autocalibrado, los segundos hasta el muro
   en las esquinas y cuántas veces entra en escape.
2. **Mezcla por confianza**: que cada estrategia diga cuánto se fía de sí misma
   y su peso se multiplique por eso. Barato y reversible.
3. **Arcos de dirección**: evaluar la curva que el carro puede recorrer de
   verdad, no un punto. Lo que más mejoraría, y lo que más cuesta.
4. Reto de obstáculos: el rojo por la derecha y el verde por la izquierda,
   usando `Deteccion.base_y` como distancia y `desviacion()` como error lateral.
5. Estacionamiento en la zona magenta.
