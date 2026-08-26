# Carro WRO 2026 — Futuros Ingenieros

Raspberry Pi 5 (visión y decisiones) → ESP32 (hardware) → puente H → motor.
Mismo código en Windows 11 y en Raspbian, Python 3.11.

```
reconizer/
├── main.py                     ← el programa principal
├── config/
│   ├── colors.json             ← las últimas 5 calibraciones de color
│   ├── robot.json              ← puerto, límites, ganancias, red
│   └── suelo.json              ← homografía imagen→suelo (la mide calibrar_suelo)
├── src/
│   ├── color_config.py   perfiles de color (guardado atómico)
│   ├── camera.py         cámara USB igual en los dos sistemas
│   ├── vision.py         máscaras HSV y detección de objetos
│   ├── geometria.py      la cámara como LIDAR 2D métrico
│   ├── protocolo.py      trama binaria hacia el ESP32
│   ├── enlace.py         hilo serie con autodetección de puerto
│   ├── imu.py            MPU6050 opcional por I2C
│   ├── navegacion.py     seguir el muro interno y girar 90° exactos
│   ├── obstaculos.py     señales rojas y verdes del reto de obstáculos
│   ├── robot.py          el núcleo que lo une todo
│   ├── robot_config.py   robot.json
│   └── servidor.py       carrito.local: vídeo + control desde el móvil
├── tools/
│   ├── simulador.py      da vueltas a la pista entera, sin carro
│   ├── calibrar_suelo.py homografía al suelo: se hace UNA vez
│   ├── calibrador.py     interfaz de calibración HSV
│   ├── panel.py          panel de pruebas de escritorio
│   ├── selftest.py       51 pruebas de visión, sin cámara
│   ├── selftest_robot.py 102 pruebas del sistema, sin carro
│   ├── test_firmware.cpp 49 pruebas de la lógica del ESP32, sin ESP32
│   └── carrito_wifi.sh   AP, mDNS, UART, I2C y servicio de arranque
└── firmware/esp32_carro/
    ├── esp32_carro.ino   firmware nuevo (sin WiFi)
    ├── protocolo.h       gemelo en C++ de protocolo.py
    └── seguridad.h       límites del servo y del motor
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
(los tres archivos tienen que estar en la misma carpeta) y súbelo. Ya no hace
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
con el muro detectado, el perfil de distancia, por dónde pasan las ruedas y la
decisión que está tomando. Desde ahí armas, paras, cambias la velocidad máxima,
el reto, la distancia al muro interno y las ganancias, en caliente.

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

El razonamiento completo está en `src/geometria.py` y `src/navegacion.py`.

**La cámara es un LIDAR 2D métrico.** Para cada columna se busca el píxel negro
más bajo — ahí el muro toca el suelo — y ese punto se proyecta al plano del piso
con una homografía medida. Sale un escaneo de 640 rayos en **milímetros**, y
todos los umbrales pasan a ser distancias físicas.

**El muro interno se distingue del externo por geometría, no por color.** En una
esquina el interno presenta una esquina **convexa** que produce un **salto de
rango** (se ve el pasillo siguiente por detrás de ella); el externo presenta una
**cóncava**, de rango continuo. El lado que salta es el interno.

**El sentido se deduce y se bloquea** en la primera esquina. No se puede
introducir por switch: la regla 9.9 lo prohíbe.

### Lo que la pista enseñó y no era obvio

Estas tres cosas salieron de medir sobre la geometría real, y cada una invalidó
un supuesto de diseño que parecía razonable:

**1. El muro interno es casi invisible en mitad de una recta.** Termina en la
esquina, así que subtiende un ángulo diminuto:

| Carro en el corredor sur, HFOV 100° | Columnas del muro interno |
|---|---|
| x = 1500 (esquina aún en el encuadre) | 32 |
| x = 1610 (esquina fuera) | **0** |
| cualquiera, muro externo | ~140 siempre |

Por eso la referencia lateral usa **el muro que se vea**: el interno mientras
esté (justo después de cada giro), y si no el externo con el objetivo trasladado
usando el ancho del corredor estimado. Seguir el externo no es arrimarse a él,
es medirlo para colocarse a `ancho − objetivo`.

**2. La esquina interna desaparece justo cuando hace falta.** A 900 mm de ella
está a 29° de rumbo; a 320 mm, a 57°; a 100 mm, a 79°. Con HFOV de 100° el
límite son 50°. Así que **no se puede disparar el giro con "la esquina está a
320 mm": a esa distancia la cámara ya no la ve.** Se ve pronto, se anota dónde
está, y después se arrastra por estima con el giroscopio y la velocidad.

**3. La distancia de disparo sale de la geometría, no del ojo.** Un giro de 90°
con radio R desplaza el carro exactamente R hacia el lado. Para acabar a
`objetivo` del muro interno nuevo hay que empezar cuando

```
z_esquina = radio_giro_mm − pared_objetivo_mm
```

Con R = 350 y objetivo = 250 son **100 mm**, no los 320 que puse a ojo: con 320
el giro terminaba a 30 mm del muro interno. Y fíjate que **no depende del ancho
del corredor**, que es justo lo que hace falta porque el ancho del corredor
siguiente no se puede medir desde el actual.

### El control lateral va en cascada

La versión directa (`dirección = kp · error_lateral`) parece más simple pero se
lanza contra la pared: con 250 mm de error inicial cualquier ganancia razonable
satura el volante, el carro cruza el corredor en diagonal, se pasa de largo y
pierde el muro de vista.

En cascada el error lateral manda sobre el **ángulo de aproximación**, acotado a
`aprox_max_grados`. Da igual lo lejos que estés: nunca atacas la pared con más
de ~22°. El volante solo persigue ese ángulo. Y sale gratis la amortiguación,
porque el ángulo del muro es en la práctica la derivada del error lateral.

### El giroscopio ayuda, no manda

Dos decisiones que multiplicaron la robustez, medidas en el simulador:

- **El rumbo objetivo de cada giro se toma del yaw actual, nunca encadenado al
  objetivo anterior.** Encadenar solo vale con un giroscopio sin deriva; con
  2 °/s (lo que da un MPU6050 sin calibrar) encadenando salían **0 vueltas
  completas de 32**, y tomando el yaw actual salen **32 de 32**.
- **El giro se cierra por yaw O por visión**, lo que llegue antes: el muro
  interno vuelve a estar paralelo y la esquina ya quedó atrás. Ninguna avería de
  una sola fuente tumba la vuelta.
- **Mientras se vea una pared, el giroscopio no corrige nada**, solo refresca su
  referencia. El ángulo del muro ya es una medida de rumbo relativa al corredor
  y además no deriva.

**El signo importa**: el navegador espera yaw en convenio de brújula, que
**aumenta al girar a la derecha**. Gira el carro a la derecha con la mano y mira
el panel: si el yaw baja, pon `imu.invertir_yaw: true` en `robot.json`.

### Dos trampas que costaron choques

| Trampa | Qué pasaba | Arreglo |
|---|---|---|
| **"No veo muro" = "está despejado"** | Una columna sin detección se marcaba libre. Un fallo de la máscara se volvía vía libre. | Sin contacto fiable la columna es **inválida**; con poca cobertura no se acelera. |
| **El muro que se esfuma por abajo** | Más cerca que el suelo de medida (~200 mm) el muro sale del recorte del chasis y el escaneo canta "despejado" justo antes de chocar. | Memoria de campo cercano: si lo último visto estaba cerca, se conserva y se le resta lo avanzado. |

Una sombra tampoco cuenta ya como muro: se exige una racha vertical continua de
píxeles negros, que un muro de 100 mm siempre tiene y una mancha plana no. Y el
robot avisa al arrancar si `parar_mm` queda por debajo del mínimo medible: si
eso pasa, la parada de seguridad **no puede dispararse nunca**.

### La lente importa más de lo que parece

| HFOV | Muro interno a 250 mm visible desde | Muro externo a 750 mm desde | Vueltas OK |
|---|---|---|---|
| 70° | 357 mm | **1071 mm** |
| 90° | 250 mm | 750 mm |
| 100° | 210 mm | 629 mm |
| 120° | 144 mm | 433 mm |

Con lente estrecha el guardia contra el muro externo no puede dispararse en
campo cercano, que es justo donde importa, y el muro interno tarda mucho más en
entrar al encuadre. El Apéndice D del reglamento pide *"a wide-angle camera"*.
El robot imprime el diagnóstico al arrancar, así que sabrás si tu lente da para
esto antes de la primera prueba.

## El simulador: dar vueltas sin gastar batería

```bash
python3 tools/simulador.py --todas
```

Cierra el lazo: lo que decide el navegador mueve un modelo de bicicleta
Ackermann sobre la pista del reglamento, y moverse cambia lo que la "cámara"
—un trazador de rayos con el mismo campo de visión— ve al frame siguiente.
Ninguna prueba unitaria puede ver que el conjunto se sale en la tercera esquina;
esto sí.

Recorre las 16 combinaciones de ancho del sorteo por los dos sentidos, y admite
`--sin-yaw`, `--deriva`, `--ruido`, `--hfov` y `--senales`. **Encontró la mitad
de los errores de esta versión**, incluidos tres que las 51 pruebas de visión y
las 109 del sistema no podían ver: el rumbo encadenado, el control lateral que
cruzaba el corredor en diagonal, y la distancia de disparo del giro.

Estado actual del Open Challenge:

| Escenario | Vueltas completas sin tocar el exterior |
|---|---|
| Nominal, 16 anchos × 2 sentidos | **32/32** |
| Sin giroscopio | 32/32 |
| Deriva de 2 °/s (MPU6050 sin calibrar) | 32/32 |
| Deriva de 5 °/s | 32/32 |
| Ruido de 25 mm en el escaneo | 32/32 |
| Ruido de 40 mm | 26/32 |
| HFOV 70° (lente estrecha) | 32/32 |

**No sustituye a la pista.** No tiene reflejos, ni exposición automática, ni
holgura en la dirección. Sirve para cazar errores de lógica y de signo antes de
gastar batería, y para comparar ajustes de forma repetible.

## Los dos retos

| | Open Challenge | Obstacle Challenge |
|---|---|---|
| Muros y esquinas | igual | igual |
| Señales | — | **rojo → el carro pasa por su derecha; verde → por su izquierda** |
| Cuándo aplica el desvío | — | solo en recta; en `GIRO` se ignora |

Ojo con el sentido de la regla, que es fácil de entender al revés: el color dice
por qué lado del **carril** va el carro, no por qué lado queda el pilar.
Invertirlo termina la ronda (regla 9.24.5).

**El modo de obstáculos es un adelanto, no está terminado.** Lo que funciona:
detecta los pilares, aplica el lado correcto de la regla, los acota contra los
muros medidos y completa las tres vueltas. Lo que no, medido con
`--senales N`: roza pilares y con dos o más por recta a veces pierde la vuelta.
La causa está identificada — **un pilar tapa el muro que tiene detrás y abre un
hueco en el perfil que se parece a la discontinuidad de una esquina convexa**,
así que el carro se mete en giros de 90° en mitad de la recta. Arreglarlo pide
validar la esquina candidata contra la geometría del corredor, no basta con
bloquear el giro mientras se ve una señal (con varios pilares por recta siempre
se ve alguno y el carro no giraría nunca).

## Calibrar el suelo: hazlo antes que nada

```bash
python3 tools/calibrar_suelo.py
```

Cuatro marcas en el suelo en posiciones conocidas, cuatro clics, y la tecla `v`
para superponer una rejilla métrica de comprobación. Si esa rejilla no cae sobre
el suelo real la calibración está mal, y todo lo demás hereda el error.

Sin `suelo.json` el sistema arranca con un modelo aproximado sacado de altura,
cabeceo y FOV declarados en `robot.json`. Es una muleta para desarrollar sin
tapete delante, no para competir.

### Los tres números que hay que medir antes de la primera prueba

| Parámetro | Cómo se mide |
|---|---|
| `radio_giro_mm` | Con tiza: volante a tope, una vuelta completa, mide el círculo. **Es el número más importante del archivo**: de él sale la distancia de disparo del giro. |
| `hfov_deg` | El campo horizontal REAL de la lente. Mira la tabla de arriba antes de decidir si te vale. |
| `mm_por_seg_a_100` | Cronómetro y cinta métrica a velocidad 100. Solo afecta a la parada final y a la estima de la esquina. |

### Parámetros que vas a tocar en la pista

| Parámetro | Qué hace |
|---|---|
| `pared_objetivo_mm` | Distancia a la que se sigue el muro **interno**. Bajarlo aprieta más por dentro |
| `giro_z_mm` | 0 = automático desde `radio_giro_mm`. Un valor > 0 lo fuerza a mano |
| `min_externo_mm` | Guardia de la regla 9.18: por debajo se ignora todo y se empuja hacia dentro |
| `aprox_max_grados` | Nunca se ataca la pared más inclinado que esto |
| `kp_rumbo` | Ganancia del lazo interior. Súbelo si va lento a corregir |
| `giro_kp` | Salida del giro. Bajarlo suaviza, subirlo cierra antes |
| `parar_mm` / `frenar_mm` | Dónde se planta y dónde frena. `parar_mm` **por encima** del mínimo medible |
| `alto_min_muro_px` | Racha vertical mínima para creerse que hay muro. Súbelo si las sombras dan falsos positivos |

## El giroscopio (opcional de verdad)

El MPU6050 va al **I2C de la Raspberry**, no al ESP32: escupe cientos de
muestras por segundo y lo que hace falta es integrarlas y filtrarlas, no
reaccionar a cada una. Metiendo eso en el ESP32 le robas tiempo al lazo de
control, que sí es crítico. En la Pi corre en su propio hilo a 100 Hz y cuesta
menos del 2% de un núcleo.

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

1. **Calibrar el suelo** y medir el radio de giro real; ajustar `giro_z_mm`.
2. Contar vueltas con las líneas naranja y azul del piso (los colores ya están).
3. Reto de obstáculos: pasar el rojo por la derecha y el verde por la izquierda,
   usando `Deteccion.base_y` como distancia y `desviacion()` como error lateral.
4. Estacionamiento en la zona magenta.
