# Contexto del proyecto — Carro WRO 2026 Futuros Ingenieros

Documento de traspaso. Resume de dónde viene el proyecto, qué decisiones se
tomaron y por qué, cómo funciona el código, y qué se aprendió a base de romperlo.
No es un reglamento: si encuentras una manera mejor de hacer algo, hazla.

---

## 1. El proyecto

Equipo **GreenTech Momboy**, categoría **Futuros Ingenieros** de la WRO Venezuela
2026 (14-22 años, 2-3 estudiantes). Competencia a primeros de septiembre.

El reto: un carro autónomo que da vueltas a una pista cuadrada. Tapete de
3200×3200 mm, pista interior de 3000×3000, **piso blanco**, muros exteriores e
interiores de **100 mm de alto y negros por dentro**. En el reto de obstáculos
hay **7 señales verdes y 7 rojas de 5×5×10 cm** y una zona de estacionamiento
magenta. En el suelo hay una **línea naranja y una azul en cada esquina**.
Robot máximo 30×20×30 cm, hardware y software libres.

**Importante para la navegación:** el ancho del carril **varía** de una pista a
otra en competencia abierta. Nada puede depender de un ancho fijo.

### Hardware

```
Raspberry Pi 5 (8 GB)  →  ESP32  →  puente H  →  motor DC
                            ↓
                          servo MG996R (dirección Ackermann)
```

- **Cámara USB 2.0 WN-L1812.K56R** (sensor IMX179), única fuente de percepción
  del entorno. En el flex pone `WN-L1812.K56RA 32PIN`.
- **MPU6050** y **TCS34725**: conectados al **ESP32** por I2C, con sus pines INT
  cableados. El código admite pasarlos al I2C de la Pi cambiando una palabra.
- El ESP32 está en medio **como medida de seguridad**: si la Pi se cuelga, él
  para el carro.

### Entorno de desarrollo

Se desarrolla **entre el PC (Windows 11) y la Raspberry (Raspbian)**, así que
todo el código tiene que funcionar en los dos sin cambios. Python 3.11 (por
OpenCV y compañía) y C++ para el ESP32.

Repo del equipo:
`OR\Future-Engineers---GreenTech-Momboy\code\raspberry\reconizer`

---

## 2. Ideas y decisiones del equipo

Esto es lo que se pidió y por qué. Son las que mandan.

**Calibración de color rápida.** Detección por color en HSV buscando respuesta
rápida, pero sabiendo que el HSV hay que recalibrarlo según la luz. De ahí:
un archivo con las **últimas 5 calibraciones**, un programa con **sliders y
además casillas para escribir el valor exacto**, y **clic sobre el objeto** en
la imagen para tomar su color. El programa principal elige entre las 5.

**Reconocimiento de objetos.** Partiendo de un enfoque previo donde "las manchas
que se tocan son un mismo objeto", se pidió mejorarlo y que los parámetros
fueran ajustables con slider y escribiendo.

**Medir el muro por columnas, no por un píxel.** La propuesta original fue medir
el píxel negro en la mitad de la cámara y trazar por dónde pasan las ruedas
(porque **la cámara no ve las ruedas**). Eso se generalizó a medir en todas las
columnas.

**Arquitectura del enlace.** Se pidió expresamente: datos muy ligeros, array de
bytes con protección contra ruido, FreeRTOS en el ESP32 y la respuesta de los
motores lo más rápida posible. Y una **última línea de defensa** que compruebe
siempre que al servo no le llegue una orden fuera de límite, para no romper
piezas.

**Sensores al ESP32, procesados donde toque.** La idea fue que el ESP32 mande
datos ligeros a la Pi cuando los sensores detecten cambios, y que si los datos
son muchos los procese la Raspberry. Con compatibilidad para moverlos a la Pi
en el futuro.

**Vueltas y media vuelta.** 3 vueltas de ida y 3 de vuelta, con el número
configurable desde la interfaz de monitoreo. La media vuelta con dos maniobras
elegibles (en recta de tres tiempos, o aprovechando el hueco de la esquina).

**Esquivar pilares.** Rojo y verde, con interruptor para activar y desactivar
desde la interfaz.

**Método de navegación favorito del equipo:** buscar el ángulo entre obstáculos
por donde sea posible pasar, dejando los obstáculos fuera de los bordes por
donde pasan las ruedas, y teniendo en cuenta el siguiente obstáculo.

**Convención de sentido (la del equipo):** pared externa a la **izquierda** =
antihorario; pared externa a la **derecha** = horario. *(Ojo: esto es al revés
de la derivación geométrica habitual — en un anillo, con la externa a la
izquierda el centro queda a tu derecha, que sería horario. Se adoptó la
convención del equipo porque es la que leen en pantalla y porque lo que de
verdad manda el comportamiento es `lado_externo`, no la etiqueta. Si algún día
cuadra mejor al revés, se cambia en un sitio.)*

**Qué funciona mejor hoy, según las pruebas en pista:** la navegación
**centrada**, con la opción de **girar cuando la pared interna deja de verse**.
Es la combinación más rápida y fiable de las probadas.

**Preferencia de interfaz:** todo ajustable en caliente desde el monitoreo, sin
recompilar ni reiniciar. La Pi crea su propio WiFi y retransmite lo que ve,
incluidos los objetos reconocidos, en `carrito.local`.

---

## 3. Arquitectura

```
reconizer/
├── main.py                  arranca todo; --simulado, --imagen, --vmax, --sin-panel
├── config/
│   ├── colors.json          5 perfiles de color rotativos + cuál está activo
│   └── robot.json           puerto, límites, ganancias, red, vueltas, obstáculos
├── src/
│   ├── color_config.py      perfiles de color, guardado atómico, migración
│   ├── camera.py            cámara USB igual en Windows y Linux
│   ├── vision.py            máscaras HSV y detección de objetos
│   ├── protocolo.py         trama binaria hacia el ESP32
│   ├── enlace.py            hilo serie, autodetección de puerto, telemetría
│   ├── sensores.py          rumbo y color: del ESP32, de la Pi, o de ninguno
│   ├── imu.py               MPU6050 por I2C de la Pi (alternativa)
│   ├── navegacion.py        perfil del muro, estrategias, esquinas, escape, media vuelta
│   ├── obstaculos.py        esquiva de pilares rojos y verdes
│   ├── vueltas.py           contador de vueltas fusionando 3 fuentes
│   ├── robot.py             el núcleo que une todo
│   ├── robot_config.py      robot.json
│   └── servidor.py          carrito.local:8080 — vídeo MJPEG + control
├── tools/
│   ├── calibrador.py        interfaz de calibración HSV
│   ├── panel.py             panel de escritorio (Tkinter)
│   ├── widgets.py           slider+casilla compartidos
│   ├── selftest.py          51 pruebas de visión, sin cámara
│   ├── selftest_robot.py    224 pruebas del sistema, sin carro
│   ├── test_firmware.cpp    59 pruebas del ESP32, con g++, sin ESP32
│   └── carrito_wifi.sh      AP, mDNS, UART, I2C, servicio
├── docs/
│   ├── CONTEXTO.md          este archivo
│   └── metodos_navegacion.html  análisis de 5 métodos de navegación
└── firmware/esp32_carro/
    ├── esp32_carro.ino      firmware (sin WiFi)
    ├── protocolo.h          gemelo en C++ de protocolo.py
    ├── seguridad.h          límites del servo y del motor (C++ puro, testeable)
    ├── lineas.h             clasificador naranja/azul (C++ puro, testeable)
    └── sensores_i2c.h       MPU6050 y TCS34725 autodetectados
```

**Regla de diseño que ha funcionado:** todo lo que puede romper hardware o
contar mal vive en C++ puro o Python puro y **se prueba sin el carro**. Por eso
hay 334 pruebas que corren en cualquier PC.

---

## 4. Cómo funciona

### 4.1 Visión y calibración

`colors.json` guarda hasta 5 perfiles; el más nuevo va primero y el sexto
descarta el más viejo. Escritura atómica (tmp + rename); si el JSON se corrompe
se respalda como `.bak` y se regenera. Un perfil guardado antes de que existiera
un color nuevo lo hereda con valores por defecto sin pisar lo calibrado.

Colores definidos: `rojo`, `verde`, `negro` (en uso), `magenta`, `naranja`,
`azul` (listos). El rojo lleva **dos rangos HSV** porque el tono se envuelve en
H=0/179.

**Detección de objetos** (`vision.py`): una conversión a HSV por frame, máscara
multi-rango, `MORPH_OPEN` (mata motas) + `MORPH_CLOSE` (tapa el brillo del
centro del pilar), y luego la parte importante — **fusión por hueco**: se dilata
una *copia* de la máscara y se etiqueta sobre esa copia, pero el área, la caja y
el centro se remiden sobre la máscara **original**. Así un pilar partido en dos
por un reflejo vuelve a ser un objeto sin que la caja se infle. Filtros: área,
**llenado** (área/caja — un pilar sólido da 0.95, el ruido disperso 0.2; es el
que más basura quita), aspecto, ancho/alto mínimos y una **ROI vertical** para
no procesar por encima del muro.

**Toma de color por clic**: el píxel exacto que se clica manda; del parche solo
sobreviven los píxeles parecidos a él. Sin eso, un clic en el borde del objeto
mezclaba fondo y el rango se comía el piso blanco. Si la muestra es acromática
(negro, gris, blanco) el tono se abre a 0-179 y mandan S y V — por eso el mismo
clic sirve para calibrar el negro de las paredes.

### 4.2 El perfil del muro (base de toda la navegación)

Para cada columna de la imagen se busca el **píxel negro más bajo**: ahí el muro
toca el suelo. Cuanto más abajo, más cerca. Sale un perfil de distancia de ancho
completo, como un LIDAR pobre. Es un `argmax` sobre la máscara del revés: una
pasada vectorizada, ~0.3 ms en 640×480.

Del perfil salen: `izq`, `der` (media de las bandas laterales), `pasillo`
(percentil 15 entre las líneas de las ruedas — percentil y no mínimo, para que
una columna con ruido no frene el carro), **`cobertura_izq`/`cobertura_der`**
(qué fracción de cada banda *ve* muro, distinto de a qué distancia está),
**bordes** (escalones del perfil) y **huecos**.

### 4.3 Estrategias de navegación (mezclables con pesos)

- **centrado** — compara el espacio libre izquierda/derecha y gira hacia el
  despejado con un PD. La más tolerante a calibración imperfecta. **La que mejor
  funciona hoy.**
- **pared** — mantiene la pared externa a distancia fija con un PD. En `auto`
  deduce sola cuál es la externa.
- **hueco** — la idea del equipo: tramos por los que el carro **cabe de verdad**,
  contando el ancho de las ruedas a la distancia del obstáculo que delimita el
  hueco. La conversión usa perspectiva de plano de suelo (un ancho real fijo se
  ve proporcional a `y − horizonte`) anclada en las líneas de las ruedas, así que
  no hace falta patrón de calibración. La puntuación incluye el **siguiente
  obstáculo**: si detrás de la boca hay otra cosa, la parte profunda del tramo
  tiene poco espacio libre y el candidato pierde puntos.

Los pesos se suman y se ven los aportes de cada una en la telemetría.
`mezcla` es la fuente de verdad; `estrategia` solo actúa si todos los pesos son 0.

### 4.4 Máquina de estados

`RECTO → PRE_GIRO → GIRO → RECTO`, con `ESCAPE` y `MEDIA_VUELTA` por encima.

- **PRE_GIRO** existe para frenar *antes* de doblar y para dejar que las ruedas
  traseras pasen la esquina interna (`retardo_giro_ms`).
- **GIRO** usa `dir_giro_abierto` (65 %, no a tope) y, si hay hueco pasable,
  apunta a él (`giro_diagonal`), que es lo que traza la diagonal a la siguiente
  pared externa.
- **ESCAPE** cuando el pasillo se cierra por debajo de `parar_bajo`.

### 4.5 Anticipación (el problema de la inercia)

Dos capas:
1. **Envolvente de velocidad** por distancia.
2. **Tiempo hasta el muro**: se mide la *velocidad de cierre* del pasillo y se
   calcula cuántos segundos faltan. Si bajan de `ttc_min` se frena aunque la
   distancia parezca aceptable. Esto es lo que hace que llegue a la esquina ya
   frenado, y es el slider que más se toca en pista.

### 4.6 Identificación de paredes y sentido

El muro **externo se ve casi siempre**; el **interno desaparece en cada esquina**.
Se lleva una **media móvil de "esta banda ve muro"** por lado; la de mayor
presencia es la externa. Votan además: los giros hechos, las desapariciones de
muro y el orden de las líneas del suelo. Se puede **forzar a mano** desde la
interfaz (auto / horario / antihorario).

Tras la media vuelta las presencias se **intercambian** y el estimador queda
**bloqueado unos segundos**.

### 4.7 Disparo de esquina

`DetectorEsquinaInterna` avisa cuando una banda que **tenía** muro deja de
tenerlo, en cualquiera de los dos lados. **No necesita saber el sentido**, así
que gira bien desde la primera esquina. Cuenta como "ya no hay muro" tanto que
la banda se quede sin píxeles como que lo que se ve ahí esté tan lejos que ya
sea la pared de enfrente (`interno_lejos`), porque según el ángulo con el que
llegues pasa una cosa o la otra.

### 4.8 Escape

Retroceso **comprometido**: se calcula cuánto espacio falta y se retrocede al
menos ese tiempo (`escape_atras_min_ms` + extra escalado por el déficit), sin
reevaluar a mitad. Se abandona solo si el espacio de delante no mejora nada en
`escape_atascado_ms`, que es la firma de tener algo pegado detrás; entonces se
prueba el giro hacia adelante. La marcha atrás lleva la dirección **hacia** el
muro, para que el morro se separe (como al salir de un aparcamiento).

### 4.9 Autocalibración del carril

En recta, con poca dirección y el frente despejado, la **suma del espacio libre
de las dos bandas es casi constante** y no depende de por dónde vayas dentro del
carril: esa suma *es* el ancho del carril en unidades del perfil. De ahí se
derivan `parar_bajo`, `girar_bajo` y `frenar_bajo`, acotados. Es lo que permite
competir en pistas de ancho distinto sin tocar nada.

### 4.10 Vueltas

Tres fuentes: **cámara** (líneas del suelo), **TCS34725** (por contacto) y los
**giros de 90°** contados por la navegación. Una esquina se da por buena cuando
se completa un par de líneas o cuando termina un giro, y hay un **periodo
refractario** de 3 s durante el cual no se admite otra venga de donde venga.
Cuatro esquinas = una vuelta. Al llegar al objetivo pide la media vuelta y
cuenta otras tantas en sentido contrario.

### 4.11 Esquiva de pilares

Rojo → se pasa por su **derecha**; verde → por su **izquierda**. Del más cercano
se calcula un objetivo a su lado correcto, separado medio ancho de carro (escalado
por perspectiva) más margen. El objetivo se **recorta al pasillo libre**: sin eso,
esquivar un pilar pegado a la pared manda el carro contra la pared. El peso sube
según se acerca (`activar_desde` → `mandar_desde`).

**Matiz importante:** *pasar por la derecha del pilar* no es *girar a la derecha*.
Si el pilar está a la izquierda del carro, el punto por el que hay que pasar puede
quedar a la izquierda del centro.

### 4.12 Enlace Pi ↔ ESP32

Trama de 11 bytes: `A5 5A | LEN | TIPO | payload | CRC8`.

`vel` y `dir` viajan en **porcentaje con signo**, nunca en unidades de hardware:
la Pi no sabe nada de grados de servo ni de PWM, así que **no existe forma de
expresar un ángulo imposible en el protocolo**. El mapeo lo hace el firmware,
que es quien conoce los límites físicos.

`protocolo.py` y `protocolo.h` son gemelos, y `selftest_robot.py` cruza vectores
generados por el binario de C++ contra los de Python: si alguien toca uno solo,
la prueba falla.

Autodetección de puerto: `/dev/serial0`, `ttyAMA0`, `ttyUSB*`, `ttyACM*`, `COM*`.
Se manda un PING y se espera respuesta. El firmware escucha **las dos bocas** (USB
y GPIO16/17) y contesta por la que recibió la última trama válida.

### 4.13 Seguridad, en orden de quién reacciona antes

1. El navegador frena cuando ve que el pasillo se cierra.
2. Si el lazo de visión se atasca >250 ms, el enlace manda velocidad 0. **No
   repite la última orden**: repetir una orden vieja es exactamente lo que hace
   que un carro siga a fondo contra la pared.
3. Si el serial se calla >300 ms, el ESP32 corta el motor y **centra el servo**.
4. Si la tarea de control del ESP32 se cuelga >200 ms, una tarea vigilante de
   prioridad máxima corta el PWM directamente sobre el hardware.
5. Ctrl+C, cerrar la ventana o perder la cámara mandan parada de emergencia.

**Límites del servo**: `SERVO_TOPE_MIN=50` y `SERVO_TOPE_MAX=145` son constantes
de compilación. La configuración en caliente solo puede *estrechar* el rango. El
límite se aplica cuatro veces. Además hay límite de grados/segundo, y el motor
**nunca invierte sin pasar por cero y esperar 150 ms**.

---

## 5. Errores que ya se cometieron (y cómo se detectaron)

Todos salieron de pruebas o de mirar capturas, no de leer el código. Vale la pena
conocerlos porque el patrón se repite.

**Sensores I2C que no aparecían.** El ESP32 los sondeaba en `setup()` justo
después de `Wire.begin()`, antes de que el MPU y el TCS despertaran (y comparten
alimentación con el motor, que hunde el riel al arrancar). Los daba por ausentes
para siempre. **Solución:** esperar 250 ms antes del primer sondeo, **reintentar
solo cada 3 s** mientras falte alguno, y un botón de reintento manual en la
interfaz. *Lección general: cualquier autodetección de hardware necesita
reintento automático + manual + un sitio donde ver si está o no.*

**Acumulador de votos que se saturaba.** El estimador de sentido votaba en cada
frame contra un acumulador con tope ±6: se saturaba en un segundo y luego hacían
falta 48 frames en contra para moverlo. Decía siempre "horario" y **deshacía la
media vuelta**. *Lección: no acumular votos con tope en un bucle que corre a 30
Hz; usar una media móvil que se pueda corregir sola.*

**Medir 180° con la diferencia angular corta.** Se envuelve: al pasar de 180
empieza a bajar y la media vuelta no terminaba nunca. *Hay que acumular el giro
paso a paso.*

**Pausa de inversión del motor que nunca saltaba.** Comparaba contra el PWM
instantáneo; en cuanto llegaba a 0 la condición dejaba de cumplirse y saltaba a
reversa. *Hay que comparar contra el último sentido real de giro y usar reloj.*

**Lector de tramas de una pasada.** Si llegaba una trama truncada, se tragaba los
bytes de la siguiente creyendo que eran su payload y perdía las dos. *Ahora
acumula en buffer y reescanea un byte hacia adelante cuando el CRC falla.*

**Clic de calibración que pisaba el borde.** El parche mezclaba objeto y fondo y
el rango salía tan abierto que la máscara se comía el piso.

**Deduplicación de esquinas demasiado corta.** 2,2 s solo tapaba lo muy seguido;
al cruzar una esquina llegan naranja, azul, a veces naranja otra vez y además el
giro → contaba 3. *Refractario de verdad + dominancia entre las dos líneas.*

**Escape que alternaba adelante/atrás cada 700 ms.** Con el muro encima, 700 ms
de marcha atrás no dan para nada: el carro iba y venía sin ganar sitio hasta
chocar. *Compromiso mínimo escalado por el déficit.*

---

## 6. Estado actual

**Funciona bien:** navegación centrada + giro por desaparición del muro interno
(lo más rápido y fiable), calibración de color, enlace y telemetría, seguridad a
todos los niveles, esquiva de pilares (validada sobre foto real con el perfil
`Prueba1` del equipo).

**Probado en banco, pendiente de pista:** el arreglo de la media vuelta, el
retroceso comprometido, el refractario del conteo de esquinas, el reintento de
sensores, la esquiva en movimiento.

**Sin montar todavía:** MPU6050 y TCS34725 físicamente (el código los espera y
funciona sin ellos).

**Pruebas:** 334 en total.
```bash
python3 tools/selftest.py           # 51, visión, sin cámara
python3 tools/selftest_robot.py     # 224, sistema, sin carro
g++ -std=c++17 -O2 -I firmware/esp32_carro tools/test_firmware.cpp -o /tmp/tfw && /tmp/tfw
```

**Cómo probar sin hardware:**
```bash
python3 main.py --simulado --imagen capturas/loquesea.png
```

---

## 7. Ideas pendientes

Del análisis en `docs/metodos_navegacion.html` (5 métodos, 3 analizados a fondo):

1. **Medir primero** con lo que hay: dar vueltas con el centrado y mirar en
   `carrito.local` el ancho de carril autocalibrado, los segundos hasta el muro
   en las esquinas y cuántas veces entra en escape.
2. **Mezcla por confianza** — que cada estrategia diga cuánto se fía de sí misma
   y su peso se multiplique por eso. Barato y reversible. Ataca directamente el
   "la pared gira tarde": en la esquina no tiene pared que seguir pero sigue
   opinando igual de fuerte.
3. **Arcos de dirección** — evaluar la curva que el carro puede recorrer de
   verdad (radio ≈ batalla/tan(ángulo)) en vez de un punto. Lo que más mejoraría
   y lo que más cuesta; hay que medir la batalla y el ángulo real de las ruedas.
4. Vista de pájaro por homografía y VFH+ quedaron descartados por ahora.

Además: estacionamiento en la zona magenta, y afinar la esquiva con obstáculos
reales en pista.

---

## 8. Notas prácticas

- **Congela la exposición y el balance de blancos** de la cámara antes de
  calibrar. Si siguen en automático, al girar hacia una pared blanca la cámara
  reajusta y la calibración HSV deja de valer.
- **Calibra el giroscopio con el carro quieto** cada vez que enciendas.
- Al subir firmware nuevo, súbelo **antes** de correr `main.py` (si cambió el
  protocolo, el firmware viejo se queda en failsafe).
- Primera prueba del día: `python3 main.py --vmax 70` con las ruedas al aire para
  confirmar el sentido de giro. Si va al revés, `INVERTIR_MOTOR 1` en el `.ino`.
- El carro **arranca siempre desarmado**. Hay que pulsar ARMAR.
- En el pabellón usa el AP propio de la Pi, no la red del recinto.
