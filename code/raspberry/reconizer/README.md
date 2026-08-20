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
│   ├── navegacion.py     perfil del muro y estrategias de esquive
│   ├── robot.py          el núcleo que lo une todo
│   ├── robot_config.py   robot.json
│   └── servidor.py       carrito.local: vídeo + control desde el móvil
├── tools/
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
la estrategia y las ganancias, en caliente.

El panel de escritorio (`main.py` lo abre solo si hay pantalla) muestra lo mismo
más los parámetros finos que en el móvil estorban.

**El carro arranca siempre DESARMADO.** Hay que pulsar ARMAR para que el motor
pueda moverse. `Escape` en el panel y el botón rojo en la web son parada de
emergencia.

Para limitar la velocidad de esta prueba: el slider *vmax* (0-255) es el tope de
PWM que el ESP32 no supera nunca, y *crucero* / *en giro* son porcentajes de ese
tope. `python3 main.py --vmax 90` lo fija desde la línea de órdenes.

---

## Cómo no chocar con los muros: qué opciones hay

Lo exploré en `src/navegacion.py`, que lleva el razonamiento completo. Resumen:

| Opción | Qué tal | Estado |
|---|---|---|
| **Área de negro en dos ventanas** | Cuatro líneas, pero el muro del fondo (lejos, inofensivo) pesa igual que el de al lado (cerca, peligroso). No distingue distancia. | Descartada |
| **Perfil de contacto muro-piso, columna por columna** | Para cada columna, el píxel negro más bajo es donde el muro toca el suelo: cuanto más abajo, más cerca. Sale un perfil de distancia de ancho completo, como un LIDAR pobre. | **Implementada, por defecto** |
| **Seguir una pared a distancia fija** | Trayectorias muy limpias y repetibles, pero hay que decirle qué pared y se pierde si esa pared desaparece. | **Implementada, seleccionable** |
| **Vista de pájaro (homografía) + pure pursuit** | Es lo "correcto" y lo que hacen los equipos fuertes, pero exige calibrar la homografía con un patrón. El perfil de arriba ya es su entrada natural. | Siguiente paso |
| **Giroscopio como rumbo** | La pista es un cuadrado: los giros son de 90° exactos. La cámara decide *cuándo* girar, el giroscopio decide *cuánto*. Es lo que más sube la fiabilidad. | **Implementada como capa encima** |
| **Ultrasonidos laterales** | Un muro de 100 mm visto en ángulo rebota el eco hacia otro lado y devuelve "sin obstáculo" justo cuando vas a chocar. | Solo como red de último metro |

Tu idea de medir el negro en la franja central en vez del píxel más bajo está
generalizada: se mide en **todas** las columnas y luego se agrega por zonas
(izquierda / pasillo de las ruedas / derecha). Las dos líneas verticales
blancas del vídeo son por dónde pasan las ruedas, calibrables con `rueda izq` y
`rueda der` porque la cámara no las ve.

La capa de **seguridad** es independiente de la estrategia: si el pasillo de las
ruedas baja de `frenar_bajo` se reduce la velocidad, y si baja de `parar_bajo`
el carro retrocede girando al revés para reencuadrar. Eso corre siempre,
incluso en modo manual: no te deja empotrar el carro aunque se lo pidas.

### Parámetros que vas a tocar en la pista

| Parámetro | Qué hace |
|---|---|
| `girar_bajo` | Espacio libre por debajo del cual asume que hay esquina y gira |
| `frenar_bajo` / `parar_bajo` | Dónde empieza a frenar y dónde se planta |
| `kp` / `kd` | PD del centrado. Sube `kp` si va lento a corregir, sube `kd` si oscila |
| `dir_giro` | Cuánto vuelca la dirección en las esquinas |
| `min_recto_ms` | Espera mínima entre dos esquinas: evita que encadene giros sobre sí mismo |
| `px_min_columna` | Píxeles negros mínimos en una columna para creerse que hay muro |
| `ignorar_abajo` | Franja inferior tapada por el chasis |

---

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

1. Probar vueltas a la pista vacía y ajustar `girar_bajo` y `kp`/`kd`.
2. Contar vueltas con las líneas naranja y azul del piso (los colores ya están).
3. Reto de obstáculos: pasar el rojo por la derecha y el verde por la izquierda,
   usando `Deteccion.base_y` como distancia y `desviacion()` como error lateral.
4. Estacionamiento en la zona magenta.
