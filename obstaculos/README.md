# Sistema de control — Reto de Obstáculos

Pila completa y autocontenida para el **Desafío de Obstáculos** de WRO 2026
Future Engineers. Raspberry Pi 5 (visión y decisiones) + ESP32 (hardware y
failsafe), unidos por serial a 115200 8N1.

No hay nada de estacionamiento en este código. Los delimitadores magenta del
cajón sí aparecen, pero **como muro intocable**, nunca como objetivo: la regla
9.25.7 termina la ronda en el instante en que el vehículo los toca.

---

## Mapa de conexiones

| Función | Pin ESP32 | Detalle |
|---|---|---|
| Tracción RPWM | GPIO 25 | IBT-2 (BTS7960), PWM 20 kHz / 8 bits |
| Tracción LPWM | GPIO 26 | idem |
| Tracción R_EN | GPIO 27 | habilitación; en BAJO = puente en alta impedancia |
| Tracción L_EN | GPIO 33 | idem |
| Dirección | GPIO 32 | servo MG996R, 50 Hz, 500–2400 µs, 16 bits |
| I2C SDA | GPIO 21 | bus a 400 kHz |
| I2C SCL | GPIO 22 | |
| MPU-6050 | 0x68 | INT (data ready) → GPIO 18, push-pull, activo ALTO |
| TCS34725 | 0x29 | INT (umbral de claro) → GPIO 19, open-drain, activo BAJO |
| Botón de inicio | GPIO 13 | a GND, pull-up interno |
| LED de estado | GPIO 2 | azul de a bordo |
| Serial2 RX2 | GPIO 16 | ← TX de la Pi |
| Serial2 TX2 | GPIO 17 | → RX de la Pi |
| Cámara | — | WN-L1812.K56R (IMX179) por USB 2.0 a la Pi, MJPG 640×480 @30 |

Las dos patas INT son **opcionales**: el firmware detecta solo si están
cableadas y, si no, sigue por sondeo. Con ellas el rumbo se integra con el `dt`
real del sensor y los cruces de línea llevan marca de tiempo del ISR.

---

## Reparto de responsabilidades

```
Raspberry Pi 5                             ESP32
──────────────                             ─────
cámara → visión → geometría                tareaRx        valida CRC, publica
        ↓                                  tareaSensores  MPU + TCS por I2C
  carril  +  señales   (por separado)      tareaControl   100 Hz, rampas, HW
        ↓                                  tareaVigilante corta si algo calla
      mezcla → FSM → Orden
        ↓
   hilo TX a 50 Hz  ─────── serial ──────► failsafe a 300 ms
```

Regla de diseño: **la Pi piensa, el ESP32 obedece y desconfía.** Si la Pi se
cuelga, el ESP32 corta la tracción él solo a los 300 ms. Si la tarea de control
del ESP32 se cuelga, el vigilante baja los enables a los 200 ms.

El motor necesita **dos llaves**: el botón físico (`armadoLocal`, que vive en el
ESP32) y la bandera `F_ARMADO` de la Pi. Ninguna de las dos basta por sí sola.

---

## Estructura

```
firmware/esp32/
  esp32_obstaculos.ino   pines, PWM, tareas FreeRTOS, failsafe
  protocolo.h            trama binaria  ← gemelo de src/protocolo.py
  seguridad.h            topes del servo, rampas, protección de inversión
  sensores_i2c.h         MPU-6050 y TCS34725 a registro
  lineas.h               clasificador naranja/azul del piso
  panel.h                botón antirrebote + patrones del LED

raspberry-pi/
  main.py                punto de entrada
  config/pista.json      todo lo ajustable
  src/protocolo.py       trama binaria  ← gemelo de protocolo.h
  src/enlace.py          serial: hilo RX + hilo TX a 50 Hz
  src/camara.py          captura MJPG en hilo, buffer de uno
  src/geometria.py       píxeles → milímetros sobre el suelo
  src/vision.py          segmentación HSV → escena en milímetros
  src/carril.py          mantenerse en la pista
  src/senales.py         rebasar los pilares por el lado que manda la regla
  src/vueltas.py         contar secciones y vueltas, saber dónde parar
  src/fsm.py             máquina de estados
  src/piloto.py          lazo principal
  src/panel.py           ventana de depuración (solo taller)
  tools/selftest.py      pruebas sin carro
```

---

## Puesta en marcha

**1. Firmware.** Arduino IDE con el core **ESP32 3.x** (la API `ledcAttach` /
`ledcWrite(pin, ...)` es de la 3.x; con la 2.x no compila). Abrir
`firmware/esp32/esp32_obstaculos.ino` y subir. No hace falta ninguna librería
externa: los dos sensores están escritos a registro sobre `Wire`.

**2. Pi.**

```bash
cd obstaculos/raspberry-pi && pip install -r requirements.txt
```

**3. Comprobar sin carro.**

```bash
python3 tools/selftest.py
```

**4. Rodar.**

```bash
python3 main.py
```

y pulsar el botón del carro cuando el juez diga «¡ya!».

En el taller: `--sin-motor` (pone el techo de PWM a 0, así ninguna rama del
código puede mover nada), `--ver` (ventana de depuración), `--puerto COM7`.

---

## Calibración, en el orden en que hay que hacerla

Cada paso supone que el anterior está bien. Saltarse el orden es la forma
habitual de perder una tarde persiguiendo un fallo que estaba dos pasos antes.

1. **Servo.** Con el carro levantado, comprobar que `centro` deja las ruedas
   rectas y que `izquierda`/`derecha` no hacen tope contra la cremallera. Los
   topes duros están en `seguridad.h` como constantes de compilación: el JSON
   solo puede estrechar ese rango, nunca ampliarlo.

2. **Geometría de la cámara.** El parámetro que de verdad importa para los
   pilares es **`fy_px`**: la distancia a una señal se deduce de su altura
   aparente, y eso solo depende de la focal. Resuélvelo poniendo un pilar a
   una distancia conocida.

   `inclinacion_deg` y `alto_cam_mm` son secundarios a propósito: solo afectan
   a la medida por base, que ahora únicamente corrobora. Un mástil torcido ya
   no hace desaparecer los pilares — antes sí, y por eso el carro los
   ignoraba. Si aun así quieres afinarlos, la ventana de `--ver` muestra las
   dos distancias juntas: cuando se separan mucho, vuelve a medir el mástil.

3. **Colores.** Con `--ver`, sobre el tapete de verdad y con la luz de verdad.
   Lo que importa no es que el pilar se vea entero: es que **nada más** se vea.

4. **`pwm_min_motor`.** Súbelo de 5 en 5 (arranca en 55) hasta que el carro
   arranque limpio desde parado sobre el tapete, y ni uno más. Es lo que evita
   que el motor zumbe sin moverse a bajo porcentaje.

5. **`mm_s_por_pct`.** Mandar 40 % durante 3 s en recta, medir la distancia,
   dividir. De este número dependen el odómetro y la duración del compromiso.

6. **`color_entrada_horario`.** Empujar el carro a mano por la pista en sentido
   horario y mirar qué color reporta el primer cruce. **No se adivina**: de
   esto depende que el sentido de la ronda se deduzca bien.

---

## Lo que este sistema no sabe hacer

Dicho antes de que lo descubra la pista:

- **No hay encoder en las ruedas.** El odómetro se integra del porcentaje de
  mando (`mm_s_por_pct`), con un error del 10–15 %. Basta para medir el tramo
  de recta hasta la meta, pero no serviría para navegar por estima. Por eso el
  detector de atasco mira **la imagen**, no la velocidad estimada: preguntarle
  al acelerador si el coche se mueve no sirve de nada cuando está clavado.
- **Un pilar cada vez.** Con dos pilares seguidos se resuelve el primero y el
  segundo se replantea después. Planificar los dos a la vez con una sola cámara
  sale peor de lo que parece.
- **El firmware no está compilado en este repositorio.** No hay compilador
  cruzado en la máquina donde se escribió: `protocolo.h`, `seguridad.h` y
  `lineas.h` son C++ puro a propósito y se pueden verificar con `g++` en un PC,
  pero el `.ino` hay que abrirlo en el IDE para confirmarlo.
- **Los rangos HSV del `pista.json` son un punto de partida**, no una
  calibración. Ninguna tabla de colores sobrevive a un pabellón nuevo.

---

## Si el carro vuelve a ignorar los pilares

Mira **`descartes`** en la ventana de `--ver` (o en `piloto.ultimo`). Dice
exactamente qué filtro se está comiendo los contornos:

| Qué aparece | Qué significa | Qué tocar |
|---|---|---|
| `descartes` vacío y `pilares vistos: 0` | la máscara HSV no ve nada | rangos de `rojo` / `verde` en `pista.json` |
| `rojo:area` | los ve, pero muy pequeños | baja `area_min` |
| `rojo:horizonte` | caen por encima del horizonte | sube `margen_horizonte_px`, y remide `inclinacion_deg` |
| `rojo:forma` / `rojo:llenado` | la mancha no parece un pilar | afloja `aspecto_min/max`, `llenado_min` |
| `geometria_discrepa` alto | mástil torcido; **ya no descarta**, solo baja confianza | remide `inclinacion_deg` cuando puedas |
| detecta pilares pero `esquive_peso` sigue a 0 | la puerta de sección los está tapando | pon `usar_limite_seccion: false` |

---

## Reglas del juego que están metidas en el código

Con la cita al lado, para que se puedan comprobar contra el reglamento:

| Regla | Dónde vive |
|---|---|
| 9.19 rojo por su derecha, verde por su izquierda | `senales.py` → `LADO_OBLIGADO` |
| 9.11 / 9.13 estado de espera y un solo botón | `fsm.py` → `Estado.ESPERA`, `panel.h` |
| 9.9 nada de calibrar a mano en el vehículo | `sensores_i2c.h`: el sesgo del giro se mide solo al encender |
| 9.25.7 tocar un delimitador termina la ronda | `carril.py` → `margen_magenta` |
| Apéndice A.5 hay margen para corregir el lado | `fsm.py` → `Estado.CORRECCION` |
| 9.23 volver a la sección de arranque y parar | `vueltas.py` → `evaluar_parada` |
| 13.1 pilar de 50 × 50 × 100 mm | `senales.py`, `geometria.py` |
| 13.9 líneas naranja y azul de 20 mm | `lineas.h` |
