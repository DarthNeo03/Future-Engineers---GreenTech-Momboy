# WRO Future Engineers 2026 — Reto Abierto + detección de obstáculos

Código de control para el vehículo autónomo: **Raspberry Pi 5** (visión, estrategia,
panel web) + **ESP32** (PWM del puente IBT_2, servo MG996R, MPU‑6050, TCS34725).

```
New Code/
├── firmware/esp32_wro/       Firmware del ESP32 (Arduino)
│   ├── esp32_wro.ino         Lazo principal, protocolo serie, actuadores
│   ├── config.h              Pines y constantes
│   ├── mpu6050.h             Giroscopio por registros (sin librerías externas)
│   └── tcs34725.h            Sensor de color + detector de líneas del tapete
└── rpi/
    ├── run.py                Punto de entrada
    ├── requirements.txt
    ├── wro/
    │   ├── params.py         Registro de los 120 parámetros calibrables
    │   ├── camera.py         Captura UVC en hilo aparte
    │   ├── geometry.py       Proyección inversa de perspectiva (IPM)
    │   ├── perception.py     Detección de muros  ← el núcleo del proyecto
    │   ├── obstacles.py      Pilares rojo/verde y delimitadores magenta
    │   ├── controller.py     Máquina de estados y ley de control
    │   ├── overlay.py        Vistas de depuración
    │   ├── robot.py          Orquestador
    │   ├── server.py         Servidor web (Flask)
    │   └── static/           Panel (HTML/CSS/JS sin dependencias)
    └── tools/
        ├── simulador.py       Simulador de lazo cerrado de la pista
        └── test_percepcion.py Pruebas de la medida de muros con escenas exactas
```

---

## 1. Por qué fallaba la detección de muros (y qué se hace aquí)

El enfoque anterior intentaba **clasificar en la imagen** «muro interior» vs
«muro exterior». Eso no puede funcionar de forma fiable: los dos muros son
negros, se tocan en las esquinas, y el fondo de la sala también es oscuro, así
que el contorno se escapa hacia mesas, sillas y personas (se ve en tus capturas:
el trazo amarillo subiendo por el fondo).

Aquí no se clasifica nada en la imagen. La cadena es:

**1) Recorte geométrico.** La cámara está a **125 mm** y los muros miden
**100 mm**. Como la cámara está *más alta* que los muros, la arista superior de
cualquier muro cae siempre **por debajo del horizonte**. Fijando además un
alcance máximo (2,2 m) sale una fila concreta de la imagen por encima de la cual
no hay nada útil, y se recorta antes de procesar. **Ese recorte, por sí solo,
elimina todo el fondo de la sala.**

**2) Máscara acromática oscura.** Muro = poco brillo **y** poca saturación. Las
líneas naranja y azul del tapete y los pilares rojo/verde quedan fuera por ser
saturados; el muro negro es acromático.

**3) Contorno del suelo libre.** Para cada columna se sube desde abajo hasta la
primera racha de N píxeles oscuros. Ese punto es la **base** del muro, que está
sobre el plano del suelo → la proyección inversa es exacta ahí.

**4) Geometría, no color.** El contorno se lleva a milímetros y se parte en
tramos rectos (cortes por hueco + split‑and‑merge). Cada tramo se clasifica por
su **orientación**: alineado con el eje del robot → muro lateral; transversal →
muro frontal. Nunca por su aspecto.

**5) La esquina es un salto convexo.** Donde acaba el muro interior, el contorno
salta de golpe a un punto mucho más lejano. La esquina *cóncava* del muro
exterior no produce ese salto. Ese salto es lo que permite (a) deducir cuál es
el muro interior y por tanto el **sentido de marcha**, y (b) anticipar la curva.

**Validación** (`python3 tools/test_percepcion.py`, escenas sintéticas de
geometría exacta): pasillo de 1000 mm → mide 1002; pasillo de 600 → mide 603;
muro interior que acaba a 800 mm → detecta el final a 796; robot girado 12° →
mide −11,6°. **18/18 comprobaciones**, error por debajo del 1 %. Se mantiene
18/18 con campo de visión de 110°, con inclinaciones de 10° a 28° y con
distorsión de barril hasta `lens_k1= −0,2`.

Un detalle que costó encontrar: los tramos se parten por distancia a la **cuerda
entre los extremos** (Douglas‑Peucker), no por el residuo de un ajuste global.
En una esquina en «L» el punto que más se separa de la recta ajustada cae en los
*extremos*, no en el vértice, así que con el ajuste global la esquina no se
parte nunca y los dos muros salen fundidos en un solo tramo con una orientación
sin sentido.

---

## 2. Estrategia de conducción

**El rumbo lo lleva el giroscopio; la posición lateral la lleva la visión.**

```
rumbo_deseado = rumbo_objetivo (múltiplo de 90°) + corrección_lateral
dirección     = k_heading × (rumbo_deseado − rumbo_actual)
```

Ventajas frente a un PID sobre la distancia al muro:

* si la visión falla un fotograma, el robot sigue recto en vez de dar un volantazo;
* las curvas son exactas: se suman 90° al rumbo objetivo, no dependen del tiempo
  que el volante esté girado;
* la corrección lateral está saturada, así que un error grande de visión no
  puede tumbar la trayectoria.

Además, el ángulo del muro medido por visión corrige lentamente la deriva del
giroscopio (fusión complementaria). Eso es lo que hace que la tercera vuelta
salga igual que la primera.

### Trazada
Se sigue el **muro interior** a `target_inner_mm` (340 por defecto). El
reglamento **prohíbe tocar el muro perimetral exterior en el Reto Abierto**
(regla 9.18), así que ir por dentro es a la vez lo más seguro y lo más corto.
Hay modos `center` y `adaptive` si prefieres ir centrado.

### Cuándo girar — esto es lo que más cuesta acertar

El disparo principal es la **distancia al muro de enfrente**, no el fin del muro
interior. La razón es geométrica:

> El muro que tienes delante **será el muro exterior del siguiente pasillo**, y
> un giro de 90° con radio *R* adelanta al robot exactamente *R*. Si empiezas a
> girar cuando el frente está a *(R + holgura)*, terminas a esa holgura del
> nuevo muro exterior — **da igual que el siguiente carril mida 600 o 1000 mm, y
> da igual por qué parte del carril vinieras**. Cada curva *recoloca*
> lateralmente al robot.

Con *R* ≈ 300 mm (coche de 20 cm) y holgura ≈ 380 mm salen los **720 mm** por
defecto de `turn_trigger_front_mm`.

⚠️ **No uses el fin del muro interior como disparo.** Para bordear la esquina
habría que empezar a girar cuando la esquina está a *(R − distancia objetivo)*
por delante, que con *R* ≈ 300 y objetivo 340 es **negativo**: hay que pasarla
antes de girar. Usarlo como disparo hace que el robot corte contra el muro
interior — es un error fácil de cometer y difícil de diagnosticar. Aquí el fin
del muro solo *arma* la curva (`corner_arm_mm`), que atenúa la corrección
lateral para no seguir tirando hacia un muro que se acaba.

### Sentido de marcha (se deduce solo, como exige el reglamento)

El reglamento sortea el sentido **después** de la revisión técnica y prohíbe
introducir datos al programa (reglas 9.3, 9.9, 12.6). Se detecta por dos vías:

1. **Sensor de color.** Cada esquina tiene una línea azul y una naranja que salen
   del vértice interior. Medido sobre el plano oficial: **azul primero →
   antihorario** (giros a izquierda), **naranja primero → horario**.
2. **Visión.** El muro que termina en un salto convexo cercano es el interior;
   el exterior sigue de largo. Sirve de respaldo si el sensor de color falla.

`direction_source` permite forzarlo, **pero solo para pruebas**.

### Vueltas y parada
Se cuentan esquinas: 4 por vuelta, 12 en total. Tras la última curva el robot
avanza hasta que la esquina del muro interior queda a `finish_end_mm` (600 mm),
lo que lo deja hacia la mitad del tramo recto — bien dentro de la sección de
meta, como pide la regla 9.25.2 (la proyección entera debe quedar dentro). Hay
un tiempo máximo de respaldo y un corte global a los 175 s (la ronda dura 180).

---

## 3. Puesta en marcha

### ESP32
Abre `firmware/esp32_wro/esp32_wro.ino` con el IDE de Arduino (o `arduino-cli`),
placa **ESP32 Dev Module**, y súbelo. Solo necesita `Wire.h`; los drivers del
MPU‑6050 y del TCS34725 están escritos por registros para no depender de
librerías. Compila igual con el core 2.x y con el 3.x de Arduino‑ESP32.

Pines: los tuyos, ya configurados en `config.h`. Añadidos opcionales:
**GPIO 4** = botón de arranque (a GND) y **GPIO 2** = LED de estado. El
reglamento (9.11) exige un botón de arranque: puedes usar ese o uno en la Pi
(`start_button_pin`).

### Raspberry Pi 5
```bash
sudo apt install -y python3-opencv python3-numpy python3-flask python3-serial
cd rpi
python3 run.py
```
El panel queda en `http://<ip-de-la-pi>:8000`. La primera ejecución crea
`config.json` con los valores por defecto.

Da permiso al puerto serie una vez: `sudo usermod -aG dialout $USER` (y reinicia
la sesión).

### En competencia
```bash
sudo rfkill block wifi bluetooth      # regla 11.10: nada de inalámbrico
python3 run.py --no-web
```

---

## 4. Calibración — en este orden

El panel explica cada parámetro con el botón **?**. Este es el orden que
funciona:

**a) Exposición fija.** Vista **Máscara**: el tapete debe salir todo negro y los
muros todo blancos. Ajusta `cam_exposure` (sube si está oscuro) con
`cam_auto_exposure` desactivado. Con exposición automática el umbral de muro
deja de valer en cuanto cambia la luz.

**b) Inclinación de la cámara — el parámetro más crítico.** Pon el robot mirando
de frente a un muro, mide con cinta desde el punto de referencia del robot hasta
la **base** del muro, escribe el número en el panel y pulsa **Calibrar**.

> Medido en simulación: un error de **±2°** degrada la trayectoria pero el robot
> termina; un error de **−3°** (crees que mira más abajo de lo que mira) hace que
> calcule las distancias **más cortas** de lo real, gire antes de tiempo y
> **choque contra el muro interior**. Es, con diferencia, lo que más hay que
> cuidar. La altura tolera bien un 8 % de error y el campo de visión unos 7°.

**c) Campo de visión.** Coloca el robot centrado en un pasillo de ancho conocido
y ajusta `cam_hfov_deg` hasta que **ancho pasillo** coincida con la cinta.

> Cuanto más ancho, mejor. Para ver un muro que está a *Y* mm de lado hay que
> estar a más de *Y/tan(fov/2)* de él: con 90° el muro interior a 340 mm se ve
> desde 340 mm por delante, pero con 70° hace falta estar a 485 mm. Con objetivos
> estrechos el robot se queda sin referencia lateral justo al arrancar, que es
> para lo que existe `seek_inner_deg`.

**Comprobación permanente sin cinta métrica:** el panel muestra el ancho de
pasillo junto a su referencia (`600` o `1000`, los dos únicos que permite el
reglamento) y el error en %. Si con los dos muros a la vista ese error se va por
encima del 8 %, la geometría está mal calibrada. Es la forma más rápida de
detectar en vivo un problema de calibración.

**d) Vista de pájaro.** Es la mejor herramienta de diagnóstico: si la
calibración está bien, un muro recto se dibuja **recto** y a la distancia real.
Si sale curvado, sobra o falta inclinación (o hay distorsión de barril → prueba
`lens_k1` entre −0,35 y −0,05).

**e) Hardware.** En modo **Manual**: `servo_center_us` hasta que ruede recto,
`servo_left_us`/`servo_right_us` sin llegar al tope mecánico (forzarlo quema el
MG996R), y `motor_min_pwm` subiendo el acelerador despacio hasta que el robot
se mueva. Estos van marcados con **●** en el panel y se envían al ESP32 al
instante.

**f) Sensor de color.** Pasa el sensor por encima de cada línea mirando los
valores `r g b` en vivo y ajusta los umbrales.

**g) Conducción.** `base_speed` a 35‑45 al principio. Si zigzaguea, baja
`k_heading` o `k_lateral`. Si sale de las curvas pegado al muro exterior, **sube**
`turn_trigger_front_mm`; si sale demasiado abierto o corta hacia dentro, bájalo.

---

## 5. El panel

* **Superposición** — contorno, tramos clasificados por color, retícula del suelo
  proyectada, límites del recorte y todos los números del control.
* **Vista de pájaro** — el suelo en milímetros: contorno, rectas ajustadas a cada
  muro, la ✕ donde termina el muro interior, la banda frontal y la trayectoria
  objetivo. **Empieza a depurar siempre por aquí.**
* **Máscara** — para ajustar exposición y umbrales.
* **Manual** — joystick táctil o flechas del teclado, con tope de velocidad
  ajustable. Si el mando deja de enviar durante 0,6 s el robot frena solo, y el
  ESP32 corta la tracción si la Pi deja de hablar durante 350 ms.
* **Barra espaciadora = PARAR** en cualquier momento.
* **Calibración** — los 120 parámetros por grupos, con buscador, filtro de
  avanzados, y la explicación de cada uno en el botón **?**.

Cada intento armado deja un CSV en `rpi/logs/` para analizarlo después.

---

## 6. Simulador

Reproduce la pista oficial y realimenta *render → percepción → control →
modelo de bicicleta*, sin robot ni cámara:

```bash
cd rpi
python3 tools/simulador.py                              # 6 casos
python3 tools/simulador.py --caso ccw_mix --video        # con vídeo de depuración
python3 tools/simulador.py --set base_speed=65           # probar parámetros
python3 tools/simulador.py --miscal cam_pitch_deg=20     # probar descalibración
```

Estado actual: **6/6 casos** completan 3 vueltas con exactamente 12 esquinas, sin
tocar ningún muro, y paran en la sección de meta (~95‑98 s a `base_speed=45`,
~83 s a 65). Se mantiene 6/6 también con `base_speed=65`, `cam_hfov_deg=70`,
`wall_mode=center` y `target_inner_mm=280`.

Los casos cubren: pasillos de 1000 mm en ambos sentidos, dos combinaciones de
carriles mixtos 600/1000, y un arranque pegado al muro exterior (el caso en que
el muro interior **no se ve** al principio, que resuelve `seek_inner_deg`).

Ajusta `MAX_SPEED_MM_S`, `MAX_STEER_DEG` y `WHEELBASE_MM` en la cabecera del
simulador a los de tu robot para que las predicciones sean útiles.

---

## 7. Obstáculos (base para el Reto con Obstáculos)

Detección por HSV de pilares rojos y verdes: el punto útil es el **centro de su
base**, que está sobre el suelo, así que la misma proyección da su posición en
milímetros sin ambigüedad de escala. Como verificación cruzada se compara con la
distancia deducida de la altura aparente (los pilares miden 100 mm); si
discrepan mucho, la detección se marca con baja confianza. Se detectan también
los delimitadores magenta del estacionamiento (solo se muestran).

Ojo con un sesgo fácil de pasar por alto: el borde inferior visible de un pilar
es su **cara frontal**, no su centro, así que hay que sumar media profundidad
(25 mm). Sin esa corrección todas las distancias salen cortas de forma
sistemática. Verificado con pilares sintéticos a 600 / 1000 / 1500 mm: error de
3 / 14 / 28 mm, y las medidas de los muros no se ven afectadas por tener pilares
en escena (el filtro de saturación los deja fuera de la máscara de muro).

El control aplica la regla 9.19 — **rojo se rebasa por su derecha, verde por su
izquierda** — mezclando el objetivo de esquiva con el seguimiento de muro según
la distancia al pilar. Actívalo con `obstacles_enabled` y el modo **Obstáculos**.

**No implementado:** la maniobra de estacionamiento en paralelo y la gestión de
pilares en las curvas. Es lo siguiente a abordar cuando el Reto Abierto esté
sólido en pista.

---

## 8. Reglamento — lo que condiciona el código

| Regla | Consecuencia en el código |
|---|---|
| 9.18 | No tocar el muro exterior en el Reto Abierto → `outer_min_mm` y trazada por dentro |
| 9.3 / 9.9 / 12.6 | Sentido sorteado tras la revisión, prohibido introducir datos → detección automática |
| 9.11 | Un solo botón de arranque → `start_button_pin` (Pi) o GPIO 4 (ESP32) |
| 11.10 | Prohibida la comunicación inalámbrica → `--no-web` + `rfkill` en competencia |
| 9.25.2 | Parada autónoma con la proyección entera dentro de la sección de meta |
| 9.1 | 3 minutos por ronda → corte a 175 s |
| Sección 8 | Carriles de 600 o 1000 mm (±100) por tramo → ancho medido en vivo y objetivo adaptativo |
| 13.9 / 13.6 | Líneas 20 mm naranja/azul, muros negros de 100 mm → umbrales y recorte geométrico |
