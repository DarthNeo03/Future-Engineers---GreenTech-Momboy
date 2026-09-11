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
firmware/esp32/esp32_obstaculos/     (la carpeta debe llamarse como el .ino:
  esp32_obstaculos.ino   pines, PWM, tareas FreeRTOS, failsafe   lo exige el IDE)
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
  src/vueltas.py         contar secciones y vueltas, y de qué lado se corre
  src/fsm.py             máquina de estados
  src/piloto.py          lazo principal
  src/panel.py           ventana de depuración (solo taller)
  tools/selftest.py      pruebas sin carro
```

---

## Puesta en marcha

**1. Firmware.** Arduino IDE con el core **ESP32 3.x** (la API `ledcAttach` /
`ledcWrite(pin, ...)` es de la 3.x; con la 2.x no compila). Abrir
`firmware/esp32/esp32_obstaculos/esp32_obstaculos.ino` y subir. No hace falta ninguna librería
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

   De este único parámetro salen las dos respuestas: el color suelto da el
   sentido *provisional* en cuanto se pisa la primera línea, y el par
   ordenado (entrada + el otro color) lo *confirma* al completar la primera
   esquina. En `--ver`, la línea `sentido` dice cuál de las dos está mandando.

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

## Qué pasa justo después de esquivar

Es el momento más delicado de la vuelta. El pilar ya no se ve —la cámara no
llega tan abajo— y el carro va apuntando hacia el lado por el que lo rebasa.

- **El seguidor de carril está callado a propósito.** Si opinara, tiraría del
  carro hacia el centro del carril y la rueda trasera barrería el pilar: con
  dirección Ackermann la cola corta por dentro.
- **Lo que se sostiene es el rumbo, no el volante.** Se congela el *yaw* del
  MPU del instante en que el pilar entró en zona ciega y un proporcional
  devuelve el carro a él. Un volante fijo no traza una recta, traza un arco:
  con el modelo Ackermann simplificado, congelarlo 0,8 s desviaba **537 mm**,
  más de medio carril, directo al muro.
- **Pero el compromiso no termina soltando el carro donde esté: lo devuelve
  alineado.** Pasada `salida_desde` (55 %) del compromiso el pilar ya quedó
  atrás, y el rumbo objetivo se lleva poco a poco del congelado al del pasillo
  libre que ve el carril. Sin esto, el carril recibía un carro cruzado y con el
  muro delante, que es de donde salía el volantazo siguiente.
- **Sin MPU se endereza, y rápido.** No hay rumbo que sostener, así que el
  volante se suelta en `soltar_sin_mpu_s` (0,35 s) de reloj y el resto del
  adelantamiento se hace recto. Antes se desvanecía a lo largo de *todo* el
  compromiso, o sea que se mantenía casi entero durante la primera mitad: eso
  es el arco, otra vez, y contra la pared. Aun así, revisa el I²C: recto es lo
  mejor que se puede hacer a ciegas, no es tan bueno como sostener el rumbo.
- **La guardia anti-muro sí sigue despierta** (`guardia_muro` en `carril.py`).
  No centra: se calla mientras haya sitio y solo empuja cuando la separación
  lateral baja de `guardia_muro_mm`. Su autoridad crece con el progreso del
  adelantamiento, porque al principio el pilar sigue al costado y al final ya
  quedó atrás.

### Y después del compromiso hay un estado propio: REINCORPORACION

Lo que sale de un rebase no es un carro en pista: es un carro descolocado
hacia un lado y apuntando al muro, así que su cámara ve el frente cerrado.
Devolverlo directamente a `PISTA` era el fallo que dejaba al carro girando
hacia el lado del esquive hasta clavarse en la pared, porque el seguidor de
carril leía ese frente cerrado como una esquina y hacía las dos cosas que no
tocaban a la vez: bajar el centrado al 25 % —el único término que aparta al
carro del muro— y empujar con `sesgo_curva` hacia el lado al que giran las
curvas de la ronda.

En `REINCORPORACION` el piloto le pasa al carril `tras_pilar=True`, y con eso:

| Qué se apaga | Por qué |
|---|---|
| el sesgo de curva | ese frente cerrado no es una esquina, es el muro al que se apunta |
| el aprendizaje del sentido | esos grados de rumbo no son de ninguna curva; si entraran en el promedio, el sentido "aprendido" sería el del último esquive |
| la rebaja del centrado | es lo único que saca al carro del muro |

Se sale a `PISTA` cuando el carro está de verdad centrado y enfilado
(`reincorporado_err`, `reincorporado_deg`) o cuando se agota
`reincorporacion_max_s`. Un pilar nuevo interrumpe el estado y manda a `SENAL`:
recolocarse no puede ser excusa para pasar de largo la señal siguiente.

Medido en el escenario de prueba (carro a 150 mm del muro derecho después de
rebasar por la derecha, ronda horaria): el volante pasa de **−12 %** —un
empujón simbólico, porque el sesgo de curva tiraba hacia el muro— a **−66 %**.

### Un muro de frente no es un muro al costado

Fallo de fondo que salió al montar un escenario de prueba con la geometría de
verdad, y que explica por qué la guardia anti-muro nunca salvaba al carro. La
separación lateral se medía sobre todos los sectores con muro, y un muro
**frontal** ocupa todos los rumbos: su sector a 4 grados del morro, a 700 mm,
da `700·sen(4°) = 49 mm`. O sea que con el carro perfectamente centrado en un
carril de 1000 mm y una esquina a 700, el código reportaba muros a 46 mm **a
los dos lados**. Y como la guardia empuja con la diferencia, los dos empujones
se cancelaban y daba **exactamente 0** justo cuando hacía falta.

Ahora solo cuenta como costado lo que está más cerca que el frente
(`frac_frente_lateral`). Lo que hay delante ya se mide aparte, en
`dist_frente_mm`.

---

## Cómo sabe hacia dónde girar

En una pista WRO **todas las curvas van hacia el mismo lado**: en sentido
horario todas a la derecha, en antihorario todas a la izquierda. O sea que
basta acertar una para saber las once restantes. El seguidor usa tres cosas,
en este orden:

1. **El rumbo del hueco.** Se parte el frente en 32 sectores, se mide a qué
   distancia aparece el primer sólido en cada uno, y se apunta al **centro del
   tramo contiguo libre más ancho**. Al centro, no al sector más profundo: en
   una recta media imagen empata en el máximo y quedarse con el primero
   sesgaba el volante a la izquierda en cada recta.

2. **La separación lateral a cada muro**, para centrarse. Cada sector con muro
   se convierte en un punto del suelo (`x = d·sen θ`, `y = d·cos θ`) y se mide
   cuánto hay de verdad a cada lado. Si solo se ve un muro —lo normal dentro de
   una curva— se sigue a distancia fija de él.

3. **El sentido de las curvas**, que el carro **aprende de la primera que
   toma** (promediando el rumbo del hueco mientras está en curva). Sirve para
   las esquinas de frente, donde el muro exterior tapa la salida y el hueco
   queda casi centrado: ahí es donde antes se iba recto contra la pared.

   Mientras no haya tomado ninguna curva, acepta como pista el sentido que
   deduce `vueltas.py` de las líneas del piso. Es solo una ayuda de arranque:
   el sentido aprendido sigue mandando sobre ella, porque no depende de ningún
   parámetro, y así un `color_entrada_horario` mal puesto no puede hacer girar
   al revés.

   **Este aprendizaje solo come datos limpios.** Con `tras_pilar` no se
   acumula nada: si los ciclos de después de un rebase entraran en el
   promedio, el sentido que se "aprendería" sería el del último esquive, y a
   partir de ahí el carro empujaría hacia ese lado en las once curvas
   restantes.

**Y el sesgo lleva dos candados**, los dos por el mismo fallo de pista: nunca
se aplica con un rebase en curso o recién terminado, y nunca empuja **hacia**
un muro que ya está a menos de `guardia_muro_mm`. El sesgo existe para
resolver un hueco indeciso, no para meter el morro en la pared; si la esquina
es de verdad, el término de rumbo la resuelve igual en cuanto haya sitio.

En la ventana de `--ver`: `hueco a N deg`, `sesgo` y `curvas hacia ...` dicen
qué está pensando, y salen avisos de `TRAS PILAR` y `MURO ENCIMA` cuando el
sesgo está callado. Si `curvas hacia` sigue en `sin aprender` después de dos
curvas, sube `sesgo_curva` o baja `aprender_curva_deg`.

## De qué lado se corre esta ronda

El reglamento sortea el sentido antes de cada ronda (9.3) y prohíbe metérselo
al programa a mano (9.4, 9.9), así que hay que leerlo de la pista. Se lee dos
veces, y la segunda corrige a la primera:

| Cuándo | Cómo | Qué vale |
|---|---|---|
| primera línea pisada | su color, contra `color_entrada_horario` | **provisional** |
| primera esquina entera | el **orden** del par naranja/azul | **firme** |

El par es el que de verdad contesta. La primera línea solo acierta si es de
verdad la de entrada, y en pista el TCS se salta líneas —una línea de 20 mm a
1 m/s dura 20 ms—: cuando se salta la de entrada, la de salida se toma por la
primera y **el sentido sale invertido**. Un carro que cree que las curvas van
al otro lado se estampa en la primera esquina de frente, y el fallo no se
parece en nada a su causa. Con el par da igual cuál de las dos se vea primero
si se ven las dos, porque lo que se mira es el orden.

Tres cautelas, y las tres evitan contestar mal en vez de no contestar:

- **Una línea suelta no se empareja con la de la esquina de al lado.** Caduca
  por `ventana_par_mm` (1500 mm; la sección de curva mide 1000) y abre un par
  nuevo. Un par inventado es peor que ningún par: contesta, y al revés.
- **Si los dos colores suben en el mismo ciclo de la Pi, no forman par.** Los
  cruces llegan como contadores acumulados, no como eventos con marca de
  tiempo, así que ahí el orden no se sabe.
- **Una vez firme, el sentido no cambia.** En esta pista no hay media vuelta:
  un par al revés es un cruce mal leído. Se cuenta en `pares_incoherentes` y
  se sigue.

Si `--ver` avisa de que **las líneas y las curvas no dicen el mismo sentido**,
una de las dos fuentes está mal, y lo más probable es `color_entrada_horario`
al revés — el aprendizaje del carril no depende de ningún parámetro.

---

## Si dice que terminó la ronda nada más pulsar el botón

Mira el resumen que imprime. Si las vueltas no cuadran con `dist_m`, el
contador se desbocó, no es que haya corrido:

```
'vueltas': 63, 'secciones': 255, 'cruces': 255, 'dist_m': 0.3
```

255 es la saturación de un contador de 8 bits, y 63 vueltas en 0,3 m es
imposible. Las tres redes que lo impiden:

| Red | Qué hace |
|---|---|
| `max_cruces_por_ciclo` | entre dos ciclos de la Pi no caben más de 2-3 cruces reales; por encima se resincroniza en vez de contar. Un reinicio del contador del ESP32 (200 → 0) daba un delta de 56 por la resta de 8 bits |
| contar solo con la ronda en marcha | en `ESPERA` el carro lleva minutos quieto delante del juez, y todo lo que entre ahí es ruido |
| `min_dist_por_vuelta_mm` | tres vueltas no caben en tres metros; una vuelta real ronda los 8 m |

Los contadores se ponen a cero **en el flanco del botón**, no al arrancar el
programa. Hacerlo en `preparar()` abría una carrera: la Pi tomaba su línea base
antes de que el ESP32 llegara a aplicar el `CAL_CERO_LINEAS`.

Si ves `resync` subiendo en la telemetría durante la ronda, el enlace serial
está perdiendo tramas: revisa el cableado de `Serial2` antes que el contador.

---

## Si no abre la cámara

```bash
python3 main.py --listar-camaras
```

Dice qué `/dev/video*` existen y cuál está ocupado, leyendo `/proc` (no
necesita `fuser` ni `lsof`, que no vienen en Raspbian Lite).

| Lo que sale | Qué pasa |
|---|---|
| `Device '/dev/video0' is busy` | otra ejecución la tiene abierta → `pkill -f "python3 main.py"` |
| `No hay ningun /dev/video*` | no está enchufada o el kernel no la vio → `lsusb`, `dmesg \| tail` |
| abre pero no entrega imagen | es el nodo de **metadatos** de la UVC, no el de captura → `v4l2-ctl --list-devices` y `--camara N` |

Si el índice del JSON falla, el programa prueba solo los demás `/dev/videoN` y
avisa de cuál acabó usando. El orden de enumeración puede cambiar con un
reinicio, y quedarse sin correr por eso teniendo el resto listo sería absurdo —
pero conviene fijar el índice bueno en `pista.json` igualmente.

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

## Si el carro gira sin sentido justo después de rebasar un pilar

Este era EL fallo: rebasaba bien y acto seguido seguía girando hacia el lado
del esquive hasta clavarse en la pared. Mira estas cuatro cosas de `--ver`, en
este orden, que es el orden en que se encadenaban las causas:

| Qué mirar | Qué significa si sale mal |
|---|---|
| `lateral izq`/`der` con una esquina de frente | si los dos bajan a la vez con el carro centrado, el muro frontal se está colando en la medida lateral: revisa `frac_frente_lateral` |
| el estado después del esquive | tiene que ser `reincorporacion`, no `pista` |
| `sesgo` durante esa reincorporación | tiene que ser `0`, y salir el aviso `TRAS PILAR` |
| `sentido` | si dice `(provisional)` después de la primera curva, el par de la esquina no se está completando: el TCS se está saltando una de las dos líneas |

Y si sale el aviso de que **las líneas y las curvas no dicen el mismo
sentido**, para y arregla `color_entrada_horario` antes de seguir rodando: el
carro está funcionando con dos respuestas contradictorias a la pregunta de
hacia dónde giran las curvas.

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
| 9.3 / 9.4 el sentido se sortea y no se configura | `vueltas.py` → `_emparejar` (par ordenado de la esquina) |
| 13.1 pilar de 50 × 50 × 100 mm | `senales.py`, `geometria.py` |
| 13.9 líneas naranja y azul de 20 mm | `lineas.h` |
