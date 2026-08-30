# Guía de puesta a punto — Reto Abierto

Este documento responde a tres cosas: **qué hace cada sensor**, **cómo decide
el carro**, y **qué se calibra, en qué orden y con qué precisión**. Todos los
números vienen de medir, no de estimar: al final de cada sección se indica con
qué herramienta se comprobó.

---

## 1. Qué hace cada sensor

### Cámara — mide distancias, no "ve paredes"

La cámara no clasifica «pared interior» y «pared exterior». Eso es justo lo que
fallaba antes: las dos son negras, se tocan en las esquinas y el fondo de la
sala también es oscuro. Lo que hace es **medir el espacio libre**:

1. **Recorte geométrico.** La cámara está a 125 mm y los muros miden 100 mm.
   Como la cámara está *por encima*, la arista superior de cualquier muro cae
   siempre **por debajo** del horizonte de la imagen. Todo lo que aparece por
   encima es sala, mesas y gente, y se descarta antes de procesar. Además se
   recorta por rango máximo (`roi_x_max_mm`, 2200 mm).
2. **Máscara acromática oscura.** Un píxel es muro si es **oscuro**
   (`wall_v_max`) **y poco saturado** (`wall_s_max`). Lo segundo es lo que
   impide que las líneas naranja y azul del tapete, o los pilares rojo y verde,
   entren como muro: el negro no tiene color.
3. **Contorno del suelo libre.** Para cada columna se busca, subiendo desde
   abajo, la primera racha de N píxeles oscuros (`wall_min_run_px`). Ese punto
   es la **base** del muro, que está sobre el plano del suelo — por eso la
   proyección inversa da milímetros exactos ahí y sólo ahí.
4. **Tramos rectos.** El contorno se lleva a coordenadas del suelo y se parte
   en segmentos. Cada tramo se clasifica por **orientación**, no por color:
   alineado con el eje del robot → muro lateral (izquierdo o derecho);
   transversal → muro frontal.
5. **Esquinas convexas.** Cuando el contorno da un salto brusco a un punto
   mucho más lejano, ahí se acaba un muro. Eso es el final del muro interior.

De ahí salen las cuatro cifras que gobiernan todo: **distancia al muro
izquierdo, al derecho, al frente, y dónde se acaba el muro interior**.

### Giroscopio (MPU‑6050) — lleva el rumbo

Sólo se usa el eje Z, integrado a 200 Hz en el ESP32, con el sesgo estimado
automáticamente al arrancar con el robot quieto. El rumbo es la **referencia
principal de dirección**:

- Las curvas son exactas: en cada esquina se suman 90° al rumbo objetivo, y el
  giro termina cuando el rumbo llega, no cuando pasa un tiempo. No depende de
  la velocidad, la batería ni el agarre.
- Si la visión falla un fotograma, el robot **sigue recto** en lugar de dar un
  volantazo. La visión sólo corrige la posición lateral, y esa corrección está
  saturada (`lat_head_max_deg`).
- La deriva del giroscopio se corrige sola: en las rectas, el ángulo del muro
  medido por visión dice cuánto se ha desviado el rumbo, y se compensa despacio
  (`yaw_vision_gain`). Eso es lo que hace que la tercera vuelta salga como la
  primera.

### Sensor de color (TCS34725) — deduce el sentido de marcha

Mira al suelo y detecta las líneas de 20 mm del tapete. Su trabajo principal no
es contar vueltas, es **saber hacia dónde hay que girar**, y hay que deducirlo
solo porque el reglamento (9.9) prohíbe introducir ese dato antes de la ronda.

En cada esquina del tapete hay una línea naranja y una azul que salen del
vértice interior. Del plano oficial se deduce el orden:

| Primera línea que se cruza | Sentido | Giros | Muro interior |
|---|---|---|---|
| **Azul** | antihorario | a la izquierda | a la izquierda |
| **Naranja** | horario | a la derecha | a la derecha |

Se normaliza R,G,B contra la suma para no depender del nivel de luz, y además
el robot mantiene una referencia de blanco que actualiza solo mientras ve
tapete. Secundariamente, los contadores de líneas sirven de verificación del
conteo de vueltas.

**Redundancia:** si el sensor de color falla, la visión también deduce el
sentido — el muro que **termina** en una esquina convexa cercana es el interior,
porque el exterior sigue de largo hasta la esquina lejana (que es cóncava y no
produce salto). En el simulador esta vía sola acierta en los cuatro casos.

---

## 2. Flujo del Reto Abierto

```
ARMAR
  │  se pone el rumbo a cero y se calibra el sesgo del giroscopio
  ▼
SIGUE_MURO ───────────────────────────────────────────────┐
  │  rumbo_deseado = rumbo_objetivo + corrección_lateral   │
  │  dirección     = k_heading × (rumbo_deseado − rumbo)   │
  │                                                        │
  │  ¿frente < turn_trigger_front_mm (660)?                │
  ▼ sí                                                     │
GIRO                                                       │
  │  rumbo_objetivo += ±90°, sin corrección lateral        │
  │  termina cuando |error de rumbo| < 15°                 │
  │  esquinas++ ; vueltas = esquinas / 4                   │
  └──────── ¿vueltas < 3? ────────────────────────────────┘
                  │ no
                  ▼
              FINAL  →  avanza hasta que el muro interior
                        se acabe a finish_end_mm  →  PARADO
```

### Cómo decide dónde girar

Esto es lo que más cuesta acertar y lo que estaba mal en la primera versión.

El disparo es **la distancia al muro de enfrente**, y la razón es geométrica:
ese muro **será, después del giro, el muro exterior del siguiente pasillo**. Un
giro de 90° con radio *R* adelanta al robot exactamente *R*. Así que si el giro
empieza cuando el frente está a *R + holgura*, el robot termina a esa holgura
del nuevo muro exterior. Dos propiedades muy útiles:

- **No depende del ancho del siguiente carril** (600 o 1000 mm, y el reglamento
  los mezcla al azar).
- **No depende de por dónde del carril veníamos**: cada curva *recoloca*
  lateralmente al robot. Un error acumulado se borra en cada esquina.

Con radio ~300 mm y holgura ~360 mm salen los 660 mm por defecto.

> **Por qué NO se usa el final del muro interior como disparo.** Parece lo
> natural, y es lo que rompía el carro. La geometría dice que para bordear la
> esquina hay que empezar el giro cuando el vértice interior está a
> `R − distancia_objetivo` por delante, que con R ≈ 300 y objetivo 340 es
> **negativo**: hay que pasar la esquina *antes* de girar. Usarlo como disparo
> hace que el robot corte contra el muro interior, que es exactamente lo que
> pasaba. Ahora ese dato se usa sólo para deducir el sentido y para «armar» la
> curva (congelar la corrección lateral y no meterse contra un muro que se
> acaba).

### Cómo se mantiene en el carril

```
error         = distancia_al_muro_interior − objetivo
corrección    = k_lateral × sentido × error        (saturada a lat_head_max_deg)
```

`objetivo` sale de `wall_mode`: `inner` pega al muro interior
(`target_inner_mm`), `center` va centrado, `adaptive` centra en pasillos de
600 mm y pega al interior en los de 1000. En el Reto Abierto conviene ir por
dentro: el reglamento (9.18) prohíbe tocar el muro exterior, y además se
recorre menos distancia.

Si el muro interior aún no se ve (pasa al arrancar lejos de él: para ver un
muro a *Y* mm de lado hay que estar a más de *Y/tan(fov/2)* de él), el robot se
acerca despacio al lado interior con `seek_inner_deg` hasta que aparece.

---

## 3. Cómo montar la cámara

Barrí el ángulo de montaje **con el robot bien calibrado a ese ángulo** y
cualquier valor entre 6° y 28° completa las 3 vueltas. Es decir: **el ángulo
concreto importa poco; lo que importa es que el programa sepa cuál es.** Dicho
eso, la geometría sí favorece un rango:

| Inclinación | Ve el suelo desde | Filas útiles | Horizonte (fila) |
|---|---|---|---|
| 0° | 231 mm | 235 | 240 |
| 10° | 180 mm | 274 | 184 |
| **18°** | **150 mm** | **320** | **136** |
| 26° | 126 mm | 370 | 84 |
| 36° | 100 mm | 430 | 8 |

**Recomendación: 15–25° hacia abajo, a los 125 mm que ya tienes.**

- Inclinada ve el suelo desde ~150 mm en vez de 231 mm. Eso importa cuando hay
  que salir de un roce.
- Dedica más filas de imagen al suelo, así que mide mejor.
- El horizonte queda dentro de la imagen con margen (fila ~136). Pasados los
  30° el horizonte se sale por arriba y pierdes el margen que absorbe el
  cabeceo del chasis al frenar y acelerar.

**Lo que la cámara debe ver:** el tapete ocupando la mitad inferior larga del
encuadre, la línea donde el muro toca el suelo bien visible a lo ancho, y por
encima algo de sala (que el programa recorta solo). Si el muro llena media
imagen, estás demasiado cerca o demasiado bajo.

**Lo más importante del montaje no es el ángulo, es la rigidez.** La
inclinación es el parámetro más sensible de todo el sistema: aguanta ±3°. Si el
soporte vibra o se mueve 3°, se te va el presupuesto de error completo. Atorníllala,
no la dejes en un soporte con juego, y no la toques después de calibrar.

**El objetivo ancho ayuda.** Con 110–125° de campo el carro dio vueltas ~10 s
más rápidas que con 65–75°, porque ve el muro interior antes y deduce el
sentido por visión sin esperar a la primera línea.

---

## 4. Calibración — el orden importa

El panel trae **calibración asistida**: en vez de mover deslizadores a ojo, le
das una medida de cinta métrica y él resuelve el parámetro. Comprobado en
`tools/test_calibracion.py`: partiendo de valores tan malos como inclinación 24°
y campo 110° (reales 18° y 90°), converge a 18.0/90.1 en **dos rondas**.

### Paso 0 — exposición fija

Vista **Máscara**. El tapete debe salir **negro** y los muros **blancos**. Con
`cam_auto_exposure` desactivado, sube o baja `cam_exposure`.

Esto no es opcional: con exposición automática el brillo cambia solo, el umbral
deja de valer, y además el driver baja los FPS para poder exponer más tiempo.

### Paso 1 — umbral

Botón **Medir umbral**. Mide el histograma de la región de interés y coloca el
corte entre muro y tapete. Si avisa de que la separación es menor de 45 niveles,
vuelve al paso 0: no hay contraste suficiente.

### Paso 2 — inclinación

Robot **mirando de frente a un muro**. Mide con cinta desde su punto de
referencia hasta la **base** del muro (400 mm va bien), escríbelo y pulsa
**Calibrar inclinación**.

### Paso 3 — campo de visión

Robot **dentro de un pasillo, mirando a lo largo de él**, viendo los dos muros.
Escribe el ancho real del pasillo y pulsa **Calibrar FOV**.

### Paso 4 — repite 2 y 3

Los dos parámetros están acoplados (cambiar el campo de visión mueve también la
componente vertical del rayo). Una segunda ronda basta.

### Paso 5 — comprobación

Vista **Vista de pájaro**. Los muros deben salir **rectos** y a la distancia que
marca la cinta. Si salen curvados, la inclinación sigue mal. Si salen rectos
pero a la distancia equivocada, es la altura o el campo de visión.

### Paso 6 — dirección y motor, en modo manual

- `servo_center_us` hasta que ruede recto con dirección a 0.
- `servo_left_us` / `servo_right_us` **sin llegar al tope mecánico**: si el
  MG996R zumba, lo estás forzando.
- `motor_min_pwm`: sube el acelerador muy despacio hasta que el carro se mueva
  de verdad. Ese es el valor.
- Si al pedir izquierda gira a la derecha → `steer_invert`. Si al avanzar
  retrocede → `motor_invert`. Si al girar a la izquierda el rumbo baja →
  `yaw_invert`.

### Paso 7 — velocidad

Sube `base_speed` sólo cuando la trayectoria sea estable. Tres vueltas caben de
sobra en tres minutos: prioriza terminar.

---

## 5. Qué parámetro importa cuánto

Medido descalibrando la cámara **real** respecto a la que cree el robot, con el
simulador (`tools/simulador.py --miscal`). Referencia: 90° / 18° / 125 mm.

| Parámetro | Tolerancia real | Si te pasas |
|---|---|---|
| `cam_pitch_deg` | **±3°** | choca en la primera o segunda curva |
| `cam_hfov_deg` | ±6° | pierde muros, tarda o no termina |
| `cam_height_mm` | ±13 mm o más | apenas afecta |

Es decir: **si el carro se va a otro lado, empieza por la inclinación.** Es el
único parámetro con un margen tan estrecho que se rompe sin avisar.

Los siguientes en importancia, ya sobre la conducción:

| Parámetro | Qué mueve | Síntoma si está mal |
|---|---|---|
| `turn_trigger_front_mm` | dónde empieza la curva | alto: se abre y roza el exterior. bajo: corta hacia el interior |
| `k_heading` | agresividad del rumbo | alto: zigzag. bajo: tarda en enderezar |
| `k_lateral` | corrección de carril | alto: zigzag. bajo: se queda descentrado |
| `target_inner_mm` | trazada | por debajo del radio de giro (~300) no puede bordear la esquina |
| `wall_v_max` | qué es muro | alto: sombras entran como muro. bajo: pierde muros |

---

## 6. Diagnóstico rápido

Con el panel abierto, mira las pastillas de arriba:

| Qué ves | Qué significa |
|---|---|
| `cam` por debajo de 18 fps | exposición automática o formato YUYV — pulsa **Consultar cámara** |
| `tirón` por encima de 200 ms | la captura se atasca: cable USB, alimentación o CPU |
| `visión` mucho menor que `cam` | el lazo no da abasto: baja resolución o `control_hz` |
| `sentido ?` mucho rato | ni el sensor de color ni la visión deciden — mira los contadores naranja/azul |
| `puntos de contorno` < 40 | la máscara no encuentra muros: umbral o exposición |
| `ancho pasillo` no coincide con la cinta | campo de visión mal calibrado |
| `muro izq/der` con `q` baja | el ajuste es poco fiable: pocos puntos o muro muy corto |

Y en la vista **Superposición**: la línea que recorre la base de los muros debe
pegarse al suelo. Si sube hacia la sala, tienes el umbral mal o la región de
interés mal recortada.

---

## 7. Herramientas

```bash
python3 tools/simulador.py                    # 6 escenarios completos de pista
python3 tools/simulador.py --caso ccw_1000 --video
python3 tools/simulador.py --miscal cam_pitch_deg=22   # cuánto error aguanta
python3 tools/simulador.py --fps 15           # efecto de una cámara lenta
python3 tools/test_calibracion.py             # convergencia de la calibración
python3 tools/test_percepcion.py              # precisión de las medidas
```

El simulador usa **el mismo** código de percepción y control que el robot, así
que un cambio que rompe el simulador rompe el carro.
