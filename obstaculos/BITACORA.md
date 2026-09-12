# Bitácora — sistema de control del Reto de Obstáculos

Traspaso del trabajo sobre `obstaculos/`. El README explica **cómo funciona**
el sistema; esto explica **cómo llegó a ser así**: qué falló, cómo se aisló
cada fallo y por qué la solución es la que es. Si retomas el trabajo —tú,
Cristian, o una sesión nueva— empieza aquí.

Cubre dos tandas: los fallos 1-5 salieron de rodar el carro en pista; los 6-9,
de una segunda sesión desde otro equipo que fue a buscarlos con instrumentos.
Cuatro de esos cuatro estaban en código de la primera tanda.

---

## Estado actual

| | |
|---|---|
| Rama | `obstacle-challenge-control` |
| Base | `main` (`7b9ee69`) |
| Código | ~7.800 líneas: firmware ESP32 + piloto de la Pi 5 |
| Pruebas | **165 comprobaciones**, `python3 tools/selftest.py`, sin hardware |
| Firmware | compila con `arduino-cli`; **no** se ha ejecutado en el ESP32 |
| Verificado en pista | detección y esquive de pilares ✔ |
| Sin verificar en pista | **todo lo demás**: esquinas, líneas del piso, vueltas, parada en meta |

Estados de la FSM (12): `espera · arranque · pista · senal · esquive ·
reincorporacion · esquina · correccion · reversa · meta · fin · fallo`

```
1907f06  La naranja no oscurece el piso, y esa era la puerta
fbe870d  El sensor de color preguntaba siempre un pelo antes de tiempo
d5c4861  La esquina la dice el piso, no la camara
1e55175  Un muro de frente no es un muro al costado
be28742  Sesenta y tres vueltas en treinta centimetros
c6bc1af  Mantener el rumbo no es mantener el volante
8341ffc  Un hueco tiene direccion, no posicion
f3a2c48  Que la altura mande sobre la base, y que el carro no se frene solo
70a604d  Sistema de control para el reto de obstaculos
```

---

## La arquitectura en un párrafo

La Pi 5 piensa, el ESP32 obedece y desconfía. La Pi corre visión, decisión y
una FSM a la velocidad de la cámara (~30 Hz); un hilo aparte reenvía el último
mando a **50 Hz fijos**, de modo que una pausa de visión nunca se confunde con
una Pi muerta. El ESP32 valida CRC, aplica rampas y topes, y corta la tracción
él solo si la Pi calla 300 ms o si su propia tarea de control calla 200 ms. El
motor necesita **dos llaves**: el botón físico (en el ESP32) y `F_ARMADO` (de
la Pi); ninguna basta por sí sola.

---

## Los nueve fallos, y qué enseñó cada uno

### 1. El carro ignoraba el color de los pilares

**Síntoma:** esquivaba a un lado cualquiera, sin mirar rojo ni verde.

**Diagnóstico:** no decidía mal — **no veía ningún pilar**. Había un filtro que
exigía que la distancia deducida de la base y la deducida de la altura
aparente coincidieran. Pero no dependen de lo mismo:

| Medida | Depende de | Con el mástil 4,5° torcido |
|---|---|---|
| por altura | solo `fy_px` | 600 / 900 / 1200 / 1500 mm — exacta |
| por base | `fy` + altura cámara + **inclinación** | 980 / 2100 / 4949 / 6000 mm |

El filtro las comparaba, no se parecían, y descartaba **todos** los pilares en
silencio. Reproducido con fotogramas sintéticos: con la inclinación real a 12°,
detección **0 a todas las distancias**.

**Lección:** el desacuerdo entre dos medidas no era señal de detección falsa,
era señal de calibración imperfecta — o sea, el estado normal el día de la
competencia. Un filtro que descarta ante la duda convierte un error de montaje
de 4° en ceguera total. Ahora la altura manda y la base solo pondera.

> **Regla que sacamos:** ningún filtro debe poder vaciar una detección
> completa sin dejar rastro. De ahí nació `Escena.descartes`, que cuenta qué
> filtro se comió cada contorno y viaja a la telemetría.

### 2. El carro se quedaba «trabado»

Dos causas independientes, las dos reales:

- Un pilar de frente a 1,5 m daba `holgura = 0`, caía bajo el umbral de «lado
  incorrecto» y metía el carro en `CORRECCION` **al 22 %** en cada aproximación
  normal. El lado solo debe juzgarse cuando de verdad se acaba el sitio
  (< 700 mm), no desde lejos.
- `pwmMinArranque` existía en el firmware pero **la trama `CONFIG` no lo
  enviaba nunca**, así que quedaba en 0. A bajo porcentaje el motor zumbaba sin
  moverse. Se extendió `CONFIG` a 7 bytes (compatible con la de 6).

**Lección:** un parámetro que existe en un lado del enlace y no viaja es peor
que no tenerlo, porque se lee el código y parece que está resuelto.

### 3. No seguía la dirección de la pista: chocaba con las paredes

**Dos fallos superpuestos en el mismo cálculo.**

Un sector **libre** no tiene muro, así que no tiene base sobre la que apoyarse.
El código le calculaba igualmente un «lateral en milímetros» usando la fila del
fondo de la imagen, y salía que el hueco estaba a 24 mm del parachoques. El
ángulo resultante era de ~1° y el volante pedía **±7 %** con la curva entera
abierta a un lado.

Y al arreglar eso apareció el segundo: en una recta, media imagen empata en la
distancia máxima, y `np.argmax` devuelve **el primero**, o sea el más a la
izquierda. El carro pedía volante a la izquierda en cada recta aunque fuera
perfectamente centrado.

| Escena | Antes | Después |
|---|---|---|
| curva, salida a la izquierda | −7,4 % | **−73,6 %** |
| curva, salida a la derecha | +6,4 % | **+73,6 %** |
| recta centrada | −50,9 % | **0,0 %** |

**Lección:** un hueco tiene **dirección**, no posición. Pedirle una posición en
el suelo obliga a inventarse una fila, y con la fila equivocada el ángulo sale
nulo. Ahora cada sector lleva su rumbo angular (solo depende de `fx`) y se
apunta al **centro del tramo contiguo libre más ancho**.

De paso se rehízo el centrado, que comparaba distancias *hacia delante* del
tercio izquierdo contra el derecho — eso responde a lo lejos que está el muro,
no a lo pegado que va el carro. Ahora cada sector con muro se convierte en un
punto del suelo y se mide la separación lateral de verdad.

**El sentido de las curvas se aprende.** En una pista WRO todas van hacia el
mismo lado, así que basta acertar una: se promedia el rumbo del hueco durante
la primera curva. El color de la primera línea pisada entra solo como pista de
arranque, y el sentido aprendido siempre manda sobre ella — `color_entrada_horario`
sigue siendo un parámetro adivinado y no puede decidir doce curvas.

### 4. Tras esquivar, seguía girando hacia el pilar

**Síntoma:** pasaba el verde y seguía curvando a la izquierda hasta el muro.

**Diagnóstico:** la fase de compromiso congelaba el **ángulo de volante** del
último fotograma con el pilar a la vista —unos 59 %— durante 0,78 s. Un volante
fijo no traza una recta: traza un arco. Con el modelo Ackermann simplificado,
**537 mm de desvío lateral**, más de medio carril.

El comentario del código ya decía «se mantiene el rumbo». La implementación
mantenía otra cosa.

**Solución:** se congela el **yaw del MPU** del instante en que el pilar entra
en zona ciega, y un proporcional devuelve el carro a ese rumbo. Sin MPU, el
respaldo suelta el volante progresivamente: peor, pero nunca cierra el arco.

Y como durante el compromiso el seguidor de carril está callado a propósito
(si opinara, la rueda trasera barrería el pilar), se añadió `guardia_muro`: no
centra, solo evita el golpe; se calla mientras haya sitio y su autoridad crece
con el progreso del adelantamiento.

**Lección:** mantener el rumbo no es mantener el volante. Cuando un comentario
y su código discrepan, el que está mal suele ser el código — pero hay que
demostrarlo, no suponerlo.

### 5. «Ronda terminada» nada más pulsar el botón

**Síntoma:** `'vueltas': 63, 'cruces': 255, 'dist_m': 0.3`.

255 es la saturación de un contador de 8 bits, y 63 vueltas en 0,3 m es
imposible. Era una **carrera de sincronización**: `preparar()` mandaba
`CAL_CERO_LINEAS`, pero la Pi tomaba su línea base antes de que el ESP32
llegara a aplicarlo. El contador pasaba de 200 a 0 y `(0−200) & 0xFF = 56`
cruces fantasma → 14 vueltas de golpe.

Tres redes, porque una sola no basta:

1. **Salto imposible = resincronizar.** Entre dos ciclos de la Pi no caben más
   de 2-3 cruces reales. Por encima se re-referencia y no se cuenta nada. Un
   envoltorio legítimo (254 → 1) sí sigue contando.
2. **No contar en `ESPERA`.** El carro pasa minutos quieto delante del juez.
3. **Suelo de distancia.** Tres vueltas no caben en tres metros; una vuelta
   real ronda los 8 m.

Y los contadores se ponen a cero **en el flanco del botón**, no al lanzar el
programa: así el cero de los dos lados cae en el mismo instante que el
cronómetro del juez, y la carrera desaparece por construcción.

**Lección:** un contador acumulado de 8 bits es robusto frente a tramas
perdidas —esa era la idea— pero no frente a un **reinicio**. La resta modular
convierte un reset en un salto enorme indistinguible de actividad real. Hay que
decidir explícitamente qué delta es físicamente posible.

### 6. Un muro de frente se contaba como muro al costado

**Síntoma:** el mismo de antes — rebasaba bien y acto seguido se clavaba en la
pared — pero seguía pasando después del arreglo 4.

**Diagnóstico:** la separación lateral metía el muro **frontal** en la cuenta.
Un muro de frente ocupa *todos* los rumbos, y el sector a 4° del morro da
`700 · sen 4° = 49 mm`. Con el carro centrado en un carril de 1000 mm y una
esquina a 700, el código reportaba **muros a 46 mm a los dos lados**. Como la
guardia empuja con la *diferencia*, los dos empujones se cancelaban: daba
exactamente 0 justo cuando hacía falta.

Ahora solo cuenta como costado lo que está más cerca que el frente
(`frac_frente_lateral`). Medido: a 150 mm del muro derecho tras rebasar por la
derecha, el volante pasa de **−12 % a −66 %**.

Dos causas encadenadas más en el mismo commit: el compromiso soltaba el carro
con el rumbo apuntando al lado del rebase (ahora la segunda mitad lo lleva al
rumbo del pasillo libre), y al volver a `PISTA` ese frente cerrado se leía como
esquina — de ahí el estado **`REINCORPORACION`**, que dice explícitamente
«todavía me estoy recolocando».

**Lección:** una magnitud derivada puede ser correcta en su fórmula y aun así
estar midiendo otra cosa. «Distancia lateral» y «distancia a lo que tengo
delante proyectada al lateral» son el mismo número y conceptos distintos.

### 7. Las esquinas eran cosa de la cámara, y la cámara no sabe

**Síntoma:** giraba antes de tiempo y giraba a medias.

**Diagnóstico:** la cámara solo sabe decir «el frente se cerró», y eso le pasa
en una esquina igual que cuando el carro quedó apuntando a un muro tras
rebasar. Además el seguidor de carril renegocia el volante cada ciclo contra el
centrado, así que en cuanto asoma hueco, afloja.

Ahora hay un reparto explícito: **la línea del piso dice CUÁNDO**, la cámara
confirma **QUE ESTÁ AHÍ** (el frente tiene que estar cerrado) y el sentido de la
ronda dice **HACIA DÓNDE**. El estado `ESQUINA` hace un giro comprometido —
`dir_esquina` sostenido hasta `esquina_grados` de yaw, sin renegociar — y entre
esquina y esquina el carro va recto.

Cuatro redes: una esquina = un giro (`dist_entre_esquinas_mm`, porque las dos
líneas caen en los mismos 1000 mm y 90+90 es la pared); `esquina_frente_max_mm`
descarta la línea de *salida* si el TCS se perdió la de entrada; un pilar
interrumpe la curva (8-10 puntos pesan más que trazar bonito); y sin MPU se sale
por tiempo.

**Bonus, encontrado al montar el guion de integración:** `piloto.ciclo` calcula
el carril **antes** que el contador, así que en el ciclo del primer cruce
`sentido_curva` todavía valía 0. Como el evento de esquina dura un solo ciclo,
**la primera curva de la ronda no disparaba nunca**. Un fallo de orden que
ninguna prueba de módulo aislado podía ver.

### 8. El sensor de color preguntaba siempre un pelo antes de tiempo

**Síntoma:** no detectaba bien las líneas. No era el sensor ni los umbrales:
era aritmética.

El chip integra cada `periodoMs` y el firmware preguntaba cada `periodoMs`. Dos
relojes libres a la misma frecuencia **derivan**: la pregunta cae siempre justo
antes de que el dato exista y, como el hueco siguiente ya estaba reservado, el
periodo real **se dobla**. Una línea de 20 mm a 1 m/s dura 20 ms; en un hueco de
48 ms cabe entera sin dejar rastro.

Cuatro arreglos, y los tres últimos salieron al medir el primero: sondear cada
2 ms y coger la muestra cuando `AVALID` lo dice (`atime` 24 → 12 ms);
`leerColor` devolvía `true` aunque siguiera integrando, así que el clasificador
se comía la muestra anterior otra vez; contar la permanencia en milisegundos en
vez de exigir dos lecturas seguidas; y adoptar entera la primera lectura de
blanco, que antes subía desde 1 a razón de 1/64 por muestra y dejaba el carro
**ciego diez segundos al encender**.

**Lección:** muestrear a la misma frecuencia a la que se produce el dato es la
receta del *aliasing*. Hay que sondear más rápido y dejar que el productor diga
cuándo.

### 9. La naranja no oscurece el piso, y esa era la puerta

**Síntoma:** no veía la naranja, y la azul apenas.

**Diagnóstico:** el clasificador exigía que el piso se **oscureciera** por
debajo del 78 % del blanco para admitir que había línea. Pero la naranja es
CMYK(0,60,100,0): refleja rojo y verde casi como el tapete, así que su claro
ronda el umbral y entra o no según la luz de la sala. La azul, oscura, pasaba
siempre. El síntoma era exactamente ese.

**El oscurecimiento nunca fue la prueba de que hay línea.** La prueba es el
color: el tapete da `r−b` casi cero haga la luz que haga. Ahora decide la
magnitud del discriminante con histéresis propia (entra a 45, sale a 25), y del
claro queda solo un suelo del 12 % para descartar el muro negro, donde las
cuentas son ruido.

En el mismo commit, **`activar_desde_mm` no mandaba el alcance: lo mandaba
`area_min`**. Un pilar de 50×100 ocupa del orden de `8,5·10⁸/d²` px — 330 a
1,6 m y 147 a 2,4 m — así que con `area_min = 320` el carro era ciego más allá
de 1,6 m aunque el JSON dijera 2400. Medido con pilares sintéticos: con 320
detecta a 1600 y no a 2000; con 140 llega a 2800.

**Lección:** un parámetro que *parece* gobernar algo puede estar dominado por
otro tres capas más abajo. `activar_desde_mm` era un número decorativo.

> **Herramienta que salió de aquí:** `main.py --lineas`, monitor del sensor de
> color sin cámara ni motor. Se empuja el carro a mano sobre cada línea y dice
> el `|sep|` máximo por color y cuántos cruces contó. Convierte «no las ve» en
> tres diagnósticos con arreglos distintos: el color no llega (ningún umbral
> arregla una señal que no está), el color llega pero el cruce fue demasiado
> rápido, o firmware viejo.

### Y una que no era del código

Un `Device '/dev/video0' is busy` por una ejecución anterior que quedó viva. No
era un fallo nuestro, pero el mensaje `no se pudo abrir la camara` a secas
obligaba a adivinar. Ahora hay `--listar-camaras` (lee `/proc`, no necesita
`fuser` ni `lsof`), `--camara N`, autobúsqueda de índice y un mensaje que
distingue «ocupada por X» de «no hay ningún `/dev/video*`» de «abre pero no da
imagen» (ese último es el nodo de metadatos de la UVC).

---

## Patrón que se repite

Ocho de los nueve tenían la misma forma: **una magnitud calculada con la
fórmula equivocada, que daba un número plausible en vez de un error**. Un pilar
a 4.949 mm, un hueco a 24 mm, un volante congelado al 59 %, un delta de 56
cruces, dos muros a 46 mm a los dos lados, un periodo de muestreo que valía el
doble de lo que decía. Ninguno lanzó una excepción. Ninguno se vio en una
prueba unitaria de la función aislada. Todos se vieron reproduciéndolos con
datos sintéticos y **mirando el número intermedio**.

Dos variantes que conviene reconocer aparte, porque no se cazan mirando una
función:

- **El parámetro decorativo.** `activar_desde_mm` decía 2400 y el alcance real
  lo fijaba `area_min` tres capas más abajo. `pwmMinArranque` existía en el
  firmware y la trama nunca lo enviaba. El código se lee y parece resuelto.
- **El fallo de orden.** El carril se calculaba antes que el contador, así que
  en el ciclo del primer cruce el sentido aún valía 0 y la primera curva no
  disparaba nunca. Cada módulo estaba bien; el guion estaba mal.

Por eso el `selftest.py` de este proyecto no prueba «que la función devuelva
algo»: prueba **que el carro gire hacia el lado correcto** en escenas
fabricadas, incluida la cámara mal calibrada a 10°, 12° y 14°.

---

## Calibración, en orden. Saltárselo es perder una tarde

1. **Servo** — `centro` deja las ruedas rectas; `izquierda`/`derecha` sin topar
   la cremallera. Los topes duros están en `seguridad.h` como constantes de
   compilación: el JSON solo puede estrechar.
2. **`fy_px`** — es el que de verdad importa. La distancia a un pilar sale de
   su altura aparente. `inclinacion_deg` y `alto_cam_mm` son secundarios **a
   propósito** desde el fallo 1.
3. **Colores HSV** — con `--ver`, sobre el tapete y la luz reales. Lo que
   importa no es que el pilar se vea entero: es que **nada más** se vea.
4. **`pwm_min_motor`** — de 5 en 5 desde 55, hasta que arranque limpio desde
   parado y ni uno más.
5. **`mm_s_por_pct`** — 40 % durante 3 s en recta, medir, dividir.
6. **`color_entrada_horario`** — empujar el carro a mano en sentido horario y
   mirar qué color reporta el primer cruce. **Sigue sin verificarse.**

---

## Lo que queda pendiente

- **⚠ EL RANGO DEL SERVO ESTÁ SIN COMPROBAR CONTRA LA MECÁNICA.** Pasó de
  65/135 a **58/142** para poder cerrar más las curvas. Los topes duros de
  `seguridad.h` son 50/145, así que el software lo admite — pero eso no dice
  nada de la cremallera. **Antes de rodar: carro levantado, mandar tope a tope
  y comprobar que no hace tope mecánico.** Si lo hace, el servo se queda
  empujando contra el final de carrera y se rompe. Es lo único pendiente que
  puede romper hardware.
- **Los umbrales de color son una estimación, no una medida.** Entra a 45, sale
  a 25. Usar `main.py --lineas` sobre el tapete real **antes** de tocarlos.
- **El firmware compila (`arduino-cli`) pero no se ha ejecutado.** Necesita
  **core ESP32 3.x** (usa `ledcAttach`) y la carpeta debe llamarse igual que el
  `.ino`. Y hay que **reflashear**: la trama de sensores creció a 16 bytes.
  La Pi tolera firmware viejo a propósito (lee los 4 bytes extra solo si están),
  pero entonces no hay diagnóstico de líneas.
- **Todo lo que no sea detectar y esquivar pilares está sin probar en pista.**
  Esquinas, líneas del piso, vueltas y parada en meta solo tienen evidencia
  sintética. Es lo siguiente que hay que rodar.
- **`color_entrada_horario` sin verificar.** Mitigado dos veces (el sentido de
  las curvas se aprende, y ahora lo confirma el par ordenado de la esquina),
  pero conviene fijarlo.
- **`margen_verde_mm` (100) ≠ `margen_rojo_mm` (70) es una compensación, no un
  arreglo.** El punto de paso es simétrico en el código; si en pista pasa más
  cerca del verde, lo más probable es que la máscara del verde recorte el pilar
  y la distancia salga mayor. Merece medirse en vez de compensarse.
- **Un pilar cada vez.** Con dos seguidos se resuelve el primero y el segundo
  se replantea después.
- **Sin encoder.** El odómetro se integra del porcentaje de mando (10-15 % de
  error). Por eso el detector de atasco mira la imagen y no la velocidad
  estimada: preguntarle al acelerador si el coche se mueve no sirve cuando
  está clavado contra un muro.
- **Nada de estacionamiento**, por decisión de alcance. Los delimitadores
  magenta sí se detectan, pero como muro intocable: tocarlos termina la ronda
  (9.25.7).

---

## Qué mirar cuando algo falle

| Síntoma | Primera línea de la telemetría a mirar |
|---|---|
| ignora los pilares | `pilares vistos` y `descartes` |
| no gira en las curvas | `hueco a N deg` y `curvas hacia ...` |
| se va contra el muro tras esquivar | `comp_err_rumbo` y `guardia` |
| termina la ronda antes de tiempo | `vueltas` contra `dist_m`, y `resync` |
| no ve las líneas del piso | `python3 main.py --lineas`, empujando a mano |
| gira antes de tiempo o a medias | si entra en `ESQUINA` por línea o por cámara |
| hace una ese al salir del rebase | `REINCORPORACION`: sale por enfilado, no por centrado |
| no abre la cámara | `python3 main.py --listar-camaras` |

El README tiene una tabla de triaje por cada uno de estos casos.
