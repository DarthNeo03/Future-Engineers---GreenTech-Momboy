# Piloto WRO 2026 — RETO DE OBSTACULOS

Este programa es una COPIA de `main/open-challenge/` con dos cosas mas:

1. **Esquivar pilares**: el ROJO se pasa por SU DERECHA y el VERDE por SU
   IZQUIERDA. Nada mas: no hay modos de esquive alternativos ni inversion por
   sentido. Desde 2026-09-11 es un **servo visual en pixeles** tomado del
   piloto de ANTi (WRO 2025): el pilar se empuja hacia el canto de la imagen
   por el que tiene que salir. Ver "Los pilares".
2. **Rescate de esquina**: cuando el piso queda en TRIANGULO -el carro metido
   en el rincon, con las dos paredes cerrando por los dos lados- se da un giro
   comprometido hacia adentro. Ahi la reversa del escape no saca al carro.

Todo lo demas (navegacion centrada, conteo de esquinas por las lineas del piso,
sentido deducido solo, parada en meta, calibracion, web) es identico al Open
Challenge y esta documentado igual mas abajo, salvo tres retoques de la
navegacion que exige el pilar (ver "Lo que la navegacion hace distinto con un
pilar en juego"). Si arreglas un fallo en uno,
miralo tambien en el otro: son dos copias a proposito, para que un experimento
de obstaculos no pueda romper una ronda limpia de Open Challenge la vispera de
una competencia.

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
python tools/selftest.py            # 103 pruebas, sin hardware
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

## Que hace en la ronda (reglas 2026)

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

---

## Los pilares

    pilar ROJO   -> se pasa por SU DERECHA   -> se le empuja al canto IZQUIERDO de la imagen
    pilar VERDE  -> se pasa por SU IZQUIERDA -> se le empuja al canto DERECHO

Y esos lados **no se invierten con el sentido de la ronda**. El reglamento habla
de la derecha y la izquierda DEL VEHICULO ("el lado del carril por el que debe
circular"), y como la camara mira hacia adelante, trabajar en pixeles de la
imagen ES trabajar en el marco del carro: sale solo en los dos sentidos. No hay
interruptor para invertirlo: pasar por el lado incorrecto termina la ronda
(9.25.5).

### Como se esquiva (servo visual, tomado del piloto de ANTi)

El esquive se reescribio (2026-09-11) tomando como referencia el piloto de
[ANTi, WRO 2025](https://github.com/atakanersoy/WRO2025_FE_ANTi) (35 s de reto
de obstaculos con puntuacion completa). Su receta es corta: el giroscopio
mantiene el rumbo de la recta y, cuando la camara ve un pilar, se le EMPUJA
hacia el canto de la imagen por el que tiene que salir, con una direccion
proporcional a cuantos pixeles le faltan. Ni puntos de paso en milimetros, ni
recorte contra el hueco del muro, ni pesos que compiten con el muro: eso era lo
que generaba los tres fallos de pista del esquive anterior (volantazo, cola que
barre el pilar, pilar que manda sobre el muro).

Tres estados (`src/obstaculos.py`; se ven en la web y sobre el video):

1. **LIBRE.** Sin pilar: centrado por el muro + rumbo por giroscopio, igual que
   en el Open Challenge.
2. **SIGUIENDO.** Se ve un pilar a menos de `activar_desde_mm`. Se mide cuanto
   le falta al **borde interior** del pilar (el que da al centro de la imagen)
   para llegar a la columna objetivo, pegada al canto (`borde_frac`), y se pide
   direccion proporcional (`k_borde`) **hacia el lado de paso**. Es
   asimetrico: el pilar solo puede pedir alejarse de el; si ya esta mas afuera
   que la columna objetivo no pide nada, y volver a la recta es trabajo del
   rumbo (un servo simetrico lo "mantendria" en el canto girando hacia el, que
   es un rumbo de colision con angulo fijo). El peso del pilar sube de
   `peso_lejos` a 1 entre `activar_desde_mm` y `mandar_desde_mm`: con peso 1 el
   centrado calla y del rumbo solo sobrevive `yaw_al_esquivar` (ANTi conserva
   mas o menos un tercio). Con el pilar a menos de `empuje_bajo_mm` y todavia en
   cuadro se suma `empuje_pct`: el canto de la imagen no da holgura para un
   carro de 20 cm. Se va a `obstaculos.vel_pct`.
3. **ADELANTANDO.** El pilar se perdio de vista a menos de `compromiso_bajo_mm`
   (salio por el canto o por abajo: la camara no llega tan abajo). Se
   **congela el rumbo** en el que quedo el carro, sin centrado, el tiempo que
   tarda el carro entero en pasarlo (distancia + `geometria.largo_carro_mm`, a
   la velocidad real, contando `limites.vmax`). Es lo que impide que la cola
   barra el pilar: con Ackermann la rueda trasera corta por dentro si se vuelve
   hacia la recta antes de tiempo. Se suelta antes si aparece otro pilar a
   menos de `mandar_desde_mm` (lo visible manda) o si el pasillo baja de
   `soltar_pasillo_mm` (el muro manda). Perdido mas lejos no hay compromiso:
   salio por el canto porque el carro ya giro de sobra, y el rumbo lo vuelve a
   traer a la vista.

Si hay dos pilares manda el MAS CERCANO, con `preferir_mm` de ventaja para el
que ya se venia siguiendo (no cambiar de pilar por ruido).

**Los pilares ya no son muro.** `muro._mascara_piso` cuenta el rojo y el verde
como piso a proposito: de ellos se ocupa el esquivador. Si entraran en el
perfil como muro, un pilar a 60 cm de frente cerraria el pasillo y dispararia
lo que solo debe disparar un muro: frenada, giro de esquina de 90 (y su conteo)
o escape en reversa. Con el pilar como piso, el contacto de esa columna es el
muro que hay detras, que es lo que de verdad cierra el paso. La vista "piso" de
la web lo enseña (los pilares salen claros, como el piso).

**Como se comprueba sin arriesgar una ronda**: con el carro PARADO delante de un
pilar, el video dibuja la columna objetivo pegada al canto y una flecha desde el
borde del pilar hasta ella, y escribe por ejemplo "rojo a 355mm -> paso por su
derecha: falta 46% dir +55". La flecha del rojo apunta a la IZQUIERDA (el carro
va a la derecha) y la del verde a la DERECHA. Mirarlo antes de cada sesion.

### Lo que la navegacion hace distinto con un pilar en juego

- **El giro de esquina cede ante el pilar y se reanuda.**
  `navegacion.giro_cede_ante_pilar` (encendido). Un giro comprometido de 90
  grados con un pilar delante lo atropella o lo pasa por el lado prohibido, y
  eso termina la ronda. Al soltar, el rumbo objetivo ya apunta a la recta
  NUEVA, asi que mientras esquiva el giroscopio sigue tirando hacia adentro; el
  giro se reanuda cuando el pilar deja de mandar, **sin volver a sumar 90** al
  rumbo (el programa anterior sumaba 90 en cada reintento y acababa apuntando a
  la pared) y sin contar la esquina (para entonces ya la contaron las lineas;
  mejor perder una que sumar una).
- **Torcido no hay esquina por pasillo.** Esquivando, el carro va cruzado
  respecto a la recta; un pasillo que se cierra o un muro lateral que
  desaparece son entonces el muro de la propia recta visto de lado, no una
  esquina, y girar 90 ahi es meterse contra el. Con mas de
  `navegacion.esquina_max_desvio_deg` de desvio esos dos disparadores se
  callan; la linea del piso sigue disparando siempre.
- **Tras un escape (o un giro vencido) el rumbo se ancla a una recta valida**
  (la de antes o sus vecinas de +-90, la mas cercana al yaw real), nunca al
  rumbo en que quedo mirando: adoptar ese rumbo fue lo que hizo que el carro
  se fuera en sentido contrario tras varios escapes.

### Lo que se dejo fuera a proposito

- La busqueda del pilar perdido de lejos (ANTi: `lost_color`): el rumbo ya lo
  vuelve a traer a la vista.
- El filtro de pilares por detras de la linea del piso (`limitar_por_lineas`
  del programa combinado). Si en pista el carro se pega a la esquina interna
  por hacer caso a un pilar del tramo siguiente, ESO es lo que hay que volver a
  poner.
- El estacionamiento: este programa no aparca.

**Sin probar en pista todavia**: el servo esta comprobado con imagenes
sinteticas y 103 pruebas sin hardware. Lo primero que hay que mirar en pista es
`borde_frac` (holgura al pasar) y `k_borde` (que el pilar llegue al canto antes
de estar encima); despues `compromiso_bajo_mm` si la cola roza.

---

## El rescate de esquina (el triangulo)

El fallo por el que existe este apartado: el carro entra en la curva y no sale.
El negro cubre casi todo el cuadro y el piso que queda es un **triangulo con el
vertice arriba**, las dos paredes del rincon cerrando por los dos lados. Ni el
escape ni el anti-bucle valen ahi: el escape retrocede, abre un palmo de pasillo
y devuelve el mando al centrado, que se vuelve a meter en el mismo hueco.

Se detecta sobre el perfil que ya se calcula, sin vision nueva
(`muro.DetectorAtrapado`): muro en casi todas las columnas, nada lejos ni en el
percentil 80, **relieve** en el perfil (una pared plana de frente da 0 % y es
otro problema, el que resuelve el escape), poco piso, y todo eso sostenido
`rescate.confirmar_ms` sin que el vertice se aleje. Entonces: reversa corta con
el volante invertido -con el morro pegado, girar no mueve el carro, lo raspa- y
un giro comprometido de `rescate.grados` (100 por defecto) hacia adentro.

**El lado**, por orden: el del PILAR si hay uno en juego; si no, el sentido de la
ronda; y si tampoco se conoce, hacia donde se ve **mas piso**.

Dos medidas que conviene no perder:

- El relieve va en **porcentaje, no en milimetros**: la misma esquina de 90
  grados da 27 % desde 700 mm y 31 % desde 400, pero en mm baja de 147 a 89, o
  sea que un umbral en mm fallaria justo cuando mas atrapado esta el carro.
- Al terminar se vuelve a una recta VALIDA (`_anclar_recta_mas_cercana`), no al
  rumbo en el que se quedo mirando. Adoptar el rumbo de salida fue exactamente
  lo que hacia que el carro se fuera en sentido contrario tras varios escapes.

El rescate **no cuenta esquina**: el carro atrapado casi siempre ya cruzo esa
linea, y sumar una vuelta por cada choque fue un fallo real.

Si el rescate salta en curvas buenas, **sube `confirmar_ms`** antes de tocar
nada mas. Hay boton "Forzar rescate de esquina" en la web para verlo en el
banco, y los cuatro numeros del detector se pintan sobre el video aunque el
carro este desarmado, para poder ajustar los umbrales con el carro puesto a mano
en el rincon.

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
| `obstaculos.borde_frac` | Columna objetivo pegada al canto. Mas pequeño = mas holgura al pasar el pilar, pero sale antes de cuadro. |
| `obstaculos.k_borde` | Ganancia del servo visual. Sube si el pilar no llega al canto antes de estar encima; baja si serpentea. |
| `obstaculos.empuje_pct` / `empuje_bajo_mm` | El empujon extra con el pilar encima. Si la rueda delantera roza el pilar, sube esto. |
| `obstaculos.compromiso_bajo_mm` | Desde que cercania un pilar perdido congela el rumbo. Si la cola barre el pilar, sube esto o `geometria.largo_carro_mm`. |
| `obstaculos.yaw_al_esquivar` | Cuanto rumbo sobrevive mientras el pilar manda. Sube si el carro se cruza demasiado; baja si vuelve hacia el pilar. |
| `navegacion.esquina_max_desvio_deg` | Desvio a partir del cual un pasillo cerrado ya no cuenta como esquina (es el muro de la recta visto de lado). |
| `rescate.confirmar_ms` | **El** parametro del rescate. Si salta en curvas buenas, subelo antes que nada. |
| `rescate.grados` | Cuanto gira el rescate. Un poco mas de 90 para salir apuntando a la recta, sin pasarse. |

## Estructura

```
obstacle-challenge/
├── main.py                 arranque; --simulado, --imagen, --vmax, --puerto
├── piloto.sh               demonio de competencia (sin web)
├── config/                 params.json y colors.json (se crean solos; 20 perfiles c/u)
├── src/
│   ├── geometria.py        pixeles <-> mm sobre el suelo; horizonte; corredor
│   ├── muro.py             perfil de contacto + rectas + esquinas + DetectorAtrapado
│   ├── vision.py           mascaras HSV y deteccion de objetos (del reconizer)
│   ├── color_config.py     perfiles de color (mismo formato que el reconizer)
│   ├── params.py           esquema autodocumentado de parametros + perfiles
│   ├── lineas.py           sentido / esquinas / vueltas (TCS + camara + giros)
│   ├── navegacion.py       RECTO / PRE_GIRO / GIRO / ESCAPE / RESCATE
│   ├── carrera.py          director de la ronda (3 vueltas y parada en meta)
│   ├── obstaculos.py       el esquive: servo visual (rojo al canto izquierdo, verde al derecho) + compromiso
│   ├── botones.py          el pulsador de competencia (cuelga del ESP32)
│   ├── protocolo.py        trama binaria v3 (gemela de protocolo.h)
│   ├── enlace.py           hilo serie; sensores del ESP32 -> eventos
│   ├── robot.py            el nucleo que une todo
│   ├── dibujo.py           overlay del video
│   ├── servidor.py         http.server + MJPEG
│   └── web/index.html      la interfaz
└── tools/selftest.py       103 pruebas sin hardware
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
