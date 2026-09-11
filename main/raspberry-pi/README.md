# Piloto WRO 2026 — Open Challenge + Reto con Obstaculos

Sistema nuevo del carro (GreenTech Momboy). Sustituye a `reconizer` tomando lo
que funcionaba (navegacion centrada, calibracion por clic, enlace binario con
el ESP32) y arreglando lo que fallaba (paredes con brillo, objetos detectados
por encima del muro, conteo de esquinas).

```
Raspberry Pi 5 (vision + decisiones)  <-USB->  ESP32 (motor, servo, MPU6050, TCS34725)
```

El firmware del ESP32 vive en `code/esp32_carro/` (protocolo v2: lee el
MPU6050 y el TCS34725 por I2C y manda yaw, cruces de linea y el estado de los
el estado del pulsador de competencia a la Pi).

---

## Puesta en marcha

**Raspberry Pi 5** (o PC Windows para probar sin carro):

```bash
python3 -m venv .venv && source .venv/bin/activate    # en Windows: .venv\Scripts\activate
pip install -r requirements.txt
python tools/selftest.py            # 409 pruebas, sin hardware
python tools/selftest_esquive.py    # el carro PASA el pilar (simulacion cinematica)
python main.py                      # camara + ESP32 + web (Open Challenge)
python main.py --reto obstaculos    # Reto con Obstaculos (config/obstaculos/)
python main.py --simulado           # sin ESP32 (pruebas en el PC)
python main.py --imagen foto.jpg    # sin camara, sobre una foto
python main.py --vmax 90            # tope de PWM solo para esta prueba
./piloto.sh arrancar                # COMPETENCIA: demonio sin web (SSH/VNC)
```

**ESP32**: abrir `code/esp32_carro/esp32_carro.ino` en el IDE de Arduino (los
6 archivos en la misma carpeta) y subir. Sin librerias externas. Al subir
firmware nuevo, subirlo ANTES de correr `main.py`. **El pulsador necesita
la version 4** (la trama de sensores paso de 14 a 15 bytes); con firmware
viejo todo lo demas sigue igual, pero el boton no llega.

Web de depuracion: **http://carrito.local:8080/** (o `http://<ip>:8080/`).
El carro **arranca desarmado**: pulsar ARMAR. En modo auto, ARMAR arranca la
carrera (cronometro + conteo).

En competencia no hay web: manda **el pulsador** que cuelga del ESP32 (la
misma pulsacion arma y desarma) y el **LED azul** dice en que estado esta el
carro; el programa se lanza como demonio con `./piloto.sh arrancar`. Ver
[El boton y el LED](#el-boton-y-el-led-el-start-del-reglamento) y
[Correr sin la web](#correr-sin-la-web-el-demonio-ssh-y-vnc).

---

## Que hace en el Open Challenge (reglas 2026)

1. **Sentido de la ronda**: en 2026 las 3 vueltas van TODAS en la misma
   direccion (ya no hay media vuelta) y esta prohibido meterle datos al robot
   antes del start, asi que el carro lo deduce solo:
   - por la **primera linea** de esquina que cruza (TCS bajo el carro o
     camara): naranja primero = horario, azul primero = antihorario
     (`lineas.naranja_es_horario` por si un tapete viniera al reves);
   - por la camara ANTES de cruzar (ve cual linea esta mas cerca);
   - de respaldo, por el lado del primer giro.
   Tambien se puede forzar desde la web para pruebas.
2. **Mantenerse en el carril**: estrategia `centrado` (la probada) o `pared`
   (seguir el muro interno a distancia fija). El giroscopio corrige el rumbo
   en recta y clava los giros de 90.
3. **Esquinas**: PRE_GIRO (frena + deja pasar las ruedas traseras + **giro
   abierto** tipo camion si hay sitio) -> GIRO de 90 por giroscopio -> RECTO.
   Disparadores: **linea del piso cruzada** (el mas fiable), pasillo
   cerrandose, o muro interno que desaparece. Ver "El bucle de las esquinas".
4. **Conteo**: una esquina se ABRE con su primera linea y se queda esperando
   a la otra, que la CIERRA (nunca abre otra: ver "El giro fantasma"). Cuenta
   una sola vez, la den por buena las lineas o el giro de 90. Si se ve el par
   entero se comprueba ademas el ORDEN, que en las cuatro esquinas es el mismo
   (horario: naranja y luego azul): un par al reves o es basura o el carro se
   dio la vuelta, asi que uno suelto no cuenta y dos seguidos invierten el
   sentido. 4 esquinas = vuelta. Alternativa: **modo de esquina por color**
   (`esquina_color.activo`): solo cuenta el color del sentido, el otro se
   ignora, y el giro se suelta al ver un pilar. Ver su seccion mas abajo.
5. **Parada final**: tras la esquina 12 avanza `carrera.parada_ms` para meter
   el carro ENTERO en la seccion de meta y se detiene solo (bono del
   reglamento). El tope de 3 minutos (`carrera.tiempo_max_s`) es del
   reglamento y va por delante de todo: `carrera.autostop` apaga la parada por
   vueltas para pruebas de resistencia, **nunca el cronometro**.

**Obstaculos**: `python3 main.py --reto obstaculos` (o `obstaculos.activo` en
la web). Ver *Identificar el pilar ANTES de decidir el lado*.
Pilar rojo se pasa por la derecha, verde por la izquierda; el punto de paso se
calcula en mm reales y se recorta al hueco libre del perfil.

---

## Horario va y antihorario no: como se busca eso

La pista es simetrica. Si los MISMOS parametros funcionan en un sentido y no en
el otro, hay un sesgo en alguna parte, y ninguna calibracion lo arregla porque
lo que ayuda en un sentido estorba en el otro. La forma de encontrarlo no es
mirar codigo: es la **prueba de espejo**. Se coge una escena, se refleja, se
invierte el sentido, y la salida tiene que ser identica con izquierda y derecha
cambiadas. Lo que no cuadre, es el sesgo.

**Lo que aparecio asi: una pared lateral FANTASMA en la esquina.** El perfil de
distancias viene suavizado (`muro.suavizado`, 7 columnas), asi que el canto del
muro interno al abrirse la curva no es un escalon sino una **rampa** de media
docena de columnas con valores intermedios que no existen en la pista. La
cadena de puntos se cortaba en UNA sola muestra, la rampa se quedaba dentro, y
el ajuste de rectas la convertia en un segmento de 2,4 m casi perpendicular al
carro: una pared lateral a 20-40 cm en plena esquina, donde no hay nada.

Y salia **en un solo sentido**. El indice del borde lo da `np.diff`, que apunta
a la columna izquierda del par, y las muestras van de 4 en 4: de que lado de la
rampa cae el corte depende de donde toque la rejilla, y eso no es simetrico al
reflejar. Medido sobre 198 escenas espejo: `interna_mm` salia distinto en
**112**, siempre inventandose la pared en antihorario.

Arreglo: el corte se lleva toda la anchura del suavizado a cada lado del borde,
y ademas se descarta cualquier tramo que salte mas que `salto_borde_mm` entre
dos muestras vecinas — un muro no puede dar un salto de dos metros de una
columna a la siguiente; eso es un canto, no una pared. Las 198 escenas quedan
simetricas.

**Honestidad sobre el alcance:** ese fallo ensucia `interna_mm`, que es lo que
usa `estrategia: pared` y lo que se ve en la web. Con `estrategia: centrado`
—que es la que lleva el perfil que funciona— **no llega al volante**, asi que
por si solo NO explica que antihorario falle. Medido tambien: con esos
parametros la cadena entera (perfil + lineas + navegador) ya era simetrica,
0 de 45 ticks distintos, y los dos sentidos clavan los 90 grados. Es decir:
**el sesgo no esta en el codigo de navegacion.** Los tres sitios donde puede
estar, en orden:

1. **`carrera.sentido`.** Si el perfil lo trae forzado a `horario` y se corre
   en antihorario, no hay nada que discutir: se cuenta el color equivocado y se
   dobla al lado equivocado. En competencia tiene que estar en `auto`, porque
   el reglamento prohibe decirle el sentido al robot antes de la salida.
2. **El TCS perdiendo la linea de ENTRADA, que es distinta en cada sentido.**
   En horario se cruza primero la naranja; en antihorario, la azul. Y no cuestan
   lo mismo: la naranja saca ~105 puntos de diferencia r-b y la azul solo 37, y
   con la ventana de integracion a medias sobre la linea el piso blanco (cinco
   veces mas luz) aplasta esa diferencia hasta 14. A velocidad de crucero una
   linea de 2 cm en diagonal deja **1,9 ventanas** de 24 ms: con
   `muestras_min` en 2, la naranja aguanta y la azul no. Es un fallo que **solo
   se nota en antihorario, por construccion**. Ojo especialmente con
   `lineas.frenar_tras_ms`: si vale 0, el freno se suelta en cuanto la linea
   sale del cuadro, que es justo el instante en que va a pasar bajo el sensor —
   el carro la cruza a crucero (1,9 ventanas) en vez de frenado (3,1).
3. **Que la camara no este clavada en el eje del carro** (o que el mastil tenga
   un par de grados de guiñada). Ver abajo.

## El eje del carro no tiene por que ser el centro de la imagen

Todo el perfil compara izquierda contra derecha, y daba por hecho que el eje
del carro cae exactamente en la columna central de la imagen. Un par de
milimetros de desplazamiento de la camara, o un par de grados de guiñada del
mastil, meten un sesgo **constante** en esa comparacion — y no es neutral:
empuja al carro hacia el muro externo en un sentido de la ronda (donde sobra
sitio y el control lo corrige sin que se note) y contra el interno en el otro.

`geometria.centro_lateral_px` lo corrige: mueve el eje del carro dentro de la
imagen, y con el se mueven el corredor de las ruedas, las distancias laterales
y las dos bandas que se comparan. Vale **0 por defecto**, asi que sin tocarlo
no cambia nada de lo ya calibrado.

Calibrarlo son dos minutos: carro en el centro de una recta y bien encarado,
mirar `izq` y `der` en la web, y mover el numero hasta que marquen lo mismo.

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

El metodo viejo sigue disponible (`muro.metodo = negro`) para comparar en
pista. `tools/selftest.py` incluye un caso sintetico donde el viejo falla y
el nuevo no.

---

## El bucle de las esquinas (y por que las lineas lo arreglan)

Sintoma: al llegar a la curva el carro se queda dando vueltas dentro de ella,
como si el hueco fuera el camino.

Causa: cuando el muro interno se acaba deja un hueco de piso blanco enorme.
Para cualquier navegacion por espacio libre ese hueco **es** el camino: el
carro se mete, desde la posicion nueva vuelve a ver otro hueco, se vuelve a
meter, y nunca sale. No es un problema de umbrales; la vision esta
contestando bien a la pregunta equivocada. En una captura real de esquina el
pasillo mide 1064 mm de via libre justo cuando hay que doblar.

Arreglo, en tres piezas:

1. **La linea del piso dice donde esta la curva** (`linea_dispara_esquina`).
   Cruzar la primera linea del par ENTRA en la esquina por si solo, sin
   esperar a que el pasillo se cierre. Es informacion fisica, no inferencia.
2. **Anti-bucle** (`bloqueo_esquina`): mientras el carro esta dentro de la
   curva, el giro de 90 se ejecuta comprometido y la camara no puede
   redirigirlo. Se termina por angulo de giroscopio o por timeout, nunca
   porque se vea un hueco tentador. La vision sigue mandando en la seguridad.
3. **Una esquina, un giro**: la curva ya girada no vuelve a disparar aunque
   la zona siga activa. Sin esto el carro encadenaba dos giros de 90 seguidos
   (180 grados) y se metia en la pared.

Se sale de la zona al completar el giro, y hay un timeout
(`lineas.esquina_max_ms`) por si el giroscopio falla: nadie se queda
bloqueado "en la esquina" para siempre.

La zona se marca tambien cuando el giro nace de la vision, asi que el
anti-bucle protege aunque el TCS no este montado o las lineas no se vean.

En la web: caja **Esquinas** de la pestaña Carrera, con un boton *Probar
esquina* que inyecta un cruce de linea para ensayar la maniobra en el banco
sin empujar el carro. Mientras el carro esta en la curva el video se enmarca
en naranja.

## Que recta es cada pared (y por que hace falta el giroscopio)

Sintoma: el carro cruza la esquina, esquiva un pilar que lo empuja hacia la
esquina interna y llega TORCIDO. Ahi ve las dos rectas a la vez -- la del
tramo que deja y la del que entra -- y se desorienta: toma la pared de
enfrente por la de su carril.

El angulo de una pared MEDIDO DESDE EL CARRO no sirve para distinguirlas,
porque cambia con lo torcido que vaya. El angulo respecto a la PISTA si, y
pasar de uno a otro solo necesita cuanto se ha desviado del rumbo de la recta,
que es justo lo que da el giroscopio:

    angulo_pista = angulo_carro - desvio_de_rumbo

Con eso, ~90 grados es una pared LATERAL (la de tu carril) y ~0 es la pared de
FRENTE (el fondo de la curva), aunque el carro vaya cruzado 45 grados. Y basta
con ver UNA pared para saber cual es: no hace falta ver las dos.

Medido sobre la escena sintetica del selftest, con el carro cruzado 45:

| | sin giroscopio | con giroscopio |
|---|---|---|
| pared de enfrente | no la reconoce (`otro`) | `frontal` a 1100 mm |

Sin giroscopio el fallo es SEGURO (no sabe) y no peligroso (confundirse): al
salirse de la tolerancia, la pared se marca `otro` y no se usa.

Que se hace con eso:

- `interna_mm` / `externa_mm`: distancia a la pared de tu carril, resuelta con
  el sentido de la ronda (en horario la interna es la derecha).
- `frontal_mm`: la pared cruzada. Dispara la esquina por si sola, y es un
  disparo POSITIVO: no se confunde con una pared lateral vista de refilon.
- La estrategia `pared` sigue la interna identificada en vez de la media de la
  banda de la imagen, que en una curva mezcla las dos rectas.

En el video: verde = pared lateral (con `lat izq` / `lat der`), rojo = pared
de FRENTE, gris = tramo sin orientacion clara. Debajo, la linea
`int ... | ext ... | frente ... | desvio ...`. Se apaga con
`navegacion.usar_rectas`.

## Giro de 90 en dos tiempos (para no perderse obstaculos)

`giro2t.activo`. En vez de doblar de una sola pasada, el carro **avanza en
diagonal y luego retrocede con la direccion invertida**. Con direccion
Ackermann el sentido de rotacion es el mismo en los dos tramos (es la
maniobra de dar la vuelta en una calle estrecha), asi que gira casi sobre el
sitio y termina **alineado con el tramo nuevo**, viendo el pasillo entero de
frente. Eso es lo que evita que un pilar se quede fuera de cuadro y el carro
se lo salte.

**SOLO se ejecuta en una esquina confirmada por el PAR DE LINEAS del piso**
(TCS o camara). Es la unica maniobra del carro que retrocede, y retroceder en
mitad de una recta -- porque la vision creyo ver una esquina donde no la hay --
es meterse contra lo que venga detras. Sin esa prueba fisica se hace el giro
normal, que solo va hacia adelante. Si la zona de esquina caduca a mitad de la
maniobra, la reversa se corta en el acto y los grados que falten se completan
hacia adelante. (Consecuencia practica: con el TCS sin montar y las lineas mal
vistas, el giro de dos tiempos no llega a usarse nunca.)

El **escape de seguridad** es otra cosa y no lleva ese candado: es la ultima
red contra un choque y tiene que poder retroceder este donde este.

Es mas lento y necesita giroscopio (mide el angulo acumulado paso a paso, no
por diferencia contra el inicio, que se envuelve). Sin MPU cae solo al giro
normal. Si le faltan grados repite avance+reversa hasta `max_ciclos`.

El retroceso esta acotado por `reversa_max_ms`: el reglamento solo permite ir
marcha atras dentro de la seccion y la vecina, y esta maniobra no debe cruzar
el limite hacia atras. El escape de seguridad no interrumpe la maniobra (ella
lleva su propia reversa); corta sola en `min_pasillo_mm`, que va por encima
de `parar_bajo_mm`.

## El giro fantasma de la azul que llega tarde

Sintoma: al cruzar la naranja el carro giraba bien, pero rato despues -ya con
el giro hecho- aparecia la azul y disparaba OTRO giro de 90, a veces hacia el
lado contrario y encima entre pilares.

Causa: el codigo trataba cada linea por separado. Si la azul llegaba fuera de
la ventana del par (se detecta bastante peor que la naranja: es mas oscura y a
la velocidad de paso cae entre dos muestras del sensor), se tomaba por la
PRIMERA linea de una esquina nueva. Y al emparejarse desfasada salian pares
"al reves", que a los dos acababan invirtiendo el sentido de la ronda.

Ahora una esquina se **abre** con su primera linea y se queda esperando a la
otra:

```
naranja  -> ABRE la esquina (entra en la zona, dispara el giro de 90)
azul     -> la CIERRA, tarde o temprano; nunca abre otra
naranja otra vez -> rebote en el borde, se ignora
```

La espera dura `lineas.cierre_max_ms` (6 s, toda la curva), muchisimo mas que
la zona de esquina, que se cierra en cuanto termina el giro. **Si la azul no
llega nunca**, la esquina la da por contada el giro de 90 y se sigue esperando
por si aparece, para que cierre esta en vez de abrir una nueva. Una esquina se
cuenta UNA sola vez, la den por buena las lineas o el giro.

Probado: con cuatro esquinas seguidas perdiendo SIEMPRE la azul, la cuenta
sigue dando 4.

## Modo de esquina por color (`esquina_color.activo`)

Dos cosas seguian pasando en pista con el giro normal: (1) al llegar a la
curva el giro de 90 iba **forzado** y se llevaba por delante los pilares de la
salida, y (2) el carro cruzaba la naranja, giraba, luego veia la azul y
**volvia a girar**. Este modo lo resuelve simplificando, no añadiendo: el TCS
solo hace DOS cosas.

```
1a linea de la ronda   -> fija el sentido: naranja = horario, azul = antihorario
cada linea de ESE color -> cuenta una esquina y dispara el giro hacia ADENTRO
                          (horario = derecha)
la del otro color       -> NO EXISTE: ni gira, ni cuenta, ni es "linea reciente"
la misma otra vez       -> rebote (esquina_color.refractario_ms), no cuenta
```

El giro (`GIRO_COLOR`) va hacia adentro con **velocidad variable**: `vel_max_pct`
con el pasillo despejado y `vel_min_pct` con el muro encima, interpolado con
el pasillo. Con giroscopio se termina al clavar los 90 (misma ley que el giro
normal, topada en `dir_pct`); sin el, cuando el pasillo abre; y siempre hay
`max_ms`. Solo va hacia adelante: el ESCAPE sigue por encima.

**Se suelta en el acto al ver un pilar** (`ceder_al_pilar`): en cuanto hay un
pilar en juego -visto por la camara a la distancia que sea, o en el punto ciego
mientras se le adelanta- el navegador pasa a RECTO y el esquive tiene el
volante, sin tocar nada del esquive. Mientras tanto el rumbo de referencia ya
apunta a la recta NUEVA, asi que el giroscopio sigue tirando hacia adentro
(acotado por `yaw_max` y cediendo al pilar). Cuando el pilar lleva
`reanudar_tras_ms` fuera de juego, se **retoma** el giro por los grados que
falten (`reanudar`); si esquivando ya quedo encarado, la esquina se da por
hecha. Si un escape interrumpe el giro, al volver tambien se retoma.

Con el modo encendido no se usan ni el par de lineas ni el giro de dos
tiempos. La esquina se cuenta al **pisar** la linea (antes de girar), asi que
la carrera espera a salir de la zona de esquina antes de arrancar la parada
en meta. La vision (pasillo cerrandose, pared de frente) sigue pudiendo
disparar este mismo giro por si el TCS falla (`vision_dispara`); tambien se le
puede quitar el voto.

**Y el giro de 90 completado cuenta la esquina que la linea no conto**
(`contar_giro_sin_linea`, encendido). Esto empezo apagado con el argumento de
"mejor perder una que sumar una", y en pista salio carisimo: en modo color la
carrera no llega a la meta de esquinas si el TCS se pierde UNA sola linea en
toda la ronda, asi que el autostop no dispara nunca y el carro sigue dando
vueltas hasta que se acaba el tiempo — sin parada en meta y sin bonificacion.
Contar por giro no duplica porque el guardia es **`_contada`** (esta curva ya
la conto una linea), no el refractario: el giro termina *segundos despues* de
pisar la linea, mucho mas que `refractario_ms`, asi que con el refractario
como unico guardia la misma curva se contaba dos veces. Ademas el giro tiene
que ir hacia el lado de la ronda.

**La linea es la que dispara el giro, y eso vive en `navegacion.linea_dispara_esquina`.**
Es el parametro mas facil de dejar apagado sin darse cuenta y el que mas caro
sale: con el apagado la linea del piso **solo cuenta**, no gira, y el carro
sigue de largo por la esquina hasta que la vision se asusta a `girar_bajo_mm`
de la pared — que es demasiado tarde para doblar, asi que acaba en escape. El
sintoma exacto es "cruza la linea y no gira". El selftest lo comprueba ahora
sobre el perfil activo: si `esquina_color.activo` esta encendido,
`linea_dispara_esquina` tiene que estarlo tambien.

Ojo con el **sentido**: si el TCS se pierde la primera linea, la siguiente que
pisa es la del OTRO color — la de salida de la curva — y esa declara el
sentido contrario para toda la ronda. Es exactamente lo que pasaba en
antihorario mientras el `c_min` se comia las azules: el carro tomaba la
naranja de salida como "horario" y doblaba a la derecha en cada esquina, o
sea contra la curva, sin poder salir de ella. Por eso ahora **la camara vota
el sentido antes de pisar nada**: si ve las dos lineas del piso delante, la
mas cercana es la que se cruzara primero, y con `sugerencia_votos` cuadros
seguidos de acuerdo esa sugerencia **manda sobre la linea pisada**. Funciona
aunque `lineas.usar_camara` este apagado, porque sugerir no es contar: la
camara no cuenta esquinas ni marca cruces, solo dice hacia donde va la ronda.
Con `carrera.sentido` forzado desde la web tampoco pasa, y ahi cuenta el color
de ese sentido aunque la primera linea vista fuera la otra.

Y por si aun asi el sentido sale al reves —la camara puede votar mal, o el TCS
puede sencillamente no ver ese color—, hay una **red de ultima instancia**: si
el sentido esta invertido, todas las lineas que el carro pisa son "la otra",
se ignoran una tras otra y el marcador de esquinas **no se mueve nunca**. Eso
no es mala suerte, es que el color que estamos esperando no va a llegar. A las
`ignoradas_para_invertir` (2) lineas ignoradas **sin una sola esquina
contada**, se adopta el color que el sensor si esta viendo y la ronda se
salva. Con una esquina ya contada no vuelve a dispararse nunca, asi que la
segunda linea de cada curva —que se ignora siempre, y con razon— no puede
invertir nada; y con el sentido forzado desde la web no se toca. En la web
sale como *inversiones* en la fila de lineas: si ahi hay algo distinto de 0,
el sentido automatico fallo y hay que mirar el TCS.

En banco: pestaña Carrera, boton **Esquina por color**, luego *Probar esquina
(linea naranja)* -> debe girar a la derecha; *Simular linea azul* -> debe salir
"ignorada: cuentan las naranja" en la fila *lineas*. El video marca la zona
como "giro por color, cuenta naranja; cede al pilar".

Medido en el selftest (modelo Ackermann): pilar visible entre los ticks 6 y
40 de la curva -> suelta en el tick 6, retoma en el 40, 90 grados clavados en
46 ticks, una sola esquina contada; con el pasillo en 450 mm la velocidad del
giro baja de 40 a 26.

## El giro al lado incorrecto: la recta que se quedo atras

Sintoma en pista (con el modo por color, pero pasa igual con el giro normal):
el carro cruza la naranja y dobla bien, y acto seguido **sigue doblando al
lado contrario**, sin haber pisado otra linea, hasta la esquina o la pared.
Otras veces, en la linea siguiente, gira directamente al lado incorrecto.

Causa: la referencia de rumbo (`rumbo_recta`) solo avanzaba 90 grados cuando
el codigo registraba una esquina. Si el TCS se pierde la linea, el centrado
toma la curva solo por el hueco blanco del muro interno: el carro dobla bien
fisicamente, pero el codigo sigue creyendo que la recta es la de ANTES. El
giroscopio tira al 45 % hacia esa recta vieja, que ahora queda al lado
contrario, y el carro se va a la pared. Y en la linea siguiente el objetivo
se calculaba sumando 90 a esa recta desfasada: giro 90 grados fuera de sitio.
Habia un segundo agujero: `enlace.yaw()` devuelve `None` si la telemetria
lleva mas de 0,4 s sin llegar, y en ese instante el apuntado se saltaba en
silencio; al volver el yaw, el giro "terminaba" con la referencia vieja.

Arreglo (`navegacion.reanclar_rumbo`, encendido):

- **En recta**: si el carro lleva `reanclar_ms` girado entre
  `reanclar_desde_deg` y `reanclar_max_deg` respecto a la recta, **en el
  sentido de la ronda**, eso fue una esquina sin registrar: se adopta la recta
  nueva en vez de tirar hacia la vieja. Girado en contra del sentido no cuenta
  (es esquivar o entrar torcido) y mas alla de 135 tampoco: ahi ya no se
  distingue de una vuelta sobre si mismo y manda el rescate de
  `desvio_max_deg`, como antes. Con el sentido desconocido no se toca.
- **Al apuntar una esquina**: el objetivo sale de la recta REAL (misma regla),
  no de la acumulada. Vale para el giro normal y el giro por color.
- **Sin yaw al entrar**: el giro queda pendiente de apuntar y lo hace en cuanto
  el yaw vuelve, descontando lo girado a ciegas.

Medido en el selftest: tras doblar solo 90 grados, el codigo viejo pedia
-45 % de volante (izquierda, en horario) de forma sostenida; con el re-anclaje
la recta pasa a 90 y el volante queda centrado. En la web sale como *rumbo
re-anclado: N esquina(s) sin registrar* y en el motivo de la decision.

Comprobacion aparte, que el codigo no puede hacer por ti: si en la web
`carrera.sentido` esta forzado a "horario" y corres en antihorario, el modo
por color ignora las azules y gira a la derecha en cada naranja, que en
antihorario es la linea de SALIDA de la curva. Dejalo en AUTO salvo en pruebas.

## La curva tiene que CABER: girar_bajo_mm contra parar_bajo_mm

El sintoma era "se acerca muchisimo a la pared antes de girar, y a veces
retrocede ahi mismo". No era el sensor ni el control: era aritmetica.

`girar_bajo_mm` es la distancia a la que la vision dispara la esquina, y la
maniobra **consume pasillo**:

| | |
|---|---|
| radio de giro con `dir_pct` 90 % (150 mm entre ejes, 26 grados a tope) | **347 mm** |
| avance del morro durante los 90 grados (`hypot(R, 200) − 200`) | **200 mm** |
| avance del pre-giro (`retardo_giro_ms` 220 ms a `vel_giro` 44) | **82 mm** |
| **pasillo que se come la maniobra** | **282 mm** |

Con `girar_bajo_mm` en **483** quedaban 483 − 282 = **201 mm** al acabar la
curva, por debajo de `parar_bajo_mm` (300): **el escape saltaba en mitad de
todas las curvas**, siempre, por construccion. Con 700 quedan 418 mm y la
curva entra entera. El selftest hace ahora esa cuenta sobre el perfil activo,
asi que el numero no se puede volver a bajar sin que salte una prueba.

Y hay un segundo escape que no debia estar: **el pasillo se mide recto delante
del carro**, asi que en mitad de un giro de 90 el morro barre hacia la pared y
la medida baja sola aunque el carro este rotando perfectamente. Dentro de un
giro comprometido el umbral se reduce a `escape.factor_en_giro` (0,6) de
`parar_bajo_mm`; con el muro de verdad encima el escape sigue mandando igual.

Aparte, `escape.salir_mm` separa "cuando doy el escape por bueno" de
`girar_bajo_mm * 0.8`, que era de donde salia: subir la distancia de disparo de
la curva alargaba tambien todas las reversas, que no tienen nada que ver.

## Mantener el carril: distancia al muro, no hueco medio

`centrado` equilibra el **espacio libre** de las dos bandas laterales, y eso no
es lo mismo que ir por el medio del carril: cuando el muro interno se acaba, el
hueco de ese lado crece aunque el carro no se haya movido. Por eso el carro se
acercaba a las paredes en las rectas, sobre todo despues de una curva.

`estrategia: pared` mide de verdad: sigue el **muro interno identificado**
(`interna_mm`, en mm) a `pared_objetivo_mm`. Y hay una razon geometrica para
elegir ese numero: para rodear una esquina de 90 grados **manteniendo la
distancia d al muro interno, el radio de giro necesario ES d**. Si la distancia
de seguimiento y el radio de la curva coinciden, la curva sale exacta: se entra
a d de un muro y se sale a d del siguiente. Con `dir_pct` 90 el radio es 347 mm,
asi que `pared_objetivo_mm` va a 347.

Cuidado con el fallback, que estaba mal: sin pared interna identificada se caia
a `banda * alcance_mm`, y eso **no es una distancia lateral** — es el hueco
medio *hacia delante* en ese lado de la imagen, que en recta vale 600-700 mm.
El error contra `pared_objetivo_mm` salia de casi 300 mm y el volante se iba al
65 % contra el muro interno por un numero que no media lo que decia. Ahora sin
pared identificada se cae al centrado, y el seguimiento va topado en
`pared_max_pct`: mantener el carril es una correccion, no una maniobra.

## El zocalo del muro no es una linea azul

Al bajar `c_min` para que entrara la azul (C=678) tambien empezo a entrar la
**sombra de la base del muro**. Ahi el canal claro cae a 200-400 y pasan dos
cosas a la vez: el negro del tapete tira a azulado, y con el claro tan bajo los
ratios son ruido (a C=300 un solo conteo mueve el ratio 0,85 puntos, asi que la
diferencia b−r baila +-6 sola). Con `azul_dif_min` en 12 eso clasifica como
azul. El carro "veia color" antes de tiempo y —ahora que la linea dispara el
giro— **doblaba ahi mismo**.

`c_min` sube a 400: sigue muy por debajo de la linea (678, margen de 1,7x) y
deja fuera la sombra. Y `muestras_min` pasa de 1 a 2, que a 24 ms de
integracion y con el freno ante linea encendido siguen sobrando muestras sobre
una linea de verdad, pero matan el pico suelto. El selftest comprueba las dos
cosas sobre el perfil activo: la azul real entra, la sombra del zocalo no.

## Cuatro cosas que hacian que el carro se quedara pegado en la curva

Todas salen del mismo sitio —**que pasa cuando la curva no sale perfecta**— y
se alimentaban entre ellas.

**1. El timeout se tomaba por meta cumplida.** Al vencer `esquina_color.max_ms`
el codigo hacia:

```python
if vencido:
    self.rumbo_objetivo = yaw       # "doy por buena la curva a medias"
```

Si la curva iba por 50 de los 90 grados, el carro salia **40 grados cruzado** y
el giroscopio se dedicaba a MANTENER ese rumbo, derecho contra la pared de
enfrente. Y como `rumbo_recta` si se habia avanzado los 90 al empezar la curva,
el desvio quedaba en −40: ni re-anclaje (pide +65) ni rescate (pide 110), asi
que nadie lo corregia. Ahora el objetivo sigue siendo la recta nueva y lo que
falte lo termina el control de rumbo en recta, acotado por `yaw_max`. En la web
sale `giros_vencidos`: si sube, la curva no cabe en `max_ms`.

**2. La curva no cabia en el tiempo que tenia.** `max_ms` estaba en 3500 ms.
Con `dir_pct` 75 el radio sale de unos 424 mm, o sea 666 mm de arco para los
90 grados; a `vel_min_pct` 22 (187 mm/s, que es lo que se usa con el muro
encima) eso son **3,6 s**. Vencia casi siempre. Ahora `dir_pct` 90 (radio
~350 mm, 550 mm de arco) y `max_ms` 5000.

**3. Al salir de la curva, la vision abria otra en el acto.** Terminado el
giro, el carro sigue metido en la geometria de la esquina: el pasillo todavia
mide menos que `girar_bajo_mm`, asi que la vision disparaba una esquina nueva
inmediatamente. Eso avanzaba `rumbo_recta` **otros 90 grados**, el carro se
ponia a doblar 180 en el mismo sitio, y cada uno de esos giros de mas contaba
una esquina. `min_recto_ms` (700 ms, unos 24 cm) no daba ni de lejos para
salir de la curva. Ahora hay `navegacion.tras_giro_ms` (1200 ms) durante los
cuales **la vision** no puede abrir otra esquina; **la linea del piso si**,
porque esa es un hecho fisico y trae su propio refractario.

**4. Y aunque la abriera, no puede sumar otros 90.** Si la curva anterior
acabo por tiempo con grados pendientes, `_apuntar_a_la_recta_siguiente` ya no
avanza la referencia: termina la que estaba. El aviso es un **hecho
registrado** (`_giro_incompleto`), no un umbral de angulo, porque entrar
torcido en contra —esquivando un pilar hacia el muro exterior— se parece mucho
y ahi la referencia SI tiene que avanzar.

Ademas, `giro_tolerancia_deg` estaba en **14,1 grados**: la curva se daba por
hecha hasta 14 grados corta, que en un metro de recta son 25 cm de deriva
contra la pared. Vuelve al 8 de siempre. Y `apertura_pct` baja de 25 a 8: ese
"giro abierto de camion" empuja al carro **contra la pared externa** justo al
entrar en la curva, y con 307 mm de radio minimo en un carril de 1000 mm no
hace ninguna falta.

## La reversa del escape tiene que mirar hacia donde dobla la ronda

Sintoma en pista, en antihorario: el carro llega a la esquina, no cabe,
**retrocede girando hacia el lado contrario del que hace falta** y se queda
encajado en la curva, intento tras intento.

El volante de la reversa se elegia siempre por geometria:

```python
lado_muro = -1 if p.izq < p.der else 1      # "donde se ve mas hueco"
```

y a menos de `parar_bajo_mm` de la pared, con el muro llenando el cuadro, esa
diferencia **es ruido**: las dos bandas laterales leen practicamente lo mismo y
el signo sale a cara o cruz. La mitad de las veces el carro retrocedia girando
al reves. Es la misma leccion que ya estaba escrita diez lineas mas arriba,
para el giro de rescate — *"elegir donde haya mas hueco es lo que podia dejar
al carro encarado hacia atras"* — y que a la marcha atras no se le habia
aplicado.

Dentro de una esquina el lado **no hay que mirarlo**: la ronda dobla siempre
hacia el mismo sitio. Y con direccion Ackermann, reversa + volante a un lado
rota el morro hacia el lado *contrario* (la misma geometria que usa el giro de
dos tiempos), asi que el volante correcto es `-sentido`:

| Ronda | Volante en la reversa | Que hace el morro |
|---|---|---|
| horario (`+1`) | izquierda | rota a la derecha, hacia la curva |
| antihorario (`-1`) | **derecha** | rota a la izquierda, hacia la curva |

Fuera de la esquina sigue mandando la vista: ahi el muro de delante suele ser
una pared lateral porque el carro va cruzado, y girar "hacia la ronda" lo
meteria mas contra ella. En la web el motivo de la decision dice cual de las
dos reglas se aplico: *escape #N pasillo=... (hacia la curva)* o
*(separandose del muro)*.

## Freno ante linea (que el TCS no se la salte)

El TCS integra 24 ms por muestra (`tcs.atime` 246): a velocidad de crucero
una linea de 2 cm deja 1 o 2 lecturas y a veces ninguna. La camara ve la
linea ANTES de que pase bajo el carro (donde la camara ya no llega), asi que
`lineas.frenar_ante_linea` baja la velocidad en recta a `vel_linea_pct` desde
`frenar_desde_mm` de la linea y la mantiene `frenar_tras_ms` despues de que
la linea salga por debajo del cuadro, o de que el TCS avise del cruce.
Funciona aunque `usar_camara` este apagado: la camara solo mide la distancia,
no cuenta ni cruza. En la web: fila *linea vista / freno*.

Si aun asi se pierde lineas, la otra palanca es `tcs.atime` (bajarlo da mas
muestras por linea) y despues **repetir la calibracion del TCS**, porque
cambian todos los valores absolutos.

## Tres lineas de calibracion

Los perfiles (de parametros y de color) van etiquetados en tres listas:
**Open Challenge**, **Esquivar obstaculos** y **Obstaculos + estacionar**.
Cada linea guarda hasta **20** perfiles y se eligen por separado, asi que
afinar el reto de obstaculos no toca la calibracion buena del Open. El
selector esta arriba de cada lista de perfiles; lo que guardes va a la linea
que tengas puesta. Los perfiles que ya existian quedaron en "Open Challenge".

### ...y una CARPETA por reto

Las etiquetas bastan para calibrar en el taller, pero comparten archivo: un
guardado distraido desde la web con el reto equivocado seleccionado se lleva
por delante la calibracion buena del Open Challenge, y en competencia no hay
tiempo de rehacerla. Por eso cada reto puede tener ademas su propia carpeta:

| `--reto` | parametros | colores |
|---|---|---|
| `open` (por defecto) | `config/params.json` | `config/colors.json` |
| `obstaculos` | `config/obstaculos/params.json` | `config/obstaculos/colors.json` |
| `estacionar` | `config/estacionar/params.json` | `config/estacionar/colors.json` |

```bash
python3 tools/crear_config_obstaculos.py   # siembra config/obstaculos/
python3 main.py --reto obstaculos          # y el carro arranca con esa
```

Lo que guardes desde la web va a la carpeta del reto con el que arrancaste.
Sin `--reto`, todo queda exactamente como estaba.

## El esquive: tres cosas que estaban mal

**1. El lado de paso se invertia** (se veia sobre todo en antihorario, pero no
era el sentido). El recorte del punto de paso contra el muro podia dejarlo al
OTRO lado del pilar. Con `margen_mm=261` el despeje pedido son 386 mm, que casi
nunca caben: con el pilar a 300 mm hacia la pared, el objetivo de 686 mm se
recortaba a 260 y el carro se iba a pasar un pilar ROJO por su izquierda.
Pasar por el lado incorrecto **termina la ronda** (regla 9.25.5). Ahora el lado
es intocable: si el hueco no da para el despeje comodo se aprieta hasta el
minimo fisico (medio carro + medio pilar), pero nunca se cruza; y si ni eso
cabe se avisa en pantalla (`no cabe por la derecha`) y manda la seguridad del
muro. El lado sale del COLOR, no del sentido, en horario y en antihorario.

**2. El volantazo.** La correccion de rumbo por giroscopio se sumaba DESPUES
del esquive. Mientras esquivas a proposito te sales del rumbo de la recta, asi
que esa correccion o **suma** (con el esquive topado en 55 % y `yaw_max` en
45 %, el volante llegaba al 100 % y la rueda trasera se llevaba el pilar) o
**resta** (y entonces no esquiva bastante). Con `navegacion.yaw_cede_al_esquivar`
el mantenimiento de rumbo cede en la misma proporcion en que manda el pilar:
medido, el volante pasa de +87 / +23 segun como estuviera cruzado, a +55 en
los dos casos.

**3. Modo de esquive por BORDE** (`obstaculos.modo = borde`). En vez de apuntar
a un hueco en milimetros, se vigila el canto del pilar que da al centro y se le
mantiene sobre una columna objetivo cerca del borde de la imagen
(`borde_frac`): al rojo se le empuja al canto izquierdo y al verde al derecho.
Es control visual puro, no depende de la calibracion de distancias. Al tenerlo
encima (`giro_final_mm`) mete el **giro grande** que lo libra con la cola. Y si
el pilar se sale de cuadro **estando todavia lejos**, se le busca para volver a
encuadrarlo (`buscar_pct`); de cerca no, que de cerca manda el compromiso de
adelantamiento. En el video se ve la linea del canto objetivo y la flecha.

**Memoria del ultimo pilar** (`obstaculos.recordar_lado`, se puede apagar):
guarda color, por que lado habia que pasarlo, donde estaba y a que distancia,
durante `memoria_ms`. Sirve en la curva, cuando el pilar sale de cuadro al
girar y el carro se olvidaba de que lo tenia al lado. Se ve en la telemetria.

## Identificar el pilar ANTES de decidir el lado

El lado de paso es la unica decision del reto que, si sale mal, **termina la
ronda** (Apendice A, seccion 5). Y se decidia con una linea:

```python
lado = +1 si color == "rojo" si no -1
```

...recalculada desde cero en cada frame, creyendose la etiqueta del detector
tal cual. Falla por tres sitios a la vez, y el sintoma en pista siempre es el
mismo: **el carro esquiva, pero por el lado que le da la gana**.

**1. El magenta se colaba como rojo.** Los dos topes del cajon de
estacionamiento son magenta, RGB(255,0,255) = **tono 150** en la escala de
OpenCV. El rango de rojo calibrado empezaba **justo en 150**, asi que cada
delimitador se veia tambien como pilar rojo y el carro intentaba pasarlo por su
derecha. Y no es una señal: es un muro de 100 mm que no se puede mover (reglas
13.7 a 13.9). Ahora toda deteccion roja o verde que caiga encima de una mancha
magenta se descarta (`ignorar_magenta`, `solape_magenta`), asi que funciona
**aunque el rango de rojo siga abierto**. El magenta pasa a rodearse como lo
que es: un obstaculo sin lado obligatorio (`esquivar_magenta`).

**2. Cualquier mancha del color valia como pilar.** Los filtros de area,
llenado y aspecto son de IMAGEN: no saben de tamaños reales. Una sombra rojiza
del zocalo, un reflejo en el tapete o un trozo de pared los pasaba. Una señal
mide **50x50x100 mm** (regla 13.1): sabiendo a que distancia esta, la geometria
dice cuanto TIENE que medir en pixeles, y lo que no cuadra no es un pilar
(`verificar_tamano`). Los objetos recortados por el canto de la imagen no se
descartan, que de cerca siempre se salen del cuadro.

El peligro de esta prueba es que, mal calibrada, no se equivoca poco: **borra
todos los pilares y deja al carro ciego**. Por eso tiene tres niveles.
`suave` (por defecto) comprueba solo el **alto**, que depende de `fy` — el
parametro fiable, porque la altura de captura es la misma con la que se
calibro — y del ancho solo mata lo imposible; separar el pilar del delimitador
queda entonces en manos del veto por solapamiento, que no usa geometria
ninguna. `estricto` añade la banda de ancho real en mm (25-120), que caza al
delimitador aunque el magenta este mal calibrado, pero **necesita `fx` bien
medido** (ver la seccion de arriba). `no` la apaga.

**3. El lado parpadeaba.** Aunque el detector falle solo uno de cada cinco
frames, el lado se recalculaba entero en cada uno: un frame rojo (objetivo a la
derecha), el siguiente magenta-leido-como-verde (objetivo a la izquierda). La
media de eso es ir de frente contra el pilar. Ahora cada pilar **se sigue entre
frames** (`emparejar_mm`, `pista_ms`), el color se decide por **votacion**
(`votos_color`) y, en cuanto gana, el lado queda **clavado** (`fijar_lado`)
hasta que el pilar se pierde. Mientras tanto ya se esquiva con el color que va
ganando: no se espera parado.

En el video, el rotulo dice el objeto identificado y en que punto esta la
decision: `[votos 2]` mientras se vota, `[FIJO]` cuando ya no cambia, y
`(cajon: lado libre)` si es un delimitador magenta.

```bash
python3 tools/selftest_obstaculos.py    # el lado, sin camara ni pista
```

Comprueba los tres fallos de arriba con detecciones sinteticas proyectadas con
la geometria real. Va incluido en `tools/selftest.py`.

## "El pilar esta ahi delante y el carro no lo ve"

Sintoma en pista: un cubo perfectamente visible a un metro, **sin recuadro** en
el video, y el rotulo diciendo *ADELANTANDO*. El carro lo rodea de casualidad,
por el lado que toque.

**Lo primero: el recuadro lo dibuja el DETECTOR DE COLOR, antes del esquive.**
Si no hay recuadro, el esquive ni se entero — el problema esta en la mascara de
color o en los filtros de forma, y tocar los parametros del esquive no va a
arreglarlo nunca. Si hay recuadro pero el carro lo ignora, entonces si es el
esquive, y el rotulo naranja dice cuantos descarto y por que.

```bash
python3 tools/diagnostico_pilares.py            # con la camara en vivo
python3 tools/diagnostico_pilares.py --imagen capturas/xxx_crudo.png
```

Abre la caja: para cada color dice cuanto del cuadro coge la mascara, las
manchas aceptadas con su tamaño REAL en mm, las rechazadas con el filtro exacto
que las mato, y que decide el esquive con lo que queda. Para capturar un frame
crudo desde el movil, sin tocar la interfaz:

    http://carrito.local:8080/api/cmd?capturar=1

**Las dos causas que se llevan la tarde**, y que ahora se avisan en cada
arranque del reto:

**1. Exposicion y balance de blancos en AUTO.** El tapete es blanco y
brillante: al girar hacia una pared clara la camara se reajusta sola, el tono
de los pilares se mueve con ella y la mascara se cae unos frames si y otros no.
Eso es exactamente "a veces no ve los cubos". Congelalos en Ajustes **antes**
de calibrar colores, o la calibracion no significa nada. El perfil sembrado ya
fija el balance de blancos en 4500; la exposicion hay que ponerla a mano
mirando el video, porque el valor util depende de la camara y del sistema (en
Linux/V4L2 son positivos, en Windows/DSHOW negativos).

**2. `fx` y el ancho de captura.** `fx_px` se guarda referido a **640** de
ancho y se escala con el ancho real. Capturando a **1920** con un `fx`
calibrado a 640, el `fx` efectivo sale **tres veces mayor**, y entonces
`lateral_mm()` mide un pilar de 50 mm como 17: el punto de paso apunta a donde
no es y `verificar_tamano` en estricto los borra todos. En el Open Challenge
casi no se nota, porque la distancia al muro sale de las FILAS (o sea de `fy`);
aqui decide por que lado se pasa. El diagnostico te dice cuanto mide un pilar
de verdad: **tiene que dar ~50 mm de ancho y ~100 mm de alto**. Si no, recalibra
`fx` en la pestaña Calibracion con la captura a la resolucion que vas a usar.

### Y cuando el detector parpadea de todas formas

Siempre parpadea un poco. Antes, en cuanto el pilar se perdia un par de frames
el esquive soltaba el volante y se iba **recto**: el carro se olvidaba del lado
y lo adelantaba por donde tocara. Ese era el *"lo esquiva y ya"*.

Ahora (`seguir_a_ciegas`, encendido) se sigue conduciendo contra la **ultima
posicion conocida**, adelantada cada frame con la velocidad real del carro,
hasta `ciego_max_ms`. En el video sale `A CIEGAS 0.2s`. Hace falta haberlo
visto `ciego_vistas_min` veces, para que un destello de color en una pared no
se lleve al carro medio segundo hacia un fantasma. Si el rotulo dice *A CIEGAS*
casi siempre, no toques el esquive: tu deteccion de color esta parpadeando y
hay que arreglar eso.

## Los pilares de la seccion siguiente no son de esta recta

Las lineas naranja y azul marcan el limite de la seccion. Un pilar que se ve
por DETRAS de ellas esta en el tramo siguiente. Si se le hace caso desde la
recta, el esquive tira del carro justo cuando toca prepararse para la curva:
se pega a la esquina interna, no le queda sitio para abrirse y engancha el
canto al girar.

`obstaculos.limitar_por_lineas` (encendido) descarta los pilares mas lejanos
que la linea mas cercana, con `margen_linea_mm` de holgura para que uno justo
antes de ella siga contando. **En cuanto el carro entra en la zona de esquina
el filtro se levanta** y esos pilares pasan a contar, que es cuando de verdad
hay que esquivarlos. Si no se ve ninguna linea, no se filtra nada.

## Como se pasa un pilar (y los tres fallos que tenia)

**1. Se iba al tope de direccion.** El angulo al punto de paso se calculaba
con la distancia al pilar, asi que al acercarse el mismo desvio lateral pedia
cada vez mas angulo, hasta el tope. Medido con los valores que el equipo tenia
en pista: **96 % de volante a 600 mm y 100 % a 400 mm**, con el pilar poniendo
el 97 % del mando. Arreglado con tres cosas:

- `mirada_min_mm` (350): mirada minima al calcular el angulo. Es lo que impide
  que el volante se dispare de cerca.
- `dir_max_pct` (55): tope de lo que puede pedir el esquive. Un pilar nunca
  deberia mandar el volante a fondo.
- `rampa_dir_pct_s` (220): cuanto puede cambiar por segundo, para que no haya
  volantazo en un solo frame.

**2. La rueda trasera se llevaba el pilar.** Cuando el pilar queda muy cerca
deja de verse: la camara no llega tan abajo. En ese momento el esquive
desaparecia, el centrado tiraba del carro hacia el medio del carril y, con
direccion Ackermann, **la cola corta por dentro** y barre el pilar. Aumentar
el margen no lo arregla: el carro se abre antes y vuelve igual de pronto.

Ahora hay **compromiso de adelantamiento**: desde que el pilar se pierde de
vista se pide RECTO (no se vuelve hacia el) el tiempo que el carro necesita
para adelantarlo con todo su largo, calculado con la velocidad real y el nuevo
`geometria.largo_carro_mm`. Se ve en el video: *ADELANTANDO 0.6s (recto)*.

**3. Un pilar pegado al muro interior lo llevaba de frente contra la esquina.**
Con `peso_max = 1.0` el pilar ponia el 100 % del mando y la evitacion de muros
dejaba de contar justo cuando el muro era el problema. Ahora, con
`ceder_ante_muro`, el peso del pilar **se desvanece segun el pasillo se
cierra**: el muro recupera el mando. Y si el hueco entre el pilar y la pared
es mas estrecho que el carro, se apunta al **centro del hueco** en vez de
elegir un lado y empotrarse (sale avisado en el video).

### El lado de paso NO se invierte con el sentido

Reglamento 9.19: rojo por la derecha, verde por la izquierda. Son la derecha y
la izquierda **del vehiculo** — el punto 5 lo dice como "el lado del CARRIL por
el que debe circular". El mismo pilar fisico se pasa por un lado distinto en
horario que en antihorario, y eso ya sale solo de trabajar en el marco del
carro: no hay nada que invertir.

Como pasar por el lado equivocado termina la ronda, no se deja a la fe: en el
video se dibuja una **flecha verde hasta el punto de paso** y el texto
*"rojo -> paso por su derecha"*, asi que se comprueba con el carro parado,
antes de arrancar. Si en tu competencia se interpretara al reves, esta el
interruptor `obstaculos.invertir_en_antihorario` (apagado por defecto).

### Valores del modo `punto` (el de antes)

Lo de arriba describe el modo `punto`, que sigue existiendo pero **ya no es el
que se usa**: con `k_dir` 1.4 y `peso_max` 0.8 el carro se apartaba un poco y
se llevaba el pilar con el costado (la seccion siguiente explica por que, con
numeros). Si se vuelve a el, estos son sus valores:

| Parametro | En pista | Para `punto` | Por que |
|---|---|---|---|
| `k_dir` | 2.83 | 1.4 | ya no hace falta forzar |
| `margen_mm` | 261 | 90 | el margen no arreglaba la cola; el compromiso si |
| `peso_max` | 1.0 | 0.8 | con 1.0 el muro deja de contar |
| `activar_desde_mm` | 4000 | 1600 | 4 m es media pista: el pilar mandaba desde lejisimos |

## Por que seguia chocando: el volante, no el lado

Con todo lo de arriba el LADO ya salia bien... y el carro seguia llevandose el
pilar por delante. Para verlo sin quemar tardes de pista se hizo un simulador
cinematico del carro, `tools/selftest_esquive.py`: modelo de bicicleta con
direccion Ackermann y el radio de giro real, servo y motor con retardo, una
camara que proyecta cada pilar con la MISMA geometria que usa el carro y lo
pierde igual que la de verdad (por el canto de la imagen a unos 25 cm, por
abajo a unos 12), las dos paredes del carril por trazado de rayos, y el
esquivador y el navegador cableados exactamente como en `robot.py`, con los
parametros de `config/obstaculos/`. Ahi el choque se reproduce (queda como
prueba de control) y se mide. Eran tres cosas, las tres en COMO se convertia
la decision en volante:

**1. El esquive pedia la mitad de lo que hace falta.** El modo `punto` pasa
el angulo al punto de paso a direccion con una ganancia fija (`k_dir`) y una
mirada minima. Pero un carro con direccion Ackermann no va "hacia donde
apunta": describe un ARCO cuyo radio depende del angulo de las ruedas.
Desplazarse 195 mm de lado (medio carro + 70 de margen + medio pilar) en
700 mm de recorrido exige un radio de ~1,3 m; en 400 mm, uno de ~500 mm, que
ya es el giro a tope de este carro. Medido en el simulador: la ganancia fija
pedia 13-19 % donde la geometria pide 50-85 %.

**2. El centrado peleaba contra el esquive, y ganaba.** La direccion final
era una media ponderada: 0,8 al pilar, 0,2 al centrado. Pero el centrado del
Open Challenge lleva `kp` 372: en cuanto el carro se desplaza 10 cm del medio
del carril pide -70 % de volante, y el 20 % de eso son -15 % que se restaban a
los +19 % del pilar. Quedaban unos +5 % efectivos. Y en el tramo en que el
peso iba subiendo de 0 a 0,8 (entre 1600 y 700 mm) el centrado mandaba mas
que el pilar: el carro se apartaba, el centrado lo devolvia al medio.

**3. Nadie frenaba.** El pilar no aparece en el perfil del muro (con metodo
`negro` solo se ve negro), asi que la velocidad seguia siendo la de crucero
(74 %, unos 630 mm/s) hasta el golpe. A esa velocidad no hay volante que
desplace el carro 20 cm en los ultimos 70.

## Como se pasa ahora un pilar: el arco, el costado y la salida

`obstaculos.modo = arco`, que es lo que trae `config/obstaculos/`. Cuatro
partes, y cada una se ve en el video.

**Aproximacion: un arco hacia la LINEA de paso.** El punto de paso es el de
siempre (a `margen_mm` + medio carro del pilar, por el lado que manda el
color). Por el pasa la linea PARALELA AL CARRIL por la que hay que ir, y se
calcula el arco que lleva al carro a esa linea (pure pursuit):

    curvatura = 2 * x / (x^2 + y^2)     [x, y del punto mirado, en el marco del carro]
    volante % = 100 * curvatura * radio_giro_mm

A tope de direccion el carro describe `geometria.radio_giro_mm`; en el rango
del servo la curvatura es casi lineal con el porcentaje. Asi el volante pedido
es el que la fisica pide, ni la mitad ni el doble. Se mira a la linea, no al
punto: el arco que pasa por un punto llega a el CRUZADO (con el pilar a 70 cm
y el paso a 20 cm de lado son 30 grados, derecho a la pared de al lado); el
arco a la linea llega paralelo, y en cuanto el carro esta sobre ella la
curvatura es cero. La mirada (`mirada_mm`, 600) se acorta hasta
`mirada_min_mm` (350) cuando queda poco que desplazarse -el pure pursuit
converge en unas cuantas miradas, y con la larga los ultimos 3 cm tardaban un
metro- y nunca mira mas alla del pilar salvo para respetar ese minimo.

Todo eso se calcula **en el marco del carril**, no en el del carro: con el
giroscopio se sabe cuanto va cruzado el carro (`error_rumbo`), y con eso el
pilar, las paredes y la linea de paso se pasan al marco de la recta. Hace
falta porque esquivando el carro va cruzado a proposito, y en su marco la
pared de al lado se ve DELANTE: el recorte del punto de paso al pasillo libre
creia que el hueco se cerraba y mandaba al "centro del hueco", de frente
contra la pared (visto en el simulador saliendo torcido de una curva). Sin
giroscopio los dos marcos coinciden y todo sigue funcionando.

Mientras el pilar manda (`mandar_desde_mm`, 1100) el peso es **1,0**: el
centrado y el rumbo se callan. El muro sigue contando por donde debe: el
punto de paso se recorta al pasillo libre sin cambiar de lado (dejando
`margen_pared_mm`, 30, menos que al pilar: rozar la pared no penaliza, mover
el pilar si) y hay una red mas abajo. Y se frena: `vel_esquive` (40 %) en
proporcion a cuanto manda el pilar.

**Al costado: seguir el pilar con el giroscopio y no barrerlo.** El pilar
sale del cuadro por el canto a unos 25 cm del morro. Desde ahi se lleva por
ESTIMA: cada frame la ficha se adelanta con la velocidad real y se ROTA con el
giroscopio (sobre el eje trasero, que es donde gira el carro), asi que se sabe
donde queda aunque no se vea, hasta que la COLA lo deja atras. Desde
`costado_desde_mm` (12 cm antes del morro) hasta la cola, el volante HACIA el
pilar se topa en la mitad de la holgura con que se le esta pasando, en %:
con 5 cm de aire, 25 %; con 1 cm, 5 %; sin aire, nada. Enderezar el rumbo hay
que permitirlo -si no el carro sigue cruzado hacia la pared de al lado todo
el paso-, pero volver al centro del carril con el pilar a la altura de la
rueda trasera es barrerlo (Ackermann: la cola corta por dentro). El
compromiso por tiempo de antes sigue de respaldo cuando no hay giroscopio.

**Saliendo (`recuperar_ms`, 1 s).** La cola acaba de dejar atras el pilar y
el carro esta pegado a una pared y algo cruzado. Se sigue a `vel_esquive` (el
siguiente pilar suele venir enseguida) y el muro se sigue midiendo como en la
maniobra (abajo).

**El muro, mientras se esquiva, se mide DE FRENTE.** El pasillo del navegador
se mide recto delante del carro, en el corredor de las ruedas, y eso vale
mientras el carro va paralelo al carril. Cruzado 10-25 grados, la pared de al
lado entra en el corredor y parece un muro de frente a 30-40 cm: en el
simulador eso frenaba en seco, disparaba una **esquina falsa** (y en modo
color el rumbo de referencia avanzaba 90 grados: derecho a la pared) o metia
una reversa en pleno paso, con el volante girado hacia el pilar. Ahora,
mientras hay maniobra (pilar mandando con peso >= 0,5, al costado o
saliendo), lo que cuenta como "muro delante" es la pared DE FRENTE
identificada por su orientacion (`frontal_mm`, que ya descuenta el
giroscopio); el pasillo crudo no frena, no abre esquinas ni dispara el
escape. La linea del piso y la pared de frente si abren la esquina.

**La red: un pilar EN el corredor dispara la reversa.** Si un pilar sigue
dentro del corredor de las ruedas a menos de `pilar_parar_mm` (180), el
esquive no lo ha librado: el navegador hace un escape marcha atras con el
volante al lado CONTRARIO al de paso (`pilar_escape_dir_pct`, 45: con
Ackermann la reversa hace girar el morro hacia el hueco), hasta que el pilar
queda a `pilar_salir_mm` o fuera del corredor, y vuelve a intentarlo. Si esto
salta a menudo, el esquive empieza tarde o va rapido; no es el modo normal de
pasar.

**Los delimitadores magenta del cajon.** Son muros de 200x20x100 contra la
pared EXTERIOR (reglas 13.7 a 13.9), y de canto miden 20 mm en la imagen. Se
rodean con medio ancho `magenta_semi_mm` (110: medio muro de 200 mas un poco),
no con el de un pilar de 50. El lado sale del SITIO: si el perfil del muro
dice que por un lado no cabe el carro, por el otro; si cabe por los dos, por
el que tenga mas hueco; si no se ve pared, por el INTERIOR de la pista
(horario: por su derecha; antihorario: por su izquierda); y sin sentido
conocido, por donde menos cruce la trayectoria. Una vez decidido, fijo, como
con las señales.

**Lo unico que hay que medir: `geometria.radio_giro_mm`.** Es lo que
convierte la curvatura en % de volante. Carro en manual con la direccion a
tope, una vuelta completa despacio, y el DIAMETRO del circulo que dibuja el
centro del carro, entre dos. Tipico entre 450 y 800; el perfil trae 550. Si
esta medido de menos el carro se queda corto al apartarse (sube `k_arco`); de
mas, se abre demasiado. El simulador comprueba que con un error del 35 % en
cualquier sentido el pilar se sigue pasando.

**Lo que dice el video y la web.** El circulo `arco` es el punto de la linea
de paso al que se mira: lejos del pilar mientras el carro tiene que
desplazarse, casi sobre el eje cuando ya va por la linea. `AL COSTADO (rojo a
-80mm, holgura 45mm) hacia la izquierda max 22%` es la regla del costado en
accion; `SALIENDO del paso 0.6s`, la salida; `EN EL CORREDOR a 210mm`, la
red a punto de saltar; `(tope de desvio: no abrirse mas)`, que el carro ya va
`desvio_max_esquive_deg` cruzado y solo se le deja enderezar.

**Lo que la fisica no deja.** Con radio de giro de 55 cm y un carro de 20 de
ancho: cambiar 20 cm de carril necesita unos 70 cm de recta y 35 cm, casi un
metro (dos arcos a tope); un pilar en el medio del carril que aparece a 65 cm
del morro se pasa, pero cruzado y rozando; dos señales de lado contrario a
menos de 1,2 m no dan para la S (la cola tarda 25 cm en librar la primera);
y un pilar a 30 cm de la pared del lado de paso deja 7 cm para repartir entre
el y la pared. Nada de eso lo arregla un parametro: lo arregla ir despacio
(`vel_esquive`) y empezar a apartarse en cuanto se ve el pilar
(`mandar_desde_mm`).

```bash
python3 tools/selftest_esquive.py              # 15 escenarios, con la config del reto
python3 tools/selftest_esquive.py --traza rojo # y la trayectoria tick a tick
```

Los escenarios: rojo y verde en cinco posiciones del carril, horario y
antihorario; carril de 600; el pilar que aparece a 65 y 50 cm de frente;
salir de la curva 30 grados torcido con el pilar delante; dos señales de
color distinto (la S); el delimitador magenta en los dos sentidos; el
detector perdiendo el 30 % de los frames; sin giroscopio; el radio de giro
mal medido; los valores por defecto del esquema; y el control que reproduce
el choque de antes. Cada uno mide colision, lado al pasar el morro, holgura
minima y si toco una pared. Va incluido en `tools/selftest.py`.

| Parametro | Valor | Que es |
|---|---|---|
| `geometria.radio_giro_mm` | 550 | MEDIRLO: radio a tope de direccion |
| `obstaculos.modo` | arco | pure pursuit a la linea de paso |
| `activar_desde_mm` / `mandar_desde_mm` | 1500 / 1100 | desde donde influye / manda del todo |
| `peso_max` | 1.0 | el centrado se calla mientras se esquiva |
| `dir_max_pct` | 85 | el arco pide a tope solo si hace falta |
| `mirada_mm` / `mirada_min_mm` | 600 / 350 | mirada sobre la linea, larga lejos y corta cerca |
| `vel_esquive` | 40 | ver un pilar es frenar |
| `margen_mm` / `margen_pared_mm` | 70 / 30 | aire al pilar / a la pared |
| `costado_desde_mm` / `costado_giro_max_pct` | 120 / 25 | el paso al costado y su tope |
| `recuperar_ms` | 1000 | la salida |
| `pilar_parar_mm` / `pilar_salir_mm` / `pilar_escape_dir_pct` | 180 / 500 / 45 | la red |
| `magenta_semi_mm` | 110 | medio delimitador |
| `ciego_max_ms` | 1500 | cuanto se sigue el pilar por estima sin verlo |

## El carro nunca debe circular en sentido contrario

Sintoma: tras chocar y retroceder varias veces, el carro se iba **en direccion
contraria**. Eso termina la ronda (regla 9.25.3).

La causa era una linea: al salir del escape se hacia `rumbo_objetivo = yaw`, o
sea, **el carro adoptaba como "rumbo bueno" aquel en el que se hubiera quedado
mirando** despues de maniobrar. Si acababa mirando hacia atras, esa pasaba a
ser su recta. Y el giro de rescate elegia lado "por donde haya mas hueco", que
tambien podia dejarlo encarado al reves.

Ahora hay un `rumbo_recta` que **solo cambia en las curvas de verdad** (+-90) y
que ningun escape puede tocar:

- al salir del escape se vuelve a el, no al yaw de la maniobra;
- el giro de rescate va hacia el lado que **acerca** a la recta;
- y si el rumbo se aleja mas de `desvio_max_deg` (110), se fuerza un rescate de
  rumbo para volver.

Ademas, un giro de rescate **ya no cuenta como esquina**: antes cada choque con
maniobra sumaba una al marcador de vueltas.

## Las patas INT del TCS y del MPU (cruzar las lineas rapido)

Las dos son **opcionales**: si no estan cableadas el firmware lo detecta solo
y sigue por sondeo. Lo dice la pestaña Calibrar, sensor por sensor.

```
TCS34725  INT -> GPIO 19      open-drain, activo BAJO (usa el pull-up interno)
MPU6050   INT -> GPIO 18      push-pull,  activo ALTO
```

No hacen lo mismo, y conviene tenerlo claro:

| | que es | que aporta |
|---|---|---|
| MPU6050 INT | *data ready*: un pulso por muestra (200 Hz) | el yaw se integra con el **dt real entre flancos**, en microsegundos, en vez de con el reloj de la tarea. Ese jitter del planificador se integraba directamente en el rumbo. |
| TCS34725 INT | *umbral* del canal claro (AILT/PERS) | un flanco **enganchado** en el instante en que el suelo se oscurece: el borde de la linea. Aunque la tarea llegue tarde, el cruce no se pierde y se sabe cuando fue. |

**El INT del TCS no hace que el sensor integre mas rapido.** Eso lo fija
`tcs.atime`: a 24 ms (246) una linea de 2 cm cruzada a 0.5 m/s deja 1 o 2
lecturas; a 2.4 ms (255) deja unas 16. Si quieres mas muestras por linea, baja
`atime` — y entonces **repite la calibracion del TCS**, porque entran menos
cuentas y todos los valores absolutos cambian (quiza tambien haya que subir
`gain`). Lo unico que no se estropea es el umbral de la interrupcion, porque
va en **porcentaje del nivel de piso** (`tcs.int_umbral_pct`), que el ESP32
aprende solo mientras rueda.

Detalles de la implementacion, por si hay que tocarla:

- Los ISR no tocan el I2C (Wire no es seguro dentro de una interrupcion):
  apuntan `micros()` y despiertan a `tareaSensores` con un aviso; la tarea es
  la que lee.
- La tarea ya no va a reloj fijo: espera el aviso con un **latido de 5 ms** de
  respaldo, asi que sin patas cableadas se comporta como antes.
- El TCS se autotestea al detectarlo: se le pone una ventana imposible, con lo
  que la pata TIENE que activarse; si no baja, se marca como no cableada.
- Si los flancos del MPU dejan de llegar (cable suelto), a los 250 ms vuelve
  al sondeo y lo avisa por el log.

## El sensor de color y la trampa del c_min

Sintoma real en pista: el TCS contaba naranjas y **nunca una azul**, asi que
ningun par se completaba ("pares a medias" subiendo en la telemetria).

Habia dos causas, y la segunda es la que de verdad mordia:

1. **Umbrales absolutos con margen ridiculo.** Sobre la linea azul el sensor
   dio C=678 R=186 B=284, o sea ratios r=70 y b=107. El piso blanco da ~85 en
   los dos. El azul solo sacaba 22 puntos al blanco en su propio canal, y el
   umbral por defecto estaba en 110: fallaba por tres unidades. Ahora el
   discriminador es la **diferencia** b-r (+37 en el azul contra ~0 en el
   blanco), que separa mucho mejor y no depende de la luz. Los umbrales
   absolutos se quedan como reja holgada, no como criterio.

2. **`c_min` por encima del claro de la linea.** El perfil guardado tenia
   `c_min=842`, sacado del 25 % de un blanco muy brillante; pero una linea de
   color ABSORBE luz y devuelve mucho menos claro (678, un 20 % del blanco).
   La lectura se descartaba antes de mirar el color. Ahora "Sobre blanco" usa
   el 8 %, y **muestrear una linea baja el `c_min`** si hace falta para que esa
   linea entre.

3. **Arreglar el codigo no arregla los perfiles ya guardados.** Los dos puntos
   de arriba se corrigieron en la regla de calibracion, pero los perfiles del
   8 de septiembre seguian llevando `c_min=842` dentro, porque un perfil
   guardado es una foto de los numeros de aquel dia: mientras no se vuelva a
   muestrear, el carro sale a pista con el valor viejo. Por eso el selftest
   comprueba ahora **el perfil ACTIVO de `config/params.json`**, no solo la
   funcion: la lectura real de la linea azul tiene que clasificarse como azul
   con los umbrales que el carro va a usar de verdad.

4. **La muestra a medio borde, que es la unica que hay con el carro andando.**
   El sensor integra 24 ms y en ese rato el borde de la linea se mueve, asi
   que la ventana mezcla linea y piso. Y la mezcla no es mitad y mitad en los
   ratios: el piso blanco devuelve cinco veces mas luz que la linea azul, asi
   que **domina la lectura y aplasta la diferencia b-r**. Con la ventana un
   75 % sobre la linea, esa diferencia cae de 36 a 14 puntos: con
   `azul_dif_min` en 18 la muestra se perdia aunque el sensor estuviera encima
   de la linea. De ahi el "no ve el azul en movimiento" y de ahi que las rejas
   absolutas (`azul_b_min`) tengan que quedar POR DEBAJO del nivel del blanco
   (~85) y no por encima. El naranja aguantaba porque su diferencia de partida
   es mucho mayor, y por eso el mismo fallo no se notaba en horario.

   Los tres remedios, por orden de coste: bajar `azul_dif_min` y las rejas
   (gratis), encender `lineas.frenar_ante_linea` para llegar mas despacio a la
   linea (gratis, la camara ya la ve venir) y bajar `tcs.atime` para tener mas
   muestras por linea (obliga a repetir la calibracion del TCS, porque cambian
   todos los valores absolutos, `c_min` incluido).

En la pestaña Calibrar se ve la diferencia b-r, lo que dice el ESP32 y **lo
que diria la Pi con los umbrales de ahora**. Si no coinciden, el ESP32 tiene
firmware viejo: vuelve a subir `code/esp32_carro`.

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

## El boton y el LED (el start del reglamento)

En competencia el robot se coloca en la pista y arranca con **una sola
pulsacion**: no hay teclado, ni pantalla, ni web, ni cable. El boton ARMAR de
la interfaz no sirve para eso, asi que el carro lleva **un pulsador**, y es el
mismo para las dos cosas, como el start/stop de un cronometro:

| Pulsacion | Con el carro... | Que hace |
|---|---|---|
| corta | parado | **ARMA** y arranca la ronda |
| corta | andando | **DESARMA**: parada de emergencia |
| corta | ronda terminada | arranca una **ronda nueva** |
| larga (`largo_ms`, 3 s) | cualquiera | para y **apaga la Pi**, solo si `botones.apagar_con_larga` esta encendido |

**Cuelga del ESP32, no de la Pi.** El firmware (`code/esp32_carro/botones.h`)
lo lee a 100 Hz con antirrebote y manda su nivel en la trama de sensores, 40
veces por segundo; `src/botones.py` decide que significa, porque es el lado
que sabe si el carro estaba armado. Dos cables y ninguna resistencia:

```
GPIO13 ---[ pulsador ]--- GND      (pull-up interno del ESP32)
GPIO2  ---> LED azul de a bordo    (el de la placa; no hay que cablear nada)
```

El 13 no es pin de arranque, asi que un pulsador pisado al encender no impide
el boot.

### El LED azul dice en que estado esta el carro

Sin web es lo unico que informa desde fuera, asi que lleva cuatro patrones
que se distinguen de un vistazo a dos metros:

| LED | Estado |
|---|---|
| **fijo encendido** | ARMADO: el carro puede salir corriendo AHORA |
| **un destello corto** cada 1.6 s | listo y desarmado, con la Pi hablando |
| **parpadeo rapido** (5 Hz) | sin ordenes de la Pi (failsafe): programa caido, cable suelto o Pi arrancando todavia |
| **dos destellos** y pausa | cortado por el pulsador (emergencia enclavada) |

Los patrones son 16 ranuras de 100 ms leidas del reloj, sin estado propio
(`bot::ledEncendido`), asi que da igual quien los pinte y cada cuanto. Si tu
placa tiene el LED en otro pin, se cambia `PIN_LED_ESTADO` en el `.ino` y ya.

### La emergencia corta en el ESP32, no en la Pi

Es la razon de peso para colgar el pulsador del ESP32. Con un solo boton el
firmware no sabe que significa una pulsacion... salvo en el caso que importa:
**si el carro esta ARMADO, lo unico que puede querer decir es parar** (nadie
pulsa para armar lo que ya anda). En ese caso el firmware **enclava un corte
local** y para el motor en el siguiente tick de 10 ms: no espera el viaje de
ida y vuelta por el serial (~50 ms) y no depende de que el programa de la Pi
este sano. Con el pulsador en la Pi, un programa colgado mandando "adelante"
dejaba la emergencia sin efecto hasta que saltara el failsafe de 300 ms.

Con el carro parado la pulsacion no corta nada: esa es la de arrancar, y la
interpreta la Pi. El enclavamiento se suelta cuando la Pi acusa recibo (manda
un mando desarmado) **y** el dedo ya no esta encima; el operador no tiene que
hacer nada raro: vuelve a pulsar y el carro sale.

### Nivel, no contador (al reves que las lineas)

Los cruces de linea viajan como contadores porque una linea se cruza en 40 ms
y perderla es perder una esquina. Un dedo, en cambio, aguanta el pulsador
100-300 ms = 4-12 tramas: el nivel llega de sobra. Y hay una razon de
seguridad que decide el empate: un contador **guarda pulsaciones pendientes**,
asi que un corte del serial con una pulsacion sin entregar podria arrancar el
carro solo al reconectar. Con nivel, lo que se pierde no se ejecuta: el fallo
es "no pasa nada", que delante de un juez es el fallo bueno.

### Lo demas que hay que saber

- **Hay que subir el firmware nuevo** (version 4): la trama de sensores paso
  de 14 a 15 bytes. La Pi acepta las dos, asi que con firmware viejo todo
  sigue funcionando *menos* el pulsador; se nota en `botones.frescos` a falso
  y en que no pasa nada al pulsar.
- **Se prueba sin cablear nada**: `/api/cmd?boton=corta` (o `?boton=larga`)
  inyecta la pulsacion por el mismo camino que la fisica, antirrebote
  incluido. Funciona igual en el PC con `--simulado`, y sigue valiendo aunque
  `botones.activo` este apagado (lo manda quien ya podia armar el carro desde
  la web).
- **El pulsador pisado cuando la Pi empieza a mirarlo queda mudo** hasta que
  se le ve suelto una vez: dedo apoyado, cable al reves o boton pegado no
  pueden lanzar la ronda solos.
- **Si el enlace se cae, se olvida el estado del pulsador**, y al volver tiene
  que verse suelto otra vez. Sin eso, un nivel viejo congelado en "pisado"
  seria una pulsacion larga fantasma, y la reconexion con el dedo encima, un
  arranque solo.
- **Tiempo muerto** (`repeticion_ms`, 800 ms): un doble toque involuntario
  justo despues del start desarmaria la ronda recien empezada.
- Todo el grupo `botones` se aplica **en caliente**. El pin no esta ahi a
  proposito: es del firmware, como los del motor.

## Correr sin la web: el demonio (SSH y VNC)

En competencia no hay nadie mirando la web y el stream MJPEG cuesta CPU y
bateria, asi que la ronda se corre con `--sin-web`. `piloto.sh` lo lanza en
segundo plano, en su propia sesion, con la salida a un log:

```bash
./piloto.sh arrancar        # segundo plano, SIN web (competencia)
./piloto.sh estado          # si corre, desde cuando y con que argumentos
./piloto.sh log -f          # el registro en directo
./piloto.sh parar           # SIGTERM: emergencia, servo al centro, cierre
./piloto.sh web             # igual pero CON la web, para depurar
./piloto.sh consola         # en primer plano (Ctrl+C para salir)
./piloto.sh servicio        # arranque automatico al encender (systemd)
```

**Por SSH**, desde el portatil y con el carro sin pantalla:

```bash
ssh pi@carrito.local '~/piloto/piloto.sh arrancar'
ssh pi@carrito.local '~/piloto/piloto.sh estado'
ssh pi@carrito.local '~/piloto/piloto.sh parar'
```

El proceso queda en su propia sesion (`setsid`), con la entrada en
`/dev/null` y la salida al log: el comando vuelve enseguida y **cerrar el SSH
no se lleva el carro por delante**.

**Por VNC**: los mismos comandos en cualquier terminal del escritorio. Para
mirar la telemetria desde el navegador de la propia Pi, arranca con
`./piloto.sh web` y abre `http://localhost:8080/`.

Detalles que importan:

- `parar` manda **SIGTERM**, que `main.py` atiende: parada de emergencia,
  servo al centro y cierre del puerto. Matar a lo bruto deja al ESP32 con la
  ultima orden hasta que salte su vigilante de 300 ms.
- El log vive en `~/.local/state/piloto/piloto.log` y rota solo a los 8 MB.
- `./piloto.sh servicio` instala `piloto.service` (systemd) para que el
  piloto arranque solo al encender la Pi, **sin web** y con los grupos
  `dialout` (el serie del ESP32) y `video`. A partir de ahi manda systemd:
  `sudo systemctl start|stop|status piloto` (el script lo detecta y avisa en
  vez de pelearse con el).
- **Sin web es a proposito**: el servicio es el arranque de competencia, donde
  el reglamento no admite nada inalambrico mientras el carro corre y el unico
  mando tiene que ser el pulsador. Si en banco quieres la telemetria desde el
  encendido, instala la unidad con `./piloto.sh servicio web` (o
  `PILOTO_WEB=1 ./piloto.sh servicio`) y reinicia el servicio: queda en
  `http://<ip-de-la-pi>:8080/`. **Tiene que decidirse al instalar la unidad**,
  porque `--sin-web` es un interruptor de `main.py` y no hay ningun argumento
  que lo deshaga: por `PILOTO_ARGS` no se puede. Vuelve a `./piloto.sh
  servicio` (sin `web`) antes de competir.
- Variables: `PILOTO_ARGS` (argumentos fijos, p.ej. `--perfil pabellon`),
  `PILOTO_PYTHON`, `PILOTO_LOG`, `PILOTO_ESTADO`.

---

## La web (todo en caliente, nada requiere reiniciar)

- **Carrera**: video anotado, ARMAR/PARAR, modo, sentido, conteo, ajustes
  rapidos y telemetria.
- **Manual**: joystick tactil (y flechas/WASD). Hombre muerto: si el joystick
  deja de refrescar 400 ms, el carro se para. Para rescates tras choque.
- **Colores**: igual que el calibrador viejo pero en el movil: clic sobre el
  objeto (con acumular para cara iluminada+sombra), sliders HSV y filtros,
  vista de mascara y de piso, 5 perfiles rotativos.
- **Ajustes**: TODOS los parametros, generados del esquema con su descripcion
  (añadir un parametro en `src/params.py` lo hace aparecer solo). 5 perfiles
  rotativos ("casa", "pabellon"...). Cada numero lleva **slider + casilla para
  escribir el valor exacto**, y los que admiten "automatico" (exposicion,
  balance de blancos, camara trasera) llevan un **boton AUTO**: ese valor es
  un centinela (-1) que con un slider de -14 a 1000 seria imposible de clavar.
  Al apagar AUTO cae en un valor manual razonable, listo para afinar.
- **Calibrar**: focales por clic, giroscopio (calibrar con el carro QUIETO),
  y TCS: poner el sensor sobre blanco/naranja/azul y pulsar el boton; los
  umbrales se calculan y viajan al ESP32.
- **Sistema**: enlace, contadores de tramas, log, reintento de sensores I2C.

## Parametros que se tocan en pista

| Parametro | Que hace |
|---|---|
| `limites.vmax` | Tope duro de PWM. El freno de mano de todas las pruebas. |
| `botones.activo` | Hacer caso al pulsador del ESP32. Encendido de fabrica: sin web es el unico mando. |
| `navegacion.kp` / `kd` | PD del centrado: kp si corrige lento, kd si oscila. |
| `navegacion.girar_bajo_mm` | Pasillo con el que asume esquina y gira. |
| `navegacion.ttc_min_s` | Freno por tiempo-hasta-el-muro (anti-inercia). El que mas se toca. |
| `navegacion.retardo_giro_ms` | Espera del pre-giro (ruedas traseras pasan el canto interno). |
| `navegacion.apertura_pct` | Cuanto se abre (contra-direccion) antes de cortar la esquina. |
| `carrera.parada_ms` | Cuanto avanza tras la ultima esquina antes de pararse en meta. |
| `muro.k_transicion` | Filas no-piso seguidas para creer el muro (sube si hay muros fantasma). |
| `navegacion.bloqueo_esquina` | Anti-bucle en las curvas. Dejalo encendido. |
| `giro2t.activo` | Giro de 90 en dos tiempos: mas lento, ve todos los obstaculos. |
| `giro2t.frac_avance` | Cuanto del giro se hace hacia adelante antes de retroceder. |
| `esquina_color.activo` | Modo de esquina por color: un solo color cuenta y el giro cede al pilar. |
| `esquina_color.dir_pct` / `vel_max_pct` / `vel_min_pct` | Cuanto dobla hacia adentro y a que velocidad (variable con el pasillo). |
| `esquina_color.refractario_ms` | Ventana en la que la misma linea pisada otra vez no cuenta. |
| `esquina_color.contar_giro_sin_linea` | Que el giro de 90 cuente la esquina que el TCS no vio. Dejalo encendido: sin el, una linea perdida deja la ronda sin parada en meta. |
| `navegacion.linea_dispara_esquina` | **Encendido siempre en modo color.** Es lo que hace que la linea GIRE y no solo cuente. Apagado, el carro cruza la linea y sigue de largo hasta la pared. |
| `navegacion.tras_giro_ms` | Tiempo tras una curva en el que la vision no puede abrir otra. Subelo si el carro encadena dos giros en la misma esquina. |
| `esquina_color.max_ms` / `dir_pct` | Cuanto tiempo tiene la curva y cuanto volante puede meter. Si `giros_vencidos` sube en la web, la curva no cabe: mas `max_ms` o mas `dir_pct`. |
| `navegacion.apertura_pct` | Cuanto se abre hacia la pared EXTERNA antes de doblar. Con radio de sobra en el carril, dejalo bajo. |
| `navegacion.girar_bajo_mm` | Distancia a la que la vision dispara la curva. **No puede bajar de `parar_bajo_mm` + lo que consume la maniobra** (~282 mm con esta geometria) o el escape salta en mitad de todas las curvas. |
| `navegacion.estrategia` | `pared` mide la distancia al muro interno; `centrado` equilibra huecos. Si el carro se acerca a las paredes en recta, es esto. |
| `navegacion.pared_objetivo_mm` | Distancia al muro interno. Ponla **igual al radio de giro** (`dir_pct`) y la curva sale exacta. |
| `escape.factor_en_giro` | Cuanto se relaja el escape dentro de una curva comprometida. Subelo a 1 para volver al comportamiento de antes. |
| `tcs.c_min` / `muestras_min` | Si el carro "ve color" antes de tiempo (sombra del zocalo), sube `c_min` — sin pasar del claro de la linea— o sube `muestras_min`. |
| `esquina_color.ignoradas_para_invertir` | Lineas del otro color ignoradas, sin contar ni una esquina, antes de aceptar que el sentido salio al reves y corregirlo. |
| `carrera.vueltas` / `autostop` | 3 y encendido para una ronda de verdad. Revisalos antes de cada tanda: un perfil de pruebas con 1 vuelta o con el autostop apagado parece "el carro no para". |
| `tcs.c_min` | Claro minimo para clasificar. **Tiene que quedar por debajo del claro de la LINEA**, no del piso: es la trampa que dejo al carro sin ver las azules. |
| `tcs.azul_dif_min` / `azul_b_min` | Lo que decide el azul con el carro en marcha, cuando la muestra mezcla linea y piso. |
| `navegacion.reanclar_rumbo` | Adopta la recta real cuando el carro doblo una esquina que el codigo no registro. Dejalo encendido. |
| `lineas.frenar_desde_mm` / `vel_linea_pct` / `frenar_tras_ms` | Freno ante linea vista por la camara, para que el TCS no se la salte. |

## Estructura

```
piloto/
├── main.py                 arranque; --simulado, --imagen, --vmax, --puerto, --sin-web
├── piloto.sh               demonio sin web (arrancar/parar/estado/log; SSH y VNC)
├── config/                 params.json y colors.json (se crean solos; 20 perfiles c/u)
│   └── obstaculos/         los del Reto con Obstaculos (main.py --reto obstaculos)
├── src/
│   ├── geometria.py        pixeles <-> mm sobre el suelo; horizonte; corredor
│   ├── muro.py             perfil robusto + rectas clasificadas + esquinas
│   ├── vision.py           mascaras HSV y deteccion de objetos (del reconizer)
│   ├── color_config.py     perfiles de color (mismo formato que el reconizer)
│   ├── params.py           esquema autodocumentado de parametros + perfiles
│   ├── lineas.py           sentido / esquinas / vueltas + zona; modo par o por color
│   ├── navegacion.py       RECTO / PRE_GIRO / GIRO / GIRO_2T / GIRO_COLOR / ESCAPE
│   ├── carrera.py          director de la ronda (3 vueltas y parada en meta)
│   ├── obstaculos.py       identifica la señal (rojo/verde/magenta), el lado y el ARCO para pasarla
│   ├── protocolo.py        trama binaria v2 (gemela de protocolo.h)
│   ├── enlace.py           hilo serie; sensores del ESP32 -> eventos
│   ├── botones.py          el boton del ESP32: corta/larga -> armar/desarmar
│   ├── robot.py            el nucleo que une todo
│   ├── dibujo.py           overlay del video
│   ├── servidor.py         http.server + MJPEG
│   └── web/index.html      la interfaz
└── tools/
    ├── selftest.py         409 pruebas sin hardware
    ├── selftest_obstaculos.py  el lado de paso, con pilares sinteticos
    ├── selftest_esquive.py     el carro PASA el pilar: simulacion cinematica
    ├── diagnostico_pilares.py  por que NO se detecta ese pilar
    └── crear_config_obstaculos.py  siembra config/obstaculos/
```

## Notas practicas (heredadas a golpes)

- **Congela exposicion y balance de blancos antes de calibrar colores**
  (`camara.exposicion` / `balance_blancos`; -1 = automatico).
- **No dejes que el piso salga quemado.** Medido sobre capturas reales: con
  el blanco a V=244 las lineas del piso bajan a saturacion 15-36 y NINGUN
  umbral HSV las distingue del piso; con V=161 el naranja llega a 255 de
  saturacion. Si las lineas no se detectan, el problema es la exposicion, no
  el rango de color. (El TCS por contacto no depende de esto: por eso es la
  fuente fiable para contar.)
- El giroscopio se calibra solo al detectarse (con el carro quieto en la
  preparacion) y hay boton para repetirlo. Sin calibrar deriva 1-3 grados/s.
- Los cruces de linea del TCS viajan como CONTADORES: perder tramas no pierde
  cruces.
- Regla 9.9 del reglamento: en competencia NO se puede calibrar despues de la
  revision tecnica. Calibrar colores/TCS ANTES de entregar el carro.
- En el Open Challenge esta prohibido tocar el muro perimetral exterior:
  `parar_bajo_mm` y el escape existen para eso; mejor conservador.
- La camara trasera para el estacionamiento tiene el hueco reservado
  (`camara.indice_trasera`); no esta implementada todavia.
