# Piloto WRO 2026 — Open Challenge + deteccion de obstaculos

Sistema nuevo del carro (GreenTech Momboy). Sustituye a `reconizer` tomando lo
que funcionaba (navegacion centrada, calibracion por clic, enlace binario con
el ESP32) y arreglando lo que fallaba (paredes con brillo, objetos detectados
por encima del muro, conteo de esquinas).

```
Raspberry Pi 5 (vision + decisiones)  <-USB->  ESP32 (motor, servo, MPU6050, TCS34725)
```

El firmware del ESP32 vive en `code/esp32_carro/` (protocolo v2: ahora lee el
MPU6050 y el TCS34725 por I2C y manda yaw + cruces de linea a la Pi).

---

## Puesta en marcha

**Raspberry Pi 5** (o PC Windows para probar sin carro):

```bash
python3 -m venv .venv && source .venv/bin/activate    # en Windows: .venv\Scripts\activate
pip install -r requirements.txt
python tools/selftest.py            # 295 pruebas, sin hardware
python main.py                      # camara + ESP32 + web
python main.py --simulado           # sin ESP32 (pruebas en el PC)
python main.py --imagen foto.jpg    # sin camara, sobre una foto
python main.py --vmax 90            # tope de PWM solo para esta prueba
```

**ESP32**: abrir `code/esp32_carro/esp32_carro.ino` en el IDE de Arduino (los
5 archivos en la misma carpeta) y subir. Sin librerias externas. Al subir
firmware nuevo, subirlo ANTES de correr `main.py`.

Web de depuracion: **http://carrito.local:8080/** (o `http://<ip>:8080/`).
El carro **arranca desarmado**: pulsar ARMAR. En modo auto, ARMAR arranca la
carrera (cronometro + conteo).

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
   reglamento). Tope de 3 minutos.

**Obstaculos (basico, apagado por defecto)**: `obstaculos.activo` en la web.
Pilar rojo se pasa por la derecha, verde por la izquierda; el punto de paso se
calcula en mm reales y se recorta al hueco libre del perfil.

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
en meta. El giro de 90 completado ya no cuenta esquina (salvo
`contar_giro_sin_linea`): si el TCS se pierde una linea, esa esquina no suma;
mejor perder una que sumar una. La vision (pasillo cerrandose, pared de
frente) sigue pudiendo disparar este mismo giro por si el TCS falla
(`vision_dispara`); tambien se le puede quitar el voto.

Ojo: si el TCS se pierde la primera naranja y ve la azul, el sentido sale al
reves toda la ronda. Es el precio de usar solo el TCS; con
`carrera.sentido` forzado desde la web no pasa, y ahi cuenta el color de ese
sentido aunque la primera linea vista fuera la otra.

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

### Valores recomendados

Los que habia en pista estaban subidos para compensar los fallos de arriba;
con estos arreglados conviene bajarlos:

| Parametro | En pista | Recomendado | Por que |
|---|---|---|---|
| `k_dir` | 2.83 | 1.4 | ya no hace falta forzar |
| `margen_mm` | 261 | 90 | el margen no arreglaba la cola; el compromiso si |
| `peso_max` | 1.0 | 0.8 | con 1.0 el muro deja de contar |
| `activar_desde_mm` | 4000 | 1600 | 4 m es media pista: el pilar mandaba desde lejisimos |

Con los valores viejos + los arreglos, el esquive queda topado en 55 %; con los
recomendados sube progresivo de 1 % a 37 % segun se acerca.

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
| `navegacion.reanclar_rumbo` | Adopta la recta real cuando el carro doblo una esquina que el codigo no registro. Dejalo encendido. |
| `lineas.frenar_desde_mm` / `vel_linea_pct` / `frenar_tras_ms` | Freno ante linea vista por la camara, para que el TCS no se la salte. |

## Estructura

```
piloto/
├── main.py                 arranque; --simulado, --imagen, --vmax, --puerto
├── config/                 params.json y colors.json (se crean solos; 5 perfiles c/u)
├── src/
│   ├── geometria.py        pixeles <-> mm sobre el suelo; horizonte; corredor
│   ├── muro.py             perfil robusto + rectas clasificadas + esquinas
│   ├── vision.py           mascaras HSV y deteccion de objetos (del reconizer)
│   ├── color_config.py     perfiles de color (mismo formato que el reconizer)
│   ├── params.py           esquema autodocumentado de parametros + perfiles
│   ├── lineas.py           sentido / esquinas / vueltas + zona; modo par o por color
│   ├── navegacion.py       RECTO / PRE_GIRO / GIRO / GIRO_2T / GIRO_COLOR / ESCAPE
│   ├── carrera.py          director de la ronda (3 vueltas y parada en meta)
│   ├── obstaculos.py       esquive rojo/verde con compromiso de paso
│   ├── protocolo.py        trama binaria v2 (gemela de protocolo.h)
│   ├── enlace.py           hilo serie; sensores del ESP32 -> eventos
│   ├── robot.py            el nucleo que une todo
│   ├── dibujo.py           overlay del video
│   ├── servidor.py         http.server + MJPEG
│   └── web/index.html      la interfaz
└── tools/selftest.py       295 pruebas sin hardware
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
