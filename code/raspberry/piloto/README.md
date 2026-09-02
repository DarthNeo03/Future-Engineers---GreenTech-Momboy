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
python tools/selftest.py            # 133 pruebas, sin hardware
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
4. **Conteo**: solo suma cuando se cruza el PAR COMPLETO de lineas y **en el
   orden correcto**. Las cuatro esquinas de una vuelta se cruzan siempre igual
   (horario: naranja y luego azul), asi que el orden es una comprobacion de
   coherencia gratis: un par al reves o es basura o el carro se dio la vuelta.
   Uno suelto se descarta sin contar; dos seguidos invierten el sentido. Si se
   pierde una linea, el par caduca y esa esquina la cuenta el giro de 90.
   4 esquinas = vuelta.
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
│   ├── lineas.py           sentido / esquinas / vueltas + zona (dentro de la curva)
│   ├── navegacion.py       RECTO / PRE_GIRO / GIRO / GIRO_2T / ESCAPE
│   ├── carrera.py          director de la ronda (3 vueltas y parada en meta)
│   ├── obstaculos.py       esquive basico rojo/verde
│   ├── protocolo.py        trama binaria v2 (gemela de protocolo.h)
│   ├── enlace.py           hilo serie; sensores del ESP32 -> eventos
│   ├── robot.py            el nucleo que une todo
│   ├── dibujo.py           overlay del video
│   ├── servidor.py         http.server + MJPEG
│   └── web/index.html      la interfaz
└── tools/selftest.py       133 pruebas sin hardware
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
