"""
obstaculos.py — Identificar las señales de transito y pasarlas por el lado que
manda el reglamento.

REGLA DEL JUEGO (reglamento 2026, punto 9.19)
El pilar ROJO se pasa por su DERECHA; el VERDE por su IZQUIERDA. "Derecha" e
"izquierda" son las del VEHICULO segun avanza, no las del tapete: el Apendice
A lo repite para la conduccion de atras hacia adelante, asi que es el marco del
carro. Por eso NO hay que invertir nada al cambiar de sentido — el mismo pilar
fisico se pasa por un lado distinto en horario que en antihorario, y eso ya
sale solo de trabajar en el marco del carro. Aun asi queda el interruptor
`invertir_en_antihorario` por si en tu pista se interpreta al reves: apagado
por defecto.

Matiz que se olvida: "pasar por la derecha del pilar" NO es "girar a la
derecha". Si el pilar esta a la izquierda del carro, el punto de paso puede
quedar a la izquierda del centro de la imagen.

=========================================================================
POR QUE EL CARRO SE LLEVABA EL PILAR POR DELANTE (lo que arregla el modo ARCO)
=========================================================================
El lado ya salia bien (ver mas abajo: veto del magenta, prueba de tamaño y
votacion). Lo que fallaba era el VOLANTE: el carro sabia por donde pasar y aun
asi chocaba. Tres causas, todas medibles en el simulador de
tools/selftest_esquive.py:

1. EL ESQUIVE PEDIA LA MITAD DE LO QUE HACE FALTA. El modo 'punto' convierte
   el angulo al punto de paso en direccion con una ganancia fija (k_dir) y una
   mirada minima. Pero un carro con direccion Ackermann no va "hacia donde
   apunta": describe un ARCO cuyo radio depende del angulo de las ruedas.
   Desplazarse 195 mm de lado en 700 mm de recorrido exige un radio de ~1,3 m;
   en 400 mm, de ~500 mm, que ya es el giro a tope. La ganancia fija pedia
   ~35 % donde la geometria pide 60-100 %.

2. EL CENTRADO PELEABA CONTRA EL ESQUIVE, Y GANABA. La direccion final era
   una media ponderada (peso 0,8 al pilar, 0,2 al centrado), pero el centrado
   del Open Challenge lleva kp=372: en cuanto el carro se desplaza 10 cm del
   medio del carril pide -70 % de volante. El 20 % de eso son -15 % que se
   restan a los +35 % del pilar, y el rumbo por giroscopio restaba otros -5 %.
   Quedaban +15 % efectivos: el carro se apartaba un poco... y se llevaba el
   pilar con el costado.

3. NADIE FRENABA. El pilar no aparece en el perfil del muro (metodo 'negro'
   solo ve negro), asi que la velocidad seguia siendo la de crucero (74 %)
   hasta el golpe. A 630 mm/s no hay volante que desplace el carro 20 cm en
   los ultimos 70 cm.

Y lo que se hace ahora (obstaculos.modo = 'arco'):

  * PURE PURSUIT SOBRE LA LINEA DE PASO. Se calcula la curvatura del arco que
    lleva al carro a la LINEA paralela al carril que pasa por el punto de paso
    (no al punto: al punto se llega cruzado, a la linea se llega paralelo). La
    curvatura se traduce a % de volante con el radio de giro real del carro
    (geometria.radio_giro_mm), que se mide con una cinta metrica. Cuanto mas
    cerca el pilar, mas corta la mirada y mas cerrado el arco: pide lo que la
    geometria pide, ni mas ni menos.
  * EL PILAR MANDA DE VERDAD (peso 1,0 dentro de mandar_desde_mm): el centrado
    y el rumbo se callan mientras se esquiva. El muro sigue contando por donde
    debe: el punto de paso se recorta al pasillo libre (_recortar) y si el
    pilar queda EN el corredor de las ruedas y demasiado cerca, el navegador
    hace un escape marcha atras en vez de chocar (Restriccion.bloqueo_mm).
  * SE FRENA: mientras un pilar manda, la velocidad baja a vel_esquive.
  * EL PILAR SE SIGUE CON EL GIROSCOPIO hasta que la COLA lo pasa. Con el yaw
    y la velocidad real se lleva la cuenta de donde queda el pilar aunque ya
    no se vea (sale por el canto de la imagen a ~30 cm), y mientras esta AL
    COSTADO se prohibe girar HACIA el: con direccion Ackermann, volver al
    centro del carril con el pilar a la altura de la rueda trasera es como se
    lo lleva. Antes esto se hacia por tiempo (el compromiso), que sigue de
    respaldo cuando no hay giroscopio.
  * LOS DELIMITADORES MAGENTA del cajon de estacionamiento son muros de 200 mm
    contra la pared exterior: se rodean por el lado en que hay sitio segun el
    perfil del muro y, si se conoce el sentido, por el interior. Su medio ancho
    es el de un muro de 200 (magenta_semi_mm), no el de un pilar de 50.

=========================================================================
POR QUE EL LADO SALIA MAL (lo que arregla la identificacion)
=========================================================================
El esquive de antes decidia el lado con una sola linea:

    lado = +1 si color == "rojo" si no -1

...y volvia a decidirlo DESDE CERO en cada frame, creyendose la etiqueta de
color tal cual salia del detector. Eso falla por tres sitios a la vez, y el
sintoma en pista es siempre el mismo: el carro esquiva, pero por el lado que
le da la gana.

1. EL MAGENTA SE CUELA COMO ROJO. Los delimitadores del cajon de
   estacionamiento son magenta, RGB(255,0,255) = tono 150 en la escala de
   OpenCV (0-179). El rango de rojo de la calibracion empezaba justo en 150,
   asi que TODO delimitador magenta se veia tambien como un pilar rojo y el
   carro intentaba pasarlo por su derecha. Y el magenta no es una señal: es un
   muro de 100 mm que no se puede mover (reglas 13.7 a 13.9). Aqui se vetan
   las detecciones rojas/verdes que caen encima de una mancha magenta, asi que
   el arreglo funciona AUNQUE el rango de rojo siga abierto.

2. CUALQUIER MANCHA DEL COLOR VALIA COMO PILAR. Una sombra rojiza en el
   zocalo, el reflejo de un pilar en el suelo o un trozo de pared con V baja
   entraban como "pilar" si pasaban los filtros de area y aspecto, que son
   filtros de IMAGEN y no saben de tamaños reales. Una señal mide 50x50x100 mm
   (regla 13.1): sabiendo a que distancia esta, se sabe cuanto TIENE que medir
   en pixeles. Lo que no mide eso, no es un pilar.

3. EL LADO PARPADEABA. Aunque la clasificacion falle solo en uno de cada
   cinco frames, el lado se recalculaba entero en cada uno: un frame el pilar
   era rojo y el objetivo se iba a la derecha, al siguiente era magenta-leido-
   como-verde y se iba a la izquierda. La media de eso es ir de frente contra
   el pilar. Ahora cada pilar se SIGUE entre frames, el color se decide por
   VOTACION y, una vez decidido, el lado QUEDA FIJO hasta que el pilar se
   pierde: pasar por el lado incorrecto termina la ronda (Apendice A, 5), asi
   que esta es la decision que menos derecho tiene a cambiar de opinion.

COMO SE PASA UN PILAR, EN CUATRO ACTOS
  1. APROXIMACION. Modo 'arco': arco hacia la linea de paso, con la mirada
     acortandose segun se acerca. Modo 'punto': se apunta al hueco con una
     ganancia fija y una mirada minima (el de antes). Modo 'borde': control
     visual sobre el canto del pilar.
  1b. A CIEGAS. La mascara de color parpadea y el pilar sale por el canto de
     la imagen antes de llegar a el. Se sigue conduciendo contra su posicion
     estimada (velocidad + giroscopio) hasta ciego_max_ms.
  2. AL COSTADO. El pilar ya paso el morro y todavia no ha pasado la cola: se
     mantiene el rumbo y se PROHIBE girar hacia el (Restriccion.no_girar).
  3. SALIDA. La cola lo paso: manda otra vez el centrado y el rumbo.

LO QUE HAY MAS ALLA DE LA LINEA DEL PISO NO ES DE ESTA RECTA
Las lineas naranja y azul marcan el limite de la seccion. Un pilar que se ve
por detras de ellas esta en el tramo SIGUIENTE. Si se le hace caso desde la
recta, el esquive tira del carro justo cuando hay que prepararse para la
curva; el carro se pega a la esquina interna y engancha el canto al girar.
Se descartan mientras se viene por la recta y cuentan al entrar en la curva.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from . import vision
from .geometria import Geometria
from .muro import PerfilMuro

# Lado por el que el VEHICULO pasa cada señal (regla 9.19).
#   +1 = el carro pasa por la DERECHA del pilar
#   -1 = el carro pasa por su IZQUIERDA
# El magenta NO esta aqui a proposito: no es una señal de transito y no manda
# ningun lado. Se rodea por donde haya sitio.
LADO_REGLAMENTO: Dict[str, int] = {"rojo": +1, "verde": -1}
SENALES = ("rojo", "verde")


def _norm_ang(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _al_carril(x: float, y: float, e: float) -> Tuple[float, float]:
    """Del marco del CARRO (x lateral +derecha, y adelante) al del CARRIL,
    cuando el carro apunta 'e' radianes a la derecha de la recta."""
    c, s = math.cos(e), math.sin(e)
    return x * c + y * s, -x * s + y * c


def _al_carro(x_l: float, y_l: float, e: float) -> Tuple[float, float]:
    """Inversa de _al_carril."""
    c, s = math.cos(e), math.sin(e)
    return x_l * c - y_l * s, x_l * s + y_l * c


@dataclass
class Restriccion:
    """Lo que el esquive le impone al navegador ADEMAS de la direccion.

    La direccion deseada y su peso van por el camino de siempre (la tupla que
    devuelve paso()); esto es lo que no cabe en una direccion:

      vel_max_pct   tope de velocidad (% de vmax) mientras un pilar manda,
                    aplicado en proporcion peso_vel (0..1). Ver un pilar es
                    frenar: a velocidad de crucero no hay volante que valga.
      bloqueo_mm    hay un pilar DENTRO del corredor de las ruedas a esta
                    distancia. El navegador lo trata como un muro para el
                    escape: si esta encima y no se ha librado, marcha atras.
      lado_bloqueo  por que lado habia que pasar ese pilar (+1 derecha). En
                    la reversa del escape el volante va al lado contrario para
                    que el morro quede apuntando al lado de paso.
      no_girar      +1 = limitado el giro a la DERECHA, -1 = a la izquierda.
                    Hay un pilar al costado, entre el morro y la cola: girar
                    hacia el es barrerlo con la rueda trasera. Hacia ese lado
                    el volante no pasa de tope_hacia_pct (0 = prohibido).
      tope_hacia_pct cuanto volante se deja hacia el pilar al costado. Sale de
                    la holgura con que se le esta pasando: con 5 cm de aire
                    se puede enderezar el rumbo sin tocarlo; con 1 cm, no.
      al_costado    hay un pilar entre el morro y la cola ahora mismo.
      recuperando   la cola acaba de dejar atras un pilar (recuperar_ms): el
                    carro esta pegado a una pared y algo cruzado, volviendo
                    al carril. El navegador sigue tratando el muro "de
                    frente" por su orientacion, no por el pasillo crudo, que
                    con el carro cruzado mide la pared de al lado.
    """
    vel_max_pct: Optional[float] = None
    peso_vel: float = 0.0
    bloqueo_mm: Optional[float] = None
    lado_bloqueo: int = 0
    no_girar: int = 0
    tope_hacia_pct: float = 0.0
    al_costado: bool = False
    recuperando: bool = False

    @property
    def activa(self) -> bool:
        return (self.vel_max_pct is not None or self.bloqueo_mm is not None
                or self.no_girar != 0)

    @property
    def maniobra(self) -> bool:
        """El carro esta en plena maniobra de esquive (pasando o saliendo)."""
        return self.al_costado or self.recuperando


@dataclass
class Candidato:
    """Una mancha de color que ya paso el filtro de "esto puede ser un pilar"."""
    color: str
    det: vision.Deteccion
    dist: float          # mm desde el MORRO
    dist_cam: float      # mm desde la lente (la que usa la geometria)
    lat: float           # mm, + a la derecha del eje del carro
    ancho_mm: float


@dataclass
class Pista:
    """Un pilar SEGUIDO entre frames.

    Existe para que el color -y por tanto el lado- no se vuelva a votar desde
    cero en cada imagen. Mientras el mismo pilar se siga viendo, los votos se
    acumulan; cuando hay bastantes, el lado se fija y ya no se mueve. Y cuando
    deja de verse, su posicion se sigue llevando por estima (velocidad y
    giroscopio) hasta que la cola del carro lo deja atras.
    """
    id: int
    votos: Dict[str, int] = field(default_factory=dict)
    color: str = ""
    lat: float = 0.0
    dist: float = 0.0          # mm desde el morro; negativo = ya paso el morro
    ancho_mm: float = 50.0
    t: float = 0.0            # ultima vez que se ACTUALIZO (visto o predicho)
    t_visto: float = 0.0      # ultima vez que se VIO de verdad
    t_costado: float = 0.0    # cuando paso el morro (dist <= 0)
    vistas: int = 0
    lado: int = 0
    fijo: bool = False

    def votar(self, color: str) -> None:
        self.votos[color] = self.votos.get(color, 0) + 1
        self.color = max(self.votos.items(), key=lambda kv: kv[1])[0]

    @property
    def apoyo(self) -> int:
        return self.votos.get(self.color, 0)

    @property
    def es_senal(self) -> bool:
        return self.color in LADO_REGLAMENTO


class Esquivador:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.info: Dict[str, Any] = {}     # para telemetria y dibujo
        self.restriccion = Restriccion()
        self.reiniciar()

    def reiniciar(self) -> None:
        self._dir_prev = 0.0
        self._t_prev = 0.0
        self._yaw_prev: Optional[float] = None
        self._comp_hasta = 0.0     # hasta cuando se mantiene el paso a ciegas
        self._comp_color = ""
        self._comp_lado = 0        # +1 el carro pasa por la derecha del pilar
        self._t_fin_costado = 0.0  # cuando la cola dejo atras el ultimo pilar
        self.restriccion = Restriccion()
        # --- seguimiento de pilares ---------------------------------------
        self._pistas: List[Pista] = []
        self._id_siguiente = 1
        # --- memoria del ultimo pilar (obstaculos.recordar_lado) -----------
        self.mem_color = ""        # que era
        self.mem_lado = 0          # +1 se pasaba por su derecha
        self.mem_lat = 0.0         # donde estaba de lado, en mm
        self.mem_dist = 0.0        # a que distancia se vio por ultima vez
        self.mem_t = 0.0           # cuando se vio por ultima vez

    # ------------------------------------------------------------------
    def recordar(self, color: str, lado: int, lat: float, dist: float,
                 ahora: float) -> None:
        self.mem_color = color
        self.mem_lado = lado
        self.mem_lat = lat
        self.mem_dist = dist
        self.mem_t = ahora

    def memoria_viva(self, ahora: float) -> bool:
        """El ultimo pilar sigue contando aunque ya no se vea. Sirve sobre
        todo en la curva: el pilar sale de cuadro al girar y sin memoria el
        carro se olvida de que lo tenia al lado."""
        if not bool(self.cfg.get("recordar_lado", True)) or not self.mem_color:
            return False
        return (ahora - self.mem_t) <= float(
            self.cfg.get("memoria_ms", 3000)) / 1000.0

    def memoria(self) -> Dict[str, Any]:
        return {"color": self.mem_color,
                "lado": "derecha" if self.mem_lado > 0 else "izquierda",
                "lat_mm": round(self.mem_lat),
                "edad_s": round(time.time() - self.mem_t, 1) if self.mem_t else None}

    # ==================================================================
    # IDENTIFICACION: que es un pilar y que no
    # ==================================================================
    def _medidas(self, d: vision.Deteccion, dist_cam: float,
                 geo: Geometria) -> Tuple[float, int]:
        """(ancho real en mm, alto en px que DEBERIA tener un pilar ahi).

        El ancho se mide sobre el plano del suelo en la fila de la BASE, que es
        donde el objeto toca el tapete y donde la proyeccion es valida. El alto
        esperado sale de la geometria: un pilar de 100 mm a 1 m se ve de un
        tamaño y solo de uno.
        """
        y = float(max(1, min(geo.H - 1, d.base_y)))
        izq = float(geo.lateral_mm(float(d.x), y))
        der = float(geo.lateral_mm(float(d.x + d.w), y))
        alto_esp = geo.alto_esperado_px(
            max(1.0, dist_cam), float(self.cfg.get("pilar_alto_mm", 100.0)))
        return abs(der - izq), alto_esp

    def _es_pilar(self, d: vision.Deteccion, dist_cam: float,
                  geo: Geometria) -> Tuple[bool, float, str]:
        """Puede esta mancha ser una señal de transito de 50x50x100 mm?

        Devuelve (vale, ancho_mm, por_que_no). Es una prueba FISICA, no de
        imagen: no depende de como este calibrado el COLOR. Pero SI depende de
        la geometria, y ahi esta el peligro: una prueba de tamaño mal calibrada
        no se equivoca poco, BORRA TODOS LOS PILARES y deja al carro ciego, que
        es el peor fallo que puede tener. Por eso hay tres niveles:

          "no"        no se comprueba nada.
          "suave"     (por defecto) solo el ALTO contra la geometria, que
                      depende de fy, el parametro fiable: la altura de captura
                      (480) es la misma con la que se calibro. El ancho solo
                      mata lo imposible. A cambio, separar el pilar del
                      delimitador magenta queda en manos del veto por
                      solapamiento, que no usa geometria ninguna.
          "estricto"  añade la banda de ancho real en mm. Es la que caza al
                      delimitador aunque el magenta este mal calibrado, pero
                      necesita fx BIEN medido. Ojo: fx se escala con el ANCHO
                      de captura, asi que si grabas a 1920 y calibraste a 640
                      sale tres veces mayor y un pilar de 50 mm se mide como
                      17 mm: todos fuera. Compruebalo antes con
                      tools/diagnostico_pilares.py, que dice cuanto mide.
        """
        ancho_mm, alto_esp = self._medidas(d, dist_cam, geo)
        modo = str(self.cfg.get("verificar_tamano", "suave"))
        if modo == "no":
            return True, ancho_mm, ""

        # Recortado por el canto de la imagen: la medida ya no es la del objeto
        # entero, asi que no se puede usar para descartarlo. Pasa siempre con
        # el pilar que ya se tiene encima (se sale por abajo del cuadro).
        margen = 3
        cortado_lados = d.x <= margen or (d.x + d.w) >= (geo.W - margen)
        cortado_abajo = d.base_y >= (geo.H - margen)
        cortado_arriba = d.y <= margen

        if not (cortado_abajo or cortado_arriba):
            tol = float(self.cfg.get("pilar_alto_tol", 0.6))
            if not (alto_esp * (1.0 - tol) <= d.h <= alto_esp * (1.0 + tol)):
                return False, ancho_mm, f"alto {d.h}px, esperaba {alto_esp}px"

        if not cortado_lados:
            if modo == "estricto":
                lo = float(self.cfg.get("pilar_ancho_min_mm", 25.0))
                hi = float(self.cfg.get("pilar_ancho_max_mm", 120.0))
            else:
                lo, hi = 10.0, 400.0          # solo lo imposible
            if not (lo <= ancho_mm <= hi):
                return False, ancho_mm, f"ancho {ancho_mm:.0f}mm"

        return True, ancho_mm, ""

    @staticmethod
    def _solape(d: vision.Deteccion, otras: List[vision.Deteccion]) -> float:
        """Fraccion del area de 'd' que tapan las cajas de 'otras'.

        Aproximacion barata a proposito (se suman los solapes en vez de unir
        las cajas): las manchas magenta de un mismo delimitador no se pisan
        entre ellas, y si la suma se pasara de 1.0 igualmente quedaria muy por
        encima del umbral, que es lo unico que se mira.
        """
        area = float(max(1, d.w * d.h))
        tapado = 0.0
        for o in otras:
            ix = max(0, min(d.x + d.w, o.x + o.w) - max(d.x, o.x))
            iy = max(0, min(d.y + d.h, o.y + o.h) - max(d.y, o.y))
            tapado += ix * iy
        return tapado / area

    def _candidatos(self, dets: Dict[str, List[vision.Deteccion]],
                    geo: Geometria, morro: float, activar: float,
                    limite: Optional[float]) -> List[Candidato]:
        """De todas las manchas de color, las que de verdad pueden ser un
        obstaculo de ESTA seccion, ordenadas de la mas cercana a la mas lejana."""
        magentas = list(dets.get("magenta", []))
        veto = bool(self.cfg.get("ignorar_magenta", True)) and bool(magentas)
        umbral = float(self.cfg.get("solape_magenta", 0.30))
        lat_max = float(self.cfg.get("lat_max_mm", 900.0))
        mirar = list(SENALES)
        if bool(self.cfg.get("esquivar_magenta", True)):
            mirar.append("magenta")

        fuera = {"tras_linea": 0, "fuera_tamano": 0, "veto_magenta": 0,
                 "otro_carril": 0, "fuera_alcance": 0}
        salida: List[Candidato] = []
        for color in mirar:
            for d in dets.get(color, []):
                dist_cam = float(geo.fila_a_distancia(d.base_y))
                dist = dist_cam - morro
                if dist <= 0 or dist > activar:
                    # Mas lejos que activar_desde_mm, o detras del morro. Se
                    # cuenta: "no veo nada" y "lo veo pero lo ignoro por
                    # lejano" se arreglan de formas muy distintas.
                    fuera["fuera_alcance"] += 1
                    continue
                if limite is not None and dist > limite:
                    fuera["tras_linea"] += 1   # detras de la linea: otra seccion
                    continue
                lat = float(geo.lateral_mm(d.cx, d.base_y))
                if abs(lat) > lat_max:         # muy afuera: otro carril
                    fuera["otro_carril"] += 1
                    continue
                if color in SENALES:
                    # (1) EL VETO DEL MAGENTA. Una señal nunca esta encima de
                    #     un delimitador del cajon; si lo esta, es el propio
                    #     delimitador leido con el rango de rojo abierto.
                    if veto and self._solape(d, magentas) >= umbral:
                        fuera["veto_magenta"] += 1
                        continue
                    # (2) LA PRUEBA FISICA DE TAMAÑO.
                    vale, ancho_mm, motivo = self._es_pilar(d, dist_cam, geo)
                    if not vale:
                        fuera["fuera_tamano"] += 1
                        # El motivo del primero que se cae va a la telemetria y
                        # al video. Si la geometria esta mal calibrada, esto lo
                        # dice en pantalla en vez de dejar al carro ciego y
                        # callado, que es como se pierde una tarde entera.
                        self.info.setdefault("descarte", motivo)
                        continue
                else:
                    ancho_mm, _ = self._medidas(d, dist_cam, geo)
                salida.append(Candidato(color=color, det=d, dist=dist,
                                        dist_cam=dist_cam, lat=lat,
                                        ancho_mm=ancho_mm))
        for k, v in fuera.items():
            if v:
                self.info[k] = v
        salida.sort(key=lambda c: c.dist)
        return salida

    # ==================================================================
    # SEGUIMIENTO: el lado se decide UNA vez por pilar
    # ==================================================================
    def _seguir(self, cands: List[Candidato], ahora: float, dt: float,
                vel_mm_s: float, yaw: Optional[float],
                largo: float) -> List[Tuple[Pista, Candidato]]:
        """Empareja cada candidato con el pilar que ya se venia siguiendo, y
        lleva por ESTIMA los que este frame no se ven.

        Asociacion por cercania en el plano del suelo: el carro avanza unos
        pocos centimetros entre frames, asi que el mismo pilar reaparece casi
        en el mismo sitio. 'emparejar_mm' es el radio de busqueda; por encima
        de el se da por un pilar nuevo.

        Cuanto vive una ficha sin verse:
          - delante del carro: hasta max(pista_ms, ciego_max_ms);
          - AL COSTADO (ya paso el morro): hasta que la cola lo deja atras,
            por geometria, con compromiso_max_ms de red por si la velocidad
            estimada fuera cero.
        """
        caduca = max(float(self.cfg.get("pista_ms", 700)),
                     float(self.cfg.get("ciego_max_ms", 600))) / 1000.0
        comp_max = float(self.cfg.get("compromiso_max_ms", 2500)) / 1000.0
        vivas: List[Pista] = []
        for p in self._pistas:
            if p.dist <= 0.0:
                if p.dist < -(largo + 60.0):
                    # la cola lo paso: el compromiso por tiempo ya no pinta
                    # nada, y mantenerlo solo alargaria el "ir recto"
                    self._comp_hasta = 0.0
                    self._t_fin_costado = ahora
                    continue
                if p.t_costado and (ahora - p.t_costado) > comp_max:
                    self._t_fin_costado = ahora
                    continue
            elif (ahora - p.t_visto) > caduca:
                continue
            vivas.append(p)
        self._pistas = vivas

        radio = float(self.cfg.get("emparejar_mm", 260.0))
        libres = [p for p in self._pistas if p.dist > -radio]
        vistos = set()
        parejas: List[Tuple[Pista, Candidato]] = []
        for c in cands:
            mejor, mejor_d = None, radio
            for p in libres:
                sep = math.hypot(p.lat - c.lat, p.dist - c.dist)
                if sep < mejor_d:
                    mejor, mejor_d = p, sep
            if mejor is None:
                mejor = Pista(id=self._id_siguiente)
                self._id_siguiente += 1
                self._pistas.append(mejor)
            else:
                libres.remove(mejor)
            mejor.votar(c.color)
            mejor.lat, mejor.dist, mejor.t = c.lat, c.dist, ahora
            mejor.t_visto = ahora
            mejor.ancho_mm = c.ancho_mm
            mejor.vistas += 1
            vistos.add(id(mejor))
            parejas.append((mejor, c))

        # --- los que este frame NO se vieron: se adelantan por estima -------
        # El carro sigue avanzando (y girando) aunque la mascara parpadee. Sin
        # esto la posicion guardada se queda congelada y al frame siguiente ya
        # no empareja con nada (el pilar esta 20 cm mas cerca de lo que dice la
        # ficha), asi que cada parpadeo estrenaba pilar y los votos no se
        # acumulaban nunca. Con el giroscopio, ademas, se rota la posicion:
        # es lo que permite seguir al pilar cuando sale por el canto de la
        # imagen y saber cuando la COLA lo ha pasado.
        avance = vel_mm_s * dt
        giro = 0.0
        if yaw is not None and self._yaw_prev is not None:
            giro = math.radians(_norm_ang(yaw - self._yaw_prev))
        self._yaw_prev = yaw
        c_g, s_g = math.cos(giro), math.sin(giro)
        # el carro rota sobre el eje trasero, no sobre el morro: las fichas se
        # miden desde el morro, asi que se pasa al eje, se rota y se vuelve
        eje = 0.8 * largo
        for pi in self._pistas:
            if id(pi) in vistos:
                continue
            x, y = pi.lat, pi.dist - avance + eje
            # giro > 0 = el carro giro a la derecha: lo de delante se va a la
            # izquierda en el marco del carro
            pi.lat, pi.dist = x * c_g - y * s_g, x * s_g + y * c_g - eje
            pi.t = ahora
            if pi.dist <= 0.0 and not pi.t_costado:
                pi.t_costado = ahora
        return parejas

    def _pista_ciega(self, ahora: float) -> Optional[Pista]:
        """El pilar no se ve en ESTE frame, pero se sabe donde esta.

        ESTE ERA EL AGUJERO POR EL QUE SE ESCAPABA EL LADO. La mascara de color
        se cae cada dos por tres: el brillo del tapete, la exposicion
        automatica que reacciona a una pared blanca, el propio pilar entrando
        en el punto ciego de delante del carro. Y en cuanto se caia, el esquive
        se iba directo a "ir recto": el carro se olvidaba de por que lado tenia
        que pasar y lo adelantaba por donde tocara. En pista se ve clavado — el
        pilar ahi delante, perfectamente visible, y el rotulo del video
        diciendo ADELANTANDO.

        Mientras la ficha del pilar siga viva se sigue conduciendo contra su
        ULTIMA posicion conocida, que _seguir() ya adelanta cada frame con la
        velocidad real del carro y el giroscopio. Se deja de hacer cuando el
        pilar queda al costado (dist <= 0: ahi manda la prohibicion de girar
        hacia el) o cuando lleva demasiado tiempo sin verse (ciego_max_ms).

        No vale cualquier ficha: hace falta haberlo visto ciego_vistas_min
        veces. Un destello suelto de color en una pared no puede llevarse al
        carro medio segundo a ciegas.
        """
        if not bool(self.cfg.get("seguir_a_ciegas", True)):
            return None
        tope = float(self.cfg.get("ciego_max_ms", 600)) / 1000.0
        minimo = int(self.cfg.get("ciego_vistas_min", 2))
        mejor: Optional[Pista] = None
        for pi in self._pistas:
            if not pi.color or pi.dist <= 0.0 or pi.vistas < minimo:
                continue
            if (ahora - pi.t_visto) > tope:
                continue
            if mejor is None or pi.dist < mejor.dist:
                mejor = pi
        return mejor

    def _pista_costado(self, largo: float, semi_obj: float) -> Optional[Pista]:
        """El pilar que esta a punto de pasar el MORRO (costado_desde_mm) o
        ya lo paso y todavia no ha pasado la COLA.

        Es el que impone el tope de volante hacia el. Si hay mas de uno (dos
        pilares muy juntos) manda el que este mas cerca del morro."""
        minimo = int(self.cfg.get("ciego_vistas_min", 2))
        desde = float(self.cfg.get("costado_desde_mm", 120.0))
        mejor: Optional[Pista] = None
        for pi in self._pistas:
            if not pi.color or pi.lado == 0 or pi.vistas < minimo:
                continue
            if not (-(largo + semi_obj) < pi.dist <= desde):
                continue
            if mejor is None or pi.dist > mejor.dist:
                mejor = pi
        return mejor

    # ------------------------------------------------------------------
    def _huecos(self, y_p: float, p: PerfilMuro, geo: Geometria,
                error_rumbo: Optional[float] = None,
                veto: Optional[List[Tuple[int, int]]] = None
                ) -> Tuple[float, float]:
        """Limites laterales (izq, der) del pasillo libre hasta la profundidad
        del pilar, en mm y EN EL MARCO DEL CARRIL. Sin pared a la vista
        devuelve +-1500.

        Cada punto de contacto del muro se pasa del marco del carro al del
        carril con el error de rumbo: con el carro cruzado, la pared de al
        lado se ve delante y a lateral cero, y medida en el marco del carro
        cerraba el hueco. Sin giroscopio (error_rumbo None) es lo de siempre.

        'veto' son rangos de columnas ocupadas por un pilar detectado. UN
        PILAR NO ES UNA PARED: si su silueta (o su sombra, con el metodo
        'negro') se cuela en el perfil, el hueco libre se parte en dos por el
        propio pilar que se esta rodeando, el punto de paso se recorta contra
        el, y el esquive termina pasandolo RASPANDO — que es la otra mitad de
        "lo choca". Sus columnas no cuentan aqui; el despeje al pilar ya lo
        pone el esquive, que sabe cuanto mide.
        """
        e = math.radians(error_rumbo) if error_rumbo is not None else 0.0
        libre_min = y_p + 250.0      # la pared debe quedar mas lejos que el pilar
        izq_lim, der_lim = -1500.0, 1500.0
        for c in range(0, p.ancho, 6):
            if not p.valido[c] or p.dist_mm[c] > libre_min + 800.0:
                continue
            if veto and any(a <= c <= b for a, b in veto):
                continue
            x_c = float(geo.lateral_mm(c, max(1, int(p.y_contacto[c]))))
            x_l, y_l = _al_carril(x_c, float(p.dist_mm[c]), e)
            ### DELANTE, no al costado. Con el carro cruzado hay contacto de
            ### muro que en el marco del carril cae a la altura del carro o
            ### detras: esa pared ya no condiciona por donde pasar el pilar
            ### que viene, y contandola el hueco salia falsamente estrecho.
            if not (50.0 < y_l <= libre_min):
                continue
            if x_l < 0:
                izq_lim = max(izq_lim, x_l)
            else:
                der_lim = min(der_lim, x_l)
        return izq_lim, der_lim

    def _columnas_de(self, dets: Dict[str, List[vision.Deteccion]],
                     geo: Optional[Geometria] = None,
                     morro: float = 60.0,
                     colores: Tuple[str, ...] = SENALES,
                     margen: int = 6) -> List[Tuple[int, int]]:
        """Rangos de columnas que ocupa cada señal: las que se VEN ahora y las
        que solo se siguen por estima.

        Las estimadas importan tanto como las vistas: cuando la mascara de
        color parpadea, el pilar desaparece de 'dets' pero NO de la mascara
        del muro (ahi sigue su sombra), asi que sin esto el hueco libre se
        parte justo en el pilar durante los frames ciegos, que son los
        ultimos antes de pasarlo.
        """
        out: List[Tuple[int, int]] = []
        for color in colores:
            for d in dets.get(color, []):
                out.append((d.x - margen, d.x + d.w + margen))
        if geo is None:
            return out
        for pi in self._pistas:
            if not pi.color or pi.dist <= 0.0 or not pi.vistas:
                continue
            semi = max(25.0, pi.ancho_mm / 2.0)
            try:
                u_i, _ = geo.suelo_a_pixel(pi.lat - semi, pi.dist + morro)
                u_d, _ = geo.suelo_a_pixel(pi.lat + semi, pi.dist + morro)
            except Exception:
                continue
            out.append((min(u_i, u_d) - margen, max(u_i, u_d) + margen))
        return out

    def _lado_magenta(self, p: Pista, sentido: int, perfil: Optional[PerfilMuro],
                      geo: Geometria, semi_obj: float, semi_carro: float,
                      veto: Optional[List[Tuple[int, int]]] = None) -> int:
        """Por donde se rodea un delimitador del cajon. El reglamento no manda
        lado, asi que se decide por SITIO:

          1. si el perfil del muro dice que a un lado no cabe el carro, por
             el otro; si cabe por los dos, por el que tenga mas hueco;
          2. si no se ve pared, por el lado INTERIOR de la pista: el cajon
             esta siempre contra la pared exterior (reglas 13.7 a 13.9), asi
             que en horario se pasa por su derecha y en antihorario por su
             izquierda;
          3. sin sentido conocido, por el lado que menos cruza la trayectoria.
        """
        if perfil is not None and getattr(perfil, "hay_muro", False):
            izq_lim, der_lim = self._huecos(p.dist, perfil, geo, None, veto)
            sitio_der = der_lim - (p.lat + semi_obj)
            sitio_izq = (p.lat - semi_obj) - izq_lim
            # (marco del carro: el sentido dice el interior, y con el carro
            # cruzado al pasar el cajon eso es lo que de verdad decide)
            necesario = 2.0 * semi_carro + 30.0
            if sitio_der >= necesario > sitio_izq:
                return +1
            if sitio_izq >= necesario > sitio_der:
                return -1
            if abs(sitio_der - sitio_izq) > 150.0:
                return 1 if sitio_der > sitio_izq else -1
        if sentido != 0:
            return 1 if sentido > 0 else -1
        return 1 if p.lat <= 0 else -1

    def _lado_de(self, p: Pista, sentido: int,
                 perfil: Optional[PerfilMuro] = None,
                 geo: Optional[Geometria] = None,
                 semi_obj: float = 25.0, semi_carro: float = 100.0,
                 veto: Optional[List[Tuple[int, int]]] = None) -> int:
        """El lado por el que se pasa este pilar. Una vez fijado no cambia.

        Mientras no haya votos suficientes se usa el lado del color que va
        ganando -hay que esquivar desde ya, no se puede esperar-, pero no se
        da por definitivo. En cuanto el mismo color gana 'votos_color' frames,
        el lado queda CLAVADO: pasar por el lado incorrecto termina la ronda,
        y un parpadeo del detector no es motivo para cambiar de opinion.
        """
        if p.fijo and p.lado:
            return p.lado

        if p.es_senal:
            lado = LADO_REGLAMENTO[p.color]
            if sentido < 0 and bool(self.cfg.get("invertir_en_antihorario", False)):
                lado = -lado
        elif geo is not None:
            lado = self._lado_magenta(p, sentido, perfil, geo, semi_obj,
                                      semi_carro, veto)
        else:
            lado = 1 if p.lat <= 0 else -1

        p.lado = lado
        if (p.apoyo >= int(self.cfg.get("votos_color", 3))
                and bool(self.cfg.get("fijar_lado", True))):
            p.fijo = True
        return lado

    def _semi_de(self, p: Pista) -> float:
        """Medio ancho REAL del obstaculo, en mm, para calcular el despeje."""
        if p.es_senal:
            return float(self.cfg.get("semi_pilar_mm", 25.0))
        # El delimitador del cajon mide 200 mm de largo y 20 de grueso, y
        # segun como se vea el centroide cae en el medio de cualquiera de las
        # dos medidas: el despeje se cuenta desde su CANTO mas lejano, no
        # desde el centro, o el carro le pasaria por encima creyendo que sobra
        # sitio. Visto de canto mide 20 mm en la imagen: de ahi que el minimo
        # sea el de un muro de 200, no lo que diga la camara.
        return max(float(self.cfg.get("magenta_semi_mm", 110.0)), p.ancho_mm / 2.0)

    # ==================================================================
    def paso(self, dets: Dict[str, List[vision.Deteccion]],
             perfil: Optional[PerfilMuro], geo: Geometria,
             dist_lineas: Optional[Dict[str, float]] = None,
             en_esquina: bool = False, sentido: int = 0,
             vel_mm_s: float = 0.0, yaw: Optional[float] = None,
             error_rumbo: Optional[float] = None) -> Tuple[float, float]:
        """Devuelve (direccion_deseada_pct, peso 0..1). Lo demas que el
        esquive le pide al navegador queda en self.restriccion.

        dets:        detecciones por color. Necesita 'rojo' y 'verde', y usa
                     'magenta' si esta: para vetar falsos pilares y para rodear
                     los delimitadores del cajon de estacionamiento.
        dist_lineas: distancia a las lineas del piso que se ven delante.
        en_esquina:  ya dentro de la curva; el limite de seccion se levanta.
        sentido:     +1 horario, -1 antihorario. Decide por donde se rodea el
                     magenta (por el interior) y lo usa el interruptor
                     invertir_en_antihorario, que viene apagado.
        vel_mm_s:    velocidad real estimada (con signo: negativa en reversa)
                     para llevar por estima donde queda cada pilar.
        yaw:         rumbo del giroscopio, en grados. Con el, la estima de los
                     pilares que no se ven tambien ROTA cuando el carro gira.
        error_rumbo: grados que el carro se desvia del rumbo de la recta
                     (+ = apunta a la derecha). Con el, el modo 'arco' apunta
                     a la linea de paso PARALELA AL CARRIL en vez de a un punto,
                     y se limita cuanto se cruza el carro al esquivar.
        """
        ahora = time.time()
        dt = min(0.25, ahora - self._t_prev) if self._t_prev else 0.05
        self._t_prev = ahora
        self.info = {}
        self.restriccion = Restriccion()
        if not bool(self.cfg.get("activo", False)):
            self._comp_hasta = 0.0
            self._pistas.clear()
            return 0.0, 0.0
        salida = self._paso(dets, perfil, geo, dist_lineas, en_esquina, sentido,
                            vel_mm_s, yaw, error_rumbo, ahora, dt)
        # --- SALIENDO del paso: la cola acaba de dejar atras un pilar --------
        # El carro queda pegado a una pared y algo cruzado. Durante un momento
        # se sigue tratando como maniobra (el muro "de frente" es el de frente
        # por orientacion, no el pasillo crudo) y se sigue despacio, que el
        # siguiente pilar suele venir enseguida.
        r = self.restriccion
        if (self._t_fin_costado and not r.al_costado
                and (ahora - self._t_fin_costado) * 1000.0
                < float(self.cfg.get("recuperar_ms", 1000))):
            r.recuperando = True
            r.vel_max_pct = float(self.cfg.get("vel_esquive", 40))
            r.peso_vel = 1.0
            self.info["recuperando_s"] = round(
                float(self.cfg.get("recuperar_ms", 1000)) / 1000.0
                - (ahora - self._t_fin_costado), 2)
        return salida

    def _paso(self, dets, perfil, geo, dist_lineas, en_esquina, sentido,
              vel_mm_s, yaw, error_rumbo, ahora: float, dt: float
              ) -> Tuple[float, float]:
        activar = float(self.cfg.get("activar_desde_mm", 1600.0))
        mandar = float(self.cfg.get("mandar_desde_mm", 700.0))
        morro = float(geo.cfg.get("morro_mm", 60.0))
        largo = float(geo.cfg.get("largo_carro_mm", 240.0))
        semi_carro = float(geo.cfg.get("ancho_carro_mm", 200.0)) / 2.0

        # --- hasta donde llega ESTA seccion --------------------------------
        limite = None
        if (bool(self.cfg.get("limitar_por_lineas", True)) and not en_esquina
                and dist_lineas):
            cerca = min(dist_lineas.values())
            limite = cerca + float(self.cfg.get("margen_linea_mm", 60.0))
            self.info["limite_mm"] = round(limite)

        # --- identificar y seguir ------------------------------------------
        cands = self._candidatos(dets, geo, morro, activar, limite)
        parejas = self._seguir(cands, ahora, dt, vel_mm_s, yaw, largo)
        # el lado se resuelve para TODAS las fichas con color, no solo para la
        # que manda: cuando un pilar pasa al costado ya tiene que saberse por
        # que lado se le estaba pasando
        veto = self._columnas_de(dets, geo, morro)
        for pi in self._pistas:
            if pi.color and pi.dist > 0.0:
                self._lado_de(pi, sentido, perfil, geo, self._semi_de(pi),
                              semi_carro, veto)

        # --- el pilar que queda AL COSTADO: no girar hacia el --------------
        costado = self._pista_costado(largo, float(self.cfg.get("magenta_semi_mm", 110.0)))
        if costado is not None:
            self._restringir_costado(costado, semi_carro)

        # --- el pilar que MANDA: el mas cercano de los que estan delante ----
        # Entre los que se VEN y el que se sigue A CIEGAS, el mas cercano. El
        # pilar que se esta pasando sale del cuadro por el canto a unos 25 cm
        # del morro; si en ese momento mandara el siguiente (visto, a 1,4 m,
        # con peso casi nulo), el navegador dejaria de estar en maniobra en
        # pleno paso y volveria a medir el muro con el pasillo crudo: reversa
        # o esquina falsa con el pilar a 15 cm. Visto en el simulador.
        ciego = False
        pista_v = parejas[0][0] if parejas else None
        pista_c = self._pista_ciega(ahora)
        if pista_c is not None and (pista_v is None or pista_c.dist < pista_v.dist):
            pista, ciego = pista_c, True
            dist, lat, d_mejor = pista.dist, pista.lat, None
            ancho_mm = pista.ancho_mm
        elif pista_v is not None:
            pista, cand = parejas[0]      # el mas cercano es el que manda
            dist, lat, d_mejor = cand.dist, cand.lat, cand.det
            ancho_mm = cand.ancho_mm
        else:
            # No se ve ninguno AHORA ni se sigue ninguno por estima.
            return self._sin_pilar(ahora, dt, costado)

        color = pista.color
        semi_obj = self._semi_de(pista)
        lado = self._lado_de(pista, sentido, perfil, geo, semi_obj, semi_carro, veto)

        # --- de aqui en adelante, en el MARCO DEL CARRIL --------------------
        # (x = lateral, + derecha; y = a lo largo de la recta), no en el del
        # carro. Esquivando, el carro va cruzado a proposito, y en SU marco la
        # pared de al lado se ve DELANTE: el pasillo libre parecia estrecharse
        # hasta cerrarse y el recorte mandaba al "centro del hueco", o sea de
        # frente contra la pared (visto en el simulador saliendo torcido de
        # una curva). En el marco del carril las paredes son paralelas y el
        # hueco es el que es. Sin giroscopio los dos marcos coinciden.
        e = math.radians(error_rumbo) if error_rumbo is not None else 0.0
        x_p, y_p = _al_carril(lat, dist, e)

        # Despeje COMODO (el que se pide) y MINIMO (el que fisicamente hace
        # falta para no tocarlo). Si el hueco no da para el comodo se aprieta
        # hacia el minimo, pero jamas se cambia de lado.
        despeje = semi_carro + float(self.cfg.get("margen_mm", 70.0)) + semi_obj
        minimo = semi_carro + semi_obj + 15.0
        x_t = x_p + lado * despeje

        # --- recortar al pasillo libre a esa distancia ---------------------
        if perfil is not None and perfil.hay_muro:
            x_t = self._recortar(x_t, y_p, perfil, geo, semi_carro,
                                 x_p, lado, minimo, error_rumbo, veto)
        # el punto de paso, de vuelta en el marco del carro (video, modos viejos)
        objetivo, _ = _al_carro(x_t, y_p, e)

        # --- ya casi encima: esto ya es el paso AL COSTADO -----------------
        # A menos de costado_desde_mm el volante ya no cambia por donde se
        # pasa el morro; solo cambia cuanto barre el costado. Manda la regla
        # del costado (poco o ningun volante hacia el pilar) y no el arco,
        # que mirando mas alla de un pilar que se tiene encima podia pedir
        # girar hacia el.
        if dist <= float(self.cfg.get("costado_desde_mm", 120.0)):
            self._restringir_costado(pista, semi_carro)
            r = self.restriccion
            if abs(lat) < semi_carro + semi_obj:
                r.bloqueo_mm = dist
                r.lado_bloqueo = lado
            self._rampa(0.0, dt)
            self.recordar(color, lado, lat, dist, ahora)
            self.info.update({"color": color, "dist_mm": round(dist),
                              "lat_mm": round(lat), "objetivo_mm": round(objetivo),
                              "lado": "derecha" if lado > 0 else "izquierda",
                              "senal": pista.es_senal, "id": pista.id,
                              "votos": pista.apoyo, "fijo": pista.fijo,
                              "ancho_mm": round(ancho_mm), "ciego": ciego,
                              "vistos": len(parejas), "peso": 0.0, "dir": 0,
                              "vel_tope_pct": r.vel_max_pct})
            if r.bloqueo_mm is not None:
                self.info["bloqueo_mm"] = round(r.bloqueo_mm)
            return 0.0, 0.0

        # --- a direccion ---------------------------------------------------
        modo = str(self.cfg.get("modo", "arco"))
        if modo == "borde" and d_mejor is not None:
            direccion = self._dir_borde(d_mejor, lado, dist, geo)
        elif modo == "arco":
            direccion = self._dir_arco(x_t, y_p, e, geo)
        else:
            # 'punto': se apunta al hueco al costado del pilar. La mirada
            # minima es lo que impide que el volante se vaya a tope al
            # acercarse: sin ella el angulo al punto crece sin limite.
            mirada = max(float(self.cfg.get("mirada_min_mm", 350.0)), dist)
            ang = math.degrees(math.atan2(objetivo, mirada))
            direccion = ang * float(self.cfg.get("k_dir", 1.4))
        # Ya muy cruzado hacia el lado de paso: no abrirse mas. Un carro que
        # va 35 grados torcido en un carril de 60-100 cm se mete en la pared
        # de al lado antes de llegar al pilar.
        tope_desvio = float(self.cfg.get("desvio_max_esquive_deg", 35.0))
        if error_rumbo is not None and lado * error_rumbo > tope_desvio:
            direccion = min(direccion, 0.0) if lado > 0 else max(direccion, 0.0)
            self.info["rumbo_tope"] = True
        tope = float(self.cfg.get("dir_max_pct", 80.0))
        direccion = max(-tope, min(tope, direccion))
        direccion = self._rampa(direccion, dt)

        t = (activar - dist) / max(1.0, activar - mandar)
        peso_max = float(self.cfg.get("peso_max", 1.0))
        peso = peso_max * min(1.0, max(0.0, t))
        if ciego:
            # Se conduce contra una posicion estimada, no medida: pesa algo
            # menos que verlo, pero muchisimo mas que olvidarse de el.
            peso *= float(self.cfg.get("peso_ciego", 0.9))

        # --- lo que se le pide al navegador ademas del volante -------------
        r = self.restriccion
        r.vel_max_pct = float(self.cfg.get("vel_esquive", 40))
        r.peso_vel = max(r.peso_vel, min(1.0, peso / max(1e-6, peso_max)))
        if abs(lat) < semi_carro + semi_obj:
            # esta EN el corredor de las ruedas: si el esquive no lo saca de
            # ahi antes de pilar_parar_mm, el navegador escapa marcha atras
            r.bloqueo_mm = dist
            r.lado_bloqueo = lado

        # --- comprometerse a adelantarlo entero (respaldo por tiempo) ------
        # Cuando hay giroscopio y velocidad la ficha del pilar se sigue hasta
        # que la cola lo pasa, y el compromiso por tiempo no llega a mandar.
        # Sin ellos (o si la estima se pierde) es lo que impide volver hacia
        # el pilar mientras esta en el punto ciego de delante del carro.
        seg = 0.0
        if dist <= mandar:
            v = max(150.0, vel_mm_s)      # suelo: no dividir por casi cero
            seg = (dist + largo + despeje * 0.5) / v
            seg = min(seg, float(self.cfg.get("compromiso_max_ms", 2500)) / 1000.0)
            self._comp_hasta = ahora + seg
            self._comp_color = color
            self._comp_lado = lado

        self.recordar(color, lado, lat, dist, ahora)
        self.info.update({"color": color, "dist_mm": round(dist),
                          "lat_mm": round(lat), "objetivo_mm": round(objetivo),
                          "lado": "derecha" if lado > 0 else "izquierda",
                          "senal": pista.es_senal, "id": pista.id,
                          "votos": pista.apoyo, "fijo": pista.fijo,
                          "ancho_mm": round(ancho_mm), "ciego": ciego,
                          "sin_ver_s": round(ahora - pista.t_visto, 2),
                          "vistos": len(parejas), "modo": modo,
                          "peso": round(peso, 2), "dir": round(direccion),
                          "compromiso_s": round(seg, 2),
                          "vel_tope_pct": r.vel_max_pct})
        if r.bloqueo_mm is not None:
            self.info["bloqueo_mm"] = round(r.bloqueo_mm)
        return direccion, peso

    # ------------------------------------------------------------------
    def _dir_arco(self, x_t: float, y_p: float, e: float,
                  geo: Geometria) -> float:
        """Modo 'arco': pure pursuit hacia la LINEA de paso.

        En el marco del carril, la linea de paso es x = x_t (paralela al
        carril, a x_t mm del eje actual del carro) y el pilar esta a y_p mm
        por delante. Se elige sobre la linea el punto que queda a 'mirada'
        mm por delante, nunca mas alla del pilar, se pasa al marco del carro
        (girado 'e') y se calcula el arco que pasa por el:

            curvatura = 2 * x / (x^2 + y^2)

        que es la formula del pure pursuit. La curvatura se convierte a
        volante con el radio de giro real del carro: a tope de direccion el
        carro describe geometria.radio_giro_mm, y en el rango del servo la
        curvatura es casi lineal con el porcentaje. Asi el volante pedido es
        el que la geometria necesita, ni la mitad (como con la ganancia fija
        del modo 'punto') ni el doble.

        Por que a la LINEA y no al punto: el arco que pasa por un punto llega
        a el CRUZADO, con el doble del angulo con que se ve el punto; con el
        pilar a 70 cm y el paso a 20 cm de lado son 30 grados, derecho a la
        pared de al lado. El arco a la linea llega paralelo, y en cuanto el
        carro esta sobre ella la curvatura es cero: recto.

        La MIRADA se acorta cuando queda poco que desplazarse: el pure
        pursuit converge a la linea en unas cuantas miradas (exponencial), y
        con la mirada larga de siempre los ultimos 3 cm tardaban un metro en
        llegar. Lejos de la linea se mantiene larga, que es lo que acota
        cuanto se cruza el carro (~ atan(x_t / mirada)). Y con el pilar mas
        cerca que la mirada, se mira hasta el pilar (el arco se cierra: es la
        unica forma de llegar a la linea antes que a el), pero nunca menos de
        mirada_min_mm: mirar a 15 cm con el pilar encima pedia el volante a
        tope por 3 cm de nada.
        """
        mirada = float(self.cfg.get("mirada_mm", 600.0))
        mirada_min = float(self.cfg.get("mirada_min_mm", 350.0))
        L = max(mirada_min, min(mirada, 3.0 * abs(x_t)))
        L = max(mirada_min, min(L, y_p))
        x, y = _al_carro(x_t, L, e)
        curvatura = 2.0 * x / max(1.0, x * x + y * y)
        radio = float(geo.cfg.get("radio_giro_mm", 550.0))
        direccion = 100.0 * curvatura * radio * float(self.cfg.get("k_arco", 1.0))
        self.info["arco_x"] = round(x)
        self.info["arco_y"] = round(y)
        return direccion

    def _dir_borde(self, d, lado: int, dist: float, geo: Geometria) -> float:
        """Modo 'borde': mantener el pilar pegado al CANTO de la imagen.

        En vez de apuntar a un hueco calculado en milimetros, se vigila el
        borde del pilar que da al centro y se le mantiene sobre una columna
        objetivo, cerca del canto por el que tiene que salir. Es control
        visual puro: no depende de la calibracion de distancias y aguanta que
        el pilar se vea mal de lejos, porque solo hace falta su silueta. Al
        pilar ROJO se le empuja hacia el canto IZQUIERDO de la imagen (el
        carro pasa por su derecha) y al VERDE hacia el derecho.
        """
        W = max(1, geo.W)
        frac = float(self.cfg.get("borde_frac", 0.18))
        # columna donde queremos ver el borde del pilar
        x_obj = W * frac if lado > 0 else W * (1.0 - frac)
        # borde del pilar que mira al centro de la imagen
        x_borde = float(d.x + d.w) if lado > 0 else float(d.x)
        err = (x_borde - x_obj) / W            # >0 = hay que empujarlo mas
        direccion = float(self.cfg.get("k_borde", 90.0)) * err

        # Ya encima: giro grande para que la COLA tambien lo libre.
        if dist <= float(self.cfg.get("giro_final_mm", 380.0)):
            direccion += lado * float(self.cfg.get("giro_final_pct", 25.0))
            self.info["giro_final"] = True
        self.info["borde_px"] = round(x_borde)
        self.info["borde_obj_px"] = round(x_obj)
        return direccion

    # ------------------------------------------------------------------
    def _restringir_costado(self, p: Pista, semi_carro: float) -> None:
        """Hay un pilar entre el morro y la cola: hacia el, poco volante (o
        ninguno), y despacio hasta que la cola lo deje atras.

        Cuanto se deja girar hacia el sale de la HOLGURA con la que se le
        esta pasando (posicion por estima). Con radio_giro_mm de 550, un 1 %
        de volante durante los 30 cm que dura el paso mueve el costado del
        carro menos de 1 mm hacia adentro: con la mitad de la holgura en % de
        volante, el barrido se queda en menos de la mitad de ella. Sin
        holgura, nada: recto. Enderezar el rumbo SI hace falta permitirlo,
        porque si no el carro sigue cruzado hacia la pared de al lado durante
        todo el paso y llega a ella antes de que la cola libre el pilar.
        """
        r = self.restriccion
        holgura = abs(p.lat) - semi_carro - self._semi_de(p)
        tope = min(float(self.cfg.get("costado_giro_max_pct", 25.0)),
                   max(0.0, holgura / 2.0))
        if bool(self.cfg.get("no_volver_al_costado", True)):
            r.no_girar = -p.lado
            r.tope_hacia_pct = tope
        r.al_costado = True
        r.vel_max_pct = float(self.cfg.get("vel_esquive", 40))
        r.peso_vel = 1.0
        self.info.update({"al_costado_mm": round(p.dist),
                          "costado_color": p.color,
                          "holgura_mm": round(holgura),
                          "tope_hacia_pct": round(tope),
                          "no_girar": ("derecha" if r.no_girar > 0 else "izquierda")
                          if r.no_girar else ""})

    def _sin_pilar(self, ahora: float, dt: float,
                   costado: Optional[Pista] = None) -> Tuple[float, float]:
        """No se ve ningun pilar delante. Tres casos muy distintos:

        - Hay uno AL COSTADO (seguido por estima): rumbo firme, sin volver
          hacia el, hasta que la cola lo pase.
        - Se perdio de CERCA sin estima que valga: esta en el punto ciego de
          delante del carro. Toca el compromiso de adelantamiento por tiempo
          (ir recto hasta que la cola pase).
        - Se perdio de LEJOS: se salio de cuadro por el canto porque el carro
          giro de mas. En modo 'borde' se corrige hacia el para volver a
          tenerlo en el canto, que es donde tiene que estar.
        """
        if costado is not None:
            # El esquive no pide volante: manda el centrado y el rumbo de
            # siempre, que es lo que endereza el carro, con el tope hacia el
            # pilar que ya puso _restringir_costado. Pedir "recto" con peso
            # dejaba al carro cruzado hacia la pared de al lado todo el paso.
            self._rampa(0.0, dt)
            self.info.update({"color": costado.color, "peso": 0.0,
                              "lado": "derecha" if costado.lado > 0 else "izquierda",
                              "dir": 0})
            return 0.0, 0.0

        if ahora < self._comp_hasta:
            return self._compromiso(ahora, dt)

        if (str(self.cfg.get("modo", "arco")) == "borde"
                and self.memoria_viva(ahora)
                and self.mem_dist > float(self.cfg.get("mandar_desde_mm", 700.0))):
            # Salio de cuadro estando lejos: no hay peligro, se le busca.
            direccion = self._rampa(
                -self.mem_lado * float(self.cfg.get("buscar_pct", 18.0)), dt)
            peso = float(self.cfg.get("peso_buscar", 0.35))
            self.info.update({"color": self.mem_color, "buscando": True,
                              "lado": "derecha" if self.mem_lado > 0 else "izquierda",
                              "dir": round(direccion), "peso": round(peso, 2)})
            return direccion, peso

        return self._compromiso(ahora, dt)

    def _compromiso(self, ahora: float, dt: float) -> Tuple[float, float]:
        """El pilar ya no se ve porque esta demasiado cerca (la camara no
        llega tan abajo) y no hay estima que lo siga. Mientras dure el
        compromiso se pide RECTO: no se corrige hacia el lado del pilar, que
        es como la rueda trasera lo barre. No se mantiene el giro de
        aproximacion, solo se impide volver."""
        if ahora >= self._comp_hasta:
            if self._comp_hasta:
                self._t_fin_costado = max(self._t_fin_costado, self._comp_hasta)
                self._comp_hasta = 0.0
            self._dir_prev = 0.0
            return 0.0, 0.0
        queda = self._comp_hasta - ahora
        peso = float(self.cfg.get("peso_compromiso", 0.7))
        direccion = self._rampa(0.0, dt)
        r = self.restriccion
        if bool(self.cfg.get("no_volver_al_costado", True)) and self._comp_lado:
            r.no_girar = -self._comp_lado
        r.vel_max_pct = float(self.cfg.get("vel_esquive", 40))
        r.peso_vel = 1.0
        self.info.update({"color": self._comp_color, "peso": round(peso, 2),
                          "lado": "derecha" if self._comp_lado > 0 else "izquierda",
                          "adelantando_s": round(queda, 2), "dir": round(direccion)})
        return direccion, peso

    def _rampa(self, objetivo: float, dt: float) -> float:
        """Limita cuanto puede cambiar la direccion por segundo. Sin esto el
        esquive da un volantazo en un solo frame."""
        tasa = float(self.cfg.get("rampa_dir_pct_s", 220.0))
        maximo = tasa * dt
        d = objetivo - self._dir_prev
        if d > maximo:
            objetivo = self._dir_prev + maximo
        elif d < -maximo:
            objetivo = self._dir_prev - maximo
        self._dir_prev = objetivo
        return objetivo

    # ------------------------------------------------------------------
    def _recortar(self, objetivo: float, dist: float, p: PerfilMuro,
                  geo: Geometria, semi_carro: float,
                  lat: float = 0.0, lado: int = 0,
                  minimo: float = 0.0,
                  error_rumbo: Optional[float] = None,
                  veto: Optional[List[Tuple[int, int]]] = None) -> float:
        """Recorta el punto de paso al hueco libre SIN CAMBIARLO DE LADO.
        Todo en el marco del carril (con error_rumbo None, el del carro).

        EL FALLO QUE ARREGLA (visto en pista, en antihorario): el recorte
        contra el muro podia dejar el objetivo al otro lado del pilar. Con los
        valores del equipo -margen 261 mm, o sea 386 mm de despeje pedido- eso
        pasaba en cuanto el pilar estaba a mas de ~250 mm hacia el muro:
        objetivo 686 mm recortado a 260 con el pilar en 300 => se pasaba por
        su IZQUIERDA un pilar rojo. Y pasar por el lado incorrecto termina la
        ronda (regla 9.25.5). No era el sentido de giro: era el recorte.

        Ahora el lado es sagrado. Si el hueco no da para el despeje comodo, se
        aprieta hasta el MINIMO fisico (medio carro + medio pilar), pero nunca
        se cruza al otro lado; si ni eso cabe, se avisa y manda la seguridad
        del muro, que es quien debe frenar.
        """
        # Cuanto se deja entre el costado del carro y la PARED. Menos que el
        # despeje al pilar a proposito: cuando el hueco esta justo (pilar a
        # 30 cm de la pared, carro de 20) hay 7 cm que repartir, y rozar la
        # pared no penaliza; mover el pilar, si.
        margen = semi_carro + float(self.cfg.get("margen_pared_mm", 30.0))
        izq_lim, der_lim = self._huecos(dist, p, geo, error_rumbo, veto)
        lo, hi = izq_lim + margen, der_lim - margen
        if lo > hi:
            self.info["hueco_estrecho_mm"] = round(der_lim - izq_lim)
            return (izq_lim + der_lim) / 2.0

        recortado = max(lo, min(hi, objetivo))
        if lado == 0:
            return recortado

        # --- el lado no se negocia -----------------------------------------
        borde = lat + lado * minimo          # lo mas pegado que se puede ir
        if lado > 0:                         # hay que quedar a la DERECHA
            if recortado < borde:
                self.info["sin_sitio"] = "no cabe por la derecha"
                return min(max(borde, recortado), max(borde, hi)) \
                    if hi >= borde else borde
        else:                                # hay que quedar a la IZQUIERDA
            if recortado > borde:
                self.info["sin_sitio"] = "no cabe por la izquierda"
                return max(min(borde, recortado), min(borde, lo)) \
                    if lo <= borde else borde
        return recortado
