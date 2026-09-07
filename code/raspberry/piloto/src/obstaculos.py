"""
obstaculos.py — Esquive de las señales de transito.

REGLA DEL JUEGO (reglamento 2026, punto 9.19)
El pilar ROJO se pasa por su DERECHA; el VERDE por su IZQUIERDA. "Derecha" e
"izquierda" son las del VEHICULO segun avanza, no las del tapete: el punto 5
lo dice como "el lado del CARRIL por el que debe circular". Por eso NO hay que
invertir nada al cambiar de sentido — el mismo pilar fisico se pasa por un
lado distinto en horario que en antihorario, y eso ya sale solo de trabajar en
el marco del carro. Aun asi queda el interruptor `invertir_en_antihorario`
por si en tu pista se interpreta al reves: apagado por defecto.

Matiz que se olvida: "pasar por la derecha del pilar" NO es "girar a la
derecha". Si el pilar esta a la izquierda del carro, el punto de paso puede
quedar a la izquierda del centro de la imagen.

COMO SE PASA UN PILAR, EN TRES ACTOS
  1. APROXIMACION. Se apunta a un punto al costado correcto del pilar,
     separado medio carro + margen. Se convierte en direccion con una
     MIRADA MINIMA (mirada_min_mm): sin ella, al acercarse el angulo al punto
     crece hasta pedir el volante a tope, que es justo lo que hacia que el
     carro girase de golpe casi al 100 %.
  2. COMPROMISO. Cuando el pilar queda demasiado cerca deja de verse: la
     camara no llega tan abajo. Si en ese momento el esquive desaparece, el
     centrado tira del carro hacia el medio del carril y la RUEDA TRASERA
     barre el pilar (direccion Ackermann: la cola corta por dentro). Por eso,
     desde que se pierde de vista se mantiene el rumbo -sin volver hacia el
     pilar- el tiempo que el carro necesita para adelantarlo con TODO su
     largo. Se calcula con la velocidad real, no con un tiempo fijo.
  3. SALIDA. Cumplido el compromiso, manda otra vez el centrado.

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
from typing import Any, Dict, List, Optional, Tuple

from . import vision
from .geometria import Geometria
from .muro import PerfilMuro


class Esquivador:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.info: Dict[str, Any] = {}     # para telemetria y dibujo
        self.reiniciar()

    def reiniciar(self) -> None:
        self._dir_prev = 0.0
        self._t_prev = 0.0
        self._comp_hasta = 0.0     # hasta cuando se mantiene el paso a ciegas
        self._comp_color = ""
        self._comp_lado = 0        # +1 el pilar quedo a la izquierda del carro
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

    # ------------------------------------------------------------------
    def paso(self, dets: Dict[str, List[vision.Deteccion]],
             perfil: Optional[PerfilMuro], geo: Geometria,
             dist_lineas: Optional[Dict[str, float]] = None,
             en_esquina: bool = False, sentido: int = 0,
             vel_mm_s: float = 0.0) -> Tuple[float, float]:
        """Devuelve (direccion_deseada_pct, peso 0..1).

        dist_lineas: distancia a las lineas del piso que se ven delante.
        en_esquina:  ya dentro de la curva; el limite de seccion se levanta.
        sentido:     +1 horario, -1 antihorario (solo lo usa el interruptor
                     invertir_en_antihorario, que viene apagado).
        vel_mm_s:    velocidad real estimada, para saber cuanto dura el
                     compromiso de adelantar al pilar.
        """
        ahora = time.time()
        dt = min(0.25, ahora - self._t_prev) if self._t_prev else 0.05
        self._t_prev = ahora
        self.info = {}
        if not bool(self.cfg.get("activo", False)):
            self._comp_hasta = 0.0
            return 0.0, 0.0

        activar = float(self.cfg.get("activar_desde_mm", 1600.0))
        mandar = float(self.cfg.get("mandar_desde_mm", 700.0))
        morro = float(geo.cfg.get("morro_mm", 60.0))

        # --- hasta donde llega ESTA seccion --------------------------------
        limite = None
        if (bool(self.cfg.get("limitar_por_lineas", True)) and not en_esquina
                and dist_lineas):
            cerca = min(dist_lineas.values())
            limite = cerca + float(self.cfg.get("margen_linea_mm", 60.0))
            self.info["limite_mm"] = round(limite)

        # --- pilar mas cercano dentro del alcance --------------------------
        descartados = 0
        mejor = None          # (dist, lat, color, deteccion)
        for color in ("rojo", "verde"):
            for d in dets.get(color, []):
                dist = float(geo.fila_a_distancia(d.base_y)) - morro
                if dist <= 0 or dist > activar:
                    continue
                if limite is not None and dist > limite:
                    descartados += 1     # esta detras de la linea: otra seccion
                    continue
                lat = float(geo.lateral_mm(d.cx, d.base_y))
                if abs(lat) > 900:                 # muy afuera: otro carril
                    continue
                if mejor is None or dist < mejor[0]:
                    mejor = (dist, lat, color, d)
        if descartados:
            self.info["tras_linea"] = descartados

        # --- no se ve ninguno: adelantando, o hay que ir a buscarlo ---------
        if mejor is None:
            return self._sin_pilar(ahora, dt)

        dist, lat, color, d_mejor = mejor

        # --- por que lado hay que pasarlo ----------------------------------
        # lado = +1 -> el carro pasa por la DERECHA del pilar (rojo)
        lado = 1 if color == "rojo" else -1
        if sentido < 0 and bool(self.cfg.get("invertir_en_antihorario", False)):
            lado = -lado

        semi_carro = float(geo.cfg.get("ancho_carro_mm", 200.0)) / 2.0
        semi_pilar = float(self.cfg.get("semi_pilar_mm", 25.0))
        # Despeje COMODO (el que se pide) y MINIMO (el que fisicamente hace
        # falta para no tocarlo). Si el hueco no da para el comodo se aprieta
        # hacia el minimo, pero jamas se cambia de lado.
        despeje = semi_carro + float(self.cfg.get("margen_mm", 70.0)) + semi_pilar
        minimo = semi_carro + semi_pilar + 15.0
        objetivo = lat + lado * despeje

        # --- recortar al pasillo libre a esa distancia ---------------------
        if perfil is not None and perfil.hay_muro:
            objetivo = self._recortar(objetivo, dist, perfil, geo, semi_carro,
                                      lat, lado, minimo)

        # --- a direccion ---------------------------------------------------
        modo = str(self.cfg.get("modo", "punto"))
        if modo == "borde":
            direccion = self._dir_borde(d_mejor, lado, dist, geo)
        else:
            # 'punto': se apunta al hueco al costado del pilar. La mirada
            # minima es lo que impide que el volante se vaya a tope al
            # acercarse: sin ella el angulo al punto crece sin limite.
            mirada = max(float(self.cfg.get("mirada_min_mm", 350.0)), dist)
            ang = math.degrees(math.atan2(objetivo, mirada))
            direccion = ang * float(self.cfg.get("k_dir", 1.4))
        tope = float(self.cfg.get("dir_max_pct", 55.0))
        direccion = max(-tope, min(tope, direccion))
        direccion = self._rampa(direccion, dt)

        t = (activar - dist) / max(1.0, activar - mandar)
        peso = float(self.cfg.get("peso_max", 0.8)) * min(1.0, max(0.0, t))

        # --- comprometerse a adelantarlo entero ----------------------------
        # SOLO cuando ya se le tiene encima: el compromiso existe para cubrir
        # el punto ciego de delante del carro, no para irse recto desde lejos.
        # Desde aqui hasta que la COLA pase el pilar no se vuelve hacia el.
        seg = 0.0
        if dist <= mandar:
            largo = float(geo.cfg.get("largo_carro_mm", 240.0))
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
                          "modo": modo,
                          "peso": round(peso, 2), "dir": round(direccion),
                          "compromiso_s": round(seg, 2)})
        return direccion, peso

    # ------------------------------------------------------------------
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
    def _sin_pilar(self, ahora: float, dt: float) -> Tuple[float, float]:
        """No se ve ningun pilar. Dos casos muy distintos:

        - Se perdio de CERCA: esta en el punto ciego de delante del carro.
          Toca el compromiso de adelantamiento (ir recto hasta que la cola
          pase), que es lo que evita que la rueda trasera lo barra.
        - Se perdio de LEJOS: se salio de cuadro por el canto porque el carro
          giro de mas. En modo 'borde' se corrige hacia el para volver a
          tenerlo en el canto, que es donde tiene que estar.
        """
        if ahora < self._comp_hasta:
            return self._compromiso(ahora, dt)

        if (str(self.cfg.get("modo", "punto")) == "borde"
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
        llega tan abajo). Mientras dure el compromiso se pide RECTO: no se
        corrige hacia el lado del pilar, que es como la rueda trasera lo
        barre. No se mantiene el giro de aproximacion, solo se impide volver."""
        if ahora >= self._comp_hasta:
            self._dir_prev = 0.0
            return 0.0, 0.0
        queda = self._comp_hasta - ahora
        peso = float(self.cfg.get("peso_compromiso", 0.7))
        direccion = self._rampa(0.0, dt)
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
                  minimo: float = 0.0) -> float:
        """Recorta el punto de paso al hueco libre SIN CAMBIARLO DE LADO.

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
        libre_min = dist + 250.0     # la pared debe quedar mas lejos que el pilar
        margen = semi_carro + 40.0
        # limites laterales del hueco a esa profundidad
        izq_lim, der_lim = -1500.0, 1500.0
        for c in range(0, p.ancho, 6):
            if not p.valido[c] or p.dist_mm[c] > libre_min:
                continue
            lat_c = float(geo.lateral_mm(c, max(1, int(p.y_contacto[c]))))
            if lat_c < 0:
                izq_lim = max(izq_lim, lat_c)
            else:
                der_lim = min(der_lim, lat_c)
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
