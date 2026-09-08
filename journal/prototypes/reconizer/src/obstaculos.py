"""
obstaculos.py — Esquivar los pilares rojos y verdes.

REGLA DE LA COMPETENCIA
    pilar ROJO  -> se pasa por su DERECHA  (el pilar queda a la izquierda del carro)
    pilar VERDE -> se pasa por su IZQUIERDA (el pilar queda a la derecha del carro)

Como se decide hacia donde ir:

1. De todos los pilares detectados se elige el MAS CERCANO (el que tiene el
   borde inferior mas abajo en la imagen). Los de mas atras solo se usan para
   ir preparando el siguiente movimiento.
2. Se calcula un punto objetivo al lado que toca del pilar, separado medio
   ancho de carro MAS un margen. Ese medio ancho no es una constante: se saca
   de la perspectiva, igual que en la busqueda de huecos, porque el mismo pilar
   a dos metros ocupa una cuarta parte de los pixeles que a medio metro.
3. El objetivo se RECORTA al pasillo libre que ve el perfil del muro. Sin eso,
   esquivar un pilar pegado a la pared manda el carro contra la pared: el pilar
   dice "pasa por aqui" y la pared dice "por ahi no cabes", y gana la pared.
4. El peso de la esquiva sube segun se acerca el pilar. De lejos apenas
   corrige; de cerca manda ella y la navegacion del muro pasa a segundo plano.
   La capa de seguridad (frenado y escape) sigue corriendo por encima de todo.

Se puede apagar entera desde la interfaz sin tocar nada mas.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .navegacion import DER, IZQ, PerfilMuro, ancho_carro_px, _lim, _PD

# A que lado del pilar tiene que pasar el carro
LADO_OBLIGATORIO = {"rojo": DER, "verde": IZQ}
COLOR_DIBUJO = {"rojo": (0, 0, 255), "verde": (0, 200, 0)}


@dataclass
class Pilar:
    color: str
    x: int
    y: int
    w: int
    h: int
    area: int

    @property
    def base_y(self) -> int:
        return self.y + self.h

    @property
    def cx(self) -> float:
        return self.x + self.w / 2.0

    @property
    def lado(self) -> int:
        """+1 = el carro pasa por la derecha del pilar."""
        return LADO_OBLIGATORIO.get(self.color, DER)

    @property
    def nombre_lado(self) -> str:
        return "derecha" if self.lado == DER else "izquierda"


@dataclass
class ResultadoEsquiva:
    activo: bool = False
    direccion: float = 0.0
    peso: float = 0.0
    motivo: str = ""
    objetivo_x: Optional[int] = None
    pilar: Optional[Pilar] = None
    siguiente: Optional[Pilar] = None
    recortado: bool = False
    objetivo_bruto: Optional[int] = None


class EsquivaPilares:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.pd = _PD()
        self.reiniciar()

    def reiniciar(self) -> None:
        self.pd.reiniciar()
        self._activo: Optional[Pilar] = None
        self._perdido = 0
        self._ultimo: ResultadoEsquiva = ResultadoEsquiva()
        self._t_soltado = 0.0
        self.pasados = 0

    # ------------------------------------------------------------------
    def _candidatos(self, dets: Dict[str, List[Any]], alto: int, ancho: int) -> List[Pilar]:
        area_min = int(self.cfg.get("area_min_pilar", 300))
        desde = float(self.cfg.get("activar_desde", 0.45)) * alto
        pilares: List[Pilar] = []
        for color in ("rojo", "verde"):
            for d in dets.get(color, []) or []:
                if d.area < area_min:
                    continue
                p = Pilar(color=color, x=int(d.x), y=int(d.y), w=int(d.w),
                          h=int(d.h), area=int(d.area))
                if p.base_y < desde:
                    continue            # todavia muy lejos para hacerle caso
                pilares.append(p)
        pilares.sort(key=lambda p: -p.base_y)      # el mas cercano primero
        return pilares

    def _seguir(self, pilares: List[Pilar], alto: int) -> Optional[Pilar]:
        """Mantiene el mismo pilar entre frames para no cambiar de idea a mitad."""
        if self._activo is not None:
            tol = float(self.cfg.get("tolerancia_seguimiento", 120))
            iguales = [p for p in pilares
                       if p.color == self._activo.color
                       and abs(p.cx - self._activo.cx) <= tol]
            if iguales:
                self._activo = max(iguales, key=lambda p: p.base_y)
                self._perdido = 0
                return self._activo
            self._perdido += 1
            if self._perdido <= int(self.cfg.get("frames_perdido", 6)):
                return self._activo          # parpadeo de la deteccion
            self._soltar()

        if pilares and (time.time() - self._t_soltado) > \
                float(self.cfg.get("refractario_ms", 350)) / 1000.0:
            self._activo = pilares[0]
            self._perdido = 0
        return self._activo

    def _soltar(self) -> None:
        if self._activo is not None:
            self.pasados += 1
        self._activo = None
        self._perdido = 0
        self._t_soltado = time.time()
        self.pd.reiniciar()

    # ------------------------------------------------------------------
    def paso(self, dets: Dict[str, List[Any]], perfil: PerfilMuro,
             cfg_nav: Dict[str, Any], ahora: Optional[float] = None) -> ResultadoEsquiva:
        ahora = ahora if ahora is not None else time.time()
        H, W = perfil.alto, perfil.ancho

        if not bool(self.cfg.get("activo", False)):
            self._activo = None
            self._ultimo = ResultadoEsquiva(motivo="esquiva apagada")
            return self._ultimo

        pilares = self._candidatos(dets, H, W)
        activo = self._seguir(pilares, H)
        if activo is None:
            self._ultimo = ResultadoEsquiva(motivo="sin pilares")
            return self._ultimo

        # --- ya lo hemos pasado? -----------------------------------------
        soltar_en = float(self.cfg.get("soltar_en", 0.93)) * H
        borde = float(self.cfg.get("borde_soltar", 0.06)) * W
        if activo.base_y >= soltar_en or activo.cx <= borde or activo.cx >= W - borde:
            self._soltar()
            self._ultimo = ResultadoEsquiva(motivo="pilar superado")
            return self._ultimo

        # --- punto por el que hay que pasar -------------------------------
        # Medio ancho de carro a la distancia del pilar, mas margen.
        medio = ancho_carro_px(float(activo.base_y), cfg_nav, H, W) / 2.0
        margen = medio * float(self.cfg.get("margen_lateral", 1.25))
        if activo.lado == DER:
            objetivo = activo.x + activo.w + margen
        else:
            objetivo = activo.x - margen

        # --- recortar al pasillo libre ------------------------------------
        objetivo_bruto = objetivo
        objetivo, recortado = self._recortar(objetivo, perfil, medio)

        err = (objetivo - W / 2.0) / (W / 2.0)
        direccion = self.pd.paso(err, float(self.cfg.get("kp", 115.0)),
                                 float(self.cfg.get("kd", 22.0)), ahora)

        # --- cuanto manda esta esquiva ------------------------------------
        desde = float(self.cfg.get("activar_desde", 0.45))
        pleno = float(self.cfg.get("mandar_desde", 0.68))
        frac = activo.base_y / float(H)
        peso = _lim((frac - desde) / max(0.01, pleno - desde), 0.0, 1.0)
        peso *= _lim(float(self.cfg.get("peso_max", 1.0)), 0.0, 1.0)

        # --- el siguiente pilar, para ir preparandose ----------------------
        siguiente = pilares[1] if len(pilares) > 1 else None
        if siguiente is not None and siguiente.lado != activo.lado:
            # Ya sabemos que despues hay que cruzar al otro lado: se mete un
            # sesgo pequeno para no llegar completamente descolocado.
            sesgo = float(self.cfg.get("sesgo_siguiente", 12.0)) * siguiente.lado
            direccion += sesgo * (1.0 - peso)

        self._ultimo = ResultadoEsquiva(
            activo=True, direccion=direccion, peso=peso,
            motivo=(f"{activo.color} por la {activo.nombre_lado}"
                    f" x->{int(objetivo)}{' (recortado)' if recortado else ''}"),
            objetivo_x=int(objetivo), pilar=activo, siguiente=siguiente,
            recortado=recortado)
        self._ultimo.objetivo_bruto = int(objetivo_bruto)
        return self._ultimo

    # ------------------------------------------------------------------
    def _recortar(self, objetivo: float, perfil: PerfilMuro,
                  medio: float) -> Tuple[float, bool]:
        """Mete el objetivo dentro de un tramo por el que de verdad se pasa.

        Si el pilar esta pegado a la pared, el punto ideal cae dentro del muro.
        Aqui se busca el hueco pasable mas cercano a ese punto y se recorta
        dentro de el dejando medio carro de margen a cada lado.
        """
        W = perfil.ancho
        objetivo = _lim(objetivo, 2.0, W - 3.0)
        huecos = [h for h in perfil.huecos if h.pasable]
        if not huecos:
            return objetivo, False

        dentro = [h for h in huecos if h.x0 <= objetivo <= h.x1]
        if dentro:
            h = dentro[0]
            lo, hi = h.x0 + medio, h.x1 - medio
            if lo > hi:
                return float(h.centro), True
            nuevo = _lim(objetivo, lo, hi)
            return nuevo, abs(nuevo - objetivo) > 1.0

        # fuera de todo hueco: nos vamos al borde del hueco mas proximo
        h = min(huecos, key=lambda k: min(abs(objetivo - k.x0), abs(objetivo - k.x1)))
        lo, hi = h.x0 + medio, h.x1 - medio
        if lo > hi:
            return float(h.centro), True
        return _lim(objetivo, lo, hi), True

    # ------------------------------------------------------------------
    def estado(self) -> Dict[str, Any]:
        r = self._ultimo
        d: Dict[str, Any] = {
            "activo": bool(self.cfg.get("activo", False)),
            "siguiendo": r.activo,
            "peso": round(r.peso, 2),
            "motivo": r.motivo,
            "pasados": self.pasados,
        }
        if r.pilar is not None:
            d["pilar"] = {"color": r.pilar.color, "lado": r.pilar.nombre_lado,
                          "cx": int(r.pilar.cx), "base_y": r.pilar.base_y,
                          "area": r.pilar.area}
            d["objetivo_x"] = r.objetivo_x
            d["recortado"] = r.recortado
        if r.siguiente is not None:
            d["siguiente"] = {"color": r.siguiente.color,
                              "lado": r.siguiente.nombre_lado}
        return d


# ---------------------------------------------------------------------------
def dibujar_pilares(frame: np.ndarray, dets: Dict[str, List[Any]],
                    r: ResultadoEsquiva) -> np.ndarray:
    """Pinta los pilares, por que lado hay que pasarlos y el punto objetivo."""
    H, W = frame.shape[:2]
    for color in ("rojo", "verde"):
        c = COLOR_DIBUJO[color]
        for d in dets.get(color, []) or []:
            cv2.rectangle(frame, (d.x, d.y), (d.x + d.w, d.y + d.h), c, 2)
            lado = LADO_OBLIGATORIO.get(color, DER)
            flecha_x = d.x + d.w + 18 if lado == DER else d.x - 18
            y = d.y + d.h // 2
            cv2.arrowedLine(frame, (int(d.x + d.w / 2), y), (int(flecha_x), y),
                            c, 2, tipLength=0.4)

    if r.activo and r.pilar is not None:
        p = r.pilar
        cv2.rectangle(frame, (p.x, p.y), (p.x + p.w, p.y + p.h),
                      COLOR_DIBUJO[p.color], 3)
        if r.objetivo_x is not None:
            x = int(r.objetivo_x)
            cv2.line(frame, (x, p.base_y - 26), (x, H - 14), (255, 255, 0), 2)
            cv2.circle(frame, (x, p.base_y), 6, (255, 255, 0), 2)
            if r.recortado and getattr(r, "objetivo_bruto", None) is not None:
                xb = int(r.objetivo_bruto)
                cv2.line(frame, (xb, p.base_y - 12), (xb, p.base_y + 12),
                         (120, 120, 120), 1)
        cv2.putText(frame, f"{p.color} por la {p.nombre_lado}  peso {r.peso:.2f}",
                    (8, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    COLOR_DIBUJO[p.color], 1, cv2.LINE_AA)
    return frame
