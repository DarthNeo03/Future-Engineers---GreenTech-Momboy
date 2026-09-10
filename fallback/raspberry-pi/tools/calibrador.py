#!/usr/bin/env python3
"""
calibrador.py — Calibrador HSV interactivo para el carro WRO Future Engineers.

    python tools/calibrador.py                  # camara 0
    python tools/calibrador.py --camara 1
    python tools/calibrador.py --imagen foto.jpg   # calibrar sobre una foto
    python tools/calibrador.py --listar-camaras

Como se usa
-----------
1. Elige el color arriba a la izquierda (rojo / verde / negro).
2. CLIC IZQUIERDO sobre el objeto en la imagen -> calcula el rango HSV solo.
   - Con "acumular" activado, cada clic AMPLIA la muestra (haz clic en la parte
     iluminada y en la sombreada del mismo pilar: el rango cubrira ambas).
   - Sin acumular, cada clic reemplaza la muestra.
3. Afina con los sliders. Cada slider tiene su casilla: puedes escribir el
   numero exacto y pulsar Enter.
4. Escribe un nombre y pulsa "Guardar perfil". Se guardan las ultimas 5
   calibraciones en config/colors.json; el programa principal lee el activo.

Teclas: Esc = salir · Espacio = pausa · s = guardar captura

Requisitos: opencv-python, numpy, y tkinter.
  Windows : tkinter viene con el instalador oficial de Python.
  Raspbian: sudo apt install python3-tk
  Pillow es opcional (mejora la vista); sin el usa ventanas de OpenCV.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

# --- permitir ejecutar desde cualquier carpeta ------------------------------
RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from src import camera, color_config as cc, vision  # noqa: E402

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception as e:  # pragma: no cover
    print("Falta tkinter.\n"
          "  Raspbian/Debian: sudo apt install python3-tk\n"
          "  Windows: reinstala Python marcando 'tcl/tk and IDLE'\n"
          f"Detalle: {e}", file=sys.stderr)
    raise SystemExit(1)

try:
    from PIL import Image, ImageTk
    HAY_PIL = True
except Exception:
    HAY_PIL = False

from widgets import Campo, MarcoScroll  # noqa: E402


# ===========================================================================
# Aplicacion
# ===========================================================================
class Calibrador:
    def __init__(self, args):
        self.args = args
        self.ruta_config = Path(args.config) if args.config else cc.RUTA_CONFIG_POR_DEFECTO
        self.datos = cc.cargar(self.ruta_config)
        perfil = cc.obtener(self.datos)
        self.colores: Dict[str, Dict[str, Any]] = {
            k: dict(v) for k, v in perfil["colores"].items()
        }
        self.color_actual = next(iter(self.colores))

        self.vision = vision.Vision(self.colores)
        self.muestras: Dict[str, List[np.ndarray]] = {}
        self.frame: Optional[np.ndarray] = None
        self.hsv: Optional[np.ndarray] = None
        self.pausado = False
        self._t_ultimo = time.time()
        self._fps = 0.0
        self._mapa_click = (1.0, 1, 0)   # escala, n_paneles, ancho_panel_mostrado
        self._campos: Dict[str, Campo] = {}        # parametros sueltos
        self._campos_rango: Dict[str, Campo] = {}  # sliders de los rangos HSV
        self._n_rangos_ui = 0

        # --- fuente de imagen -------------------------------------------
        self.cap = None
        self.imagen_fija = None
        if args.imagen:
            img = cv2.imread(str(args.imagen))
            if img is None:
                raise SystemExit(f"No se pudo leer la imagen {args.imagen}")
            self.imagen_fija = img
        else:
            cam = perfil.get("camara", {})
            self.cap = camera.abrir(
                indice=args.camara if args.camara is not None else cam.get("indice", 0),
                ancho=args.ancho or cam.get("ancho", 640),
                alto=args.alto or cam.get("alto", 480),
                fps=cam.get("fps", 30),
                fourcc=cam.get("fourcc", "MJPG"),
            )
            if self.cap is None:
                raise SystemExit("No se pudo abrir la camara. Prueba --listar-camaras "
                                 "o usa --imagen para calibrar sobre una foto.")

        self._construir_gui()
        self._reconstruir_panel_color()
        self.root.after(30, self._bucle)

    # ------------------------------------------------------------------
    # GUI
    # ------------------------------------------------------------------
    def _construir_gui(self):
        self.root = tk.Tk()
        self.root.title("Calibrador HSV — WRO Future Engineers")
        self.root.protocol("WM_DELETE_WINDOW", self.cerrar)
        try:
            ttk.Style().theme_use("clam")
        except Exception:
            pass

        # ---- barra superior: perfiles ---------------------------------
        top = ttk.Frame(self.root, padding=(6, 6, 6, 2))
        top.pack(side="top", fill="x")

        ttk.Label(top, text="Perfil:").pack(side="left")
        self.var_perfil = tk.StringVar(value=self.datos["activo"])
        self.combo_perfil = ttk.Combobox(top, textvariable=self.var_perfil, width=22,
                                         state="readonly", values=cc.listar(self.datos))
        self.combo_perfil.pack(side="left", padx=4)
        ttk.Button(top, text="Cargar", command=self.cargar_perfil).pack(side="left")
        ttk.Button(top, text="Activar", command=self.activar_perfil).pack(side="left", padx=(2, 10))

        ttk.Label(top, text="Guardar como:").pack(side="left")
        self.var_nombre = tk.StringVar(value=time.strftime("calib_%m%d_%H%M"))
        ttk.Entry(top, textvariable=self.var_nombre, width=18).pack(side="left", padx=4)
        ttk.Button(top, text="Guardar perfil", command=self.guardar_perfil).pack(side="left")
        ttk.Button(top, text="Borrar", command=self.borrar_perfil).pack(side="left", padx=(2, 0))

        # ---- cuerpo ----------------------------------------------------
        cuerpo = ttk.Frame(self.root)
        cuerpo.pack(side="top", fill="both", expand=True)

        izq = ttk.Frame(cuerpo, padding=(6, 4))
        izq.pack(side="left", fill="y")

        cab = ttk.Frame(izq)
        cab.pack(fill="x")
        ttk.Label(cab, text="Color:").pack(side="left")
        self.var_color = tk.StringVar(value=self.color_actual)
        self.combo_color = ttk.Combobox(cab, textvariable=self.var_color, width=12,
                                        state="readonly", values=list(self.colores))
        self.combo_color.pack(side="left", padx=4)
        self.combo_color.bind("<<ComboboxSelected>>", lambda e: self.cambiar_color())
        ttk.Button(cab, text="Reset", width=6,
                   command=self.reset_color).pack(side="left")

        self.panel = MarcoScroll(izq)
        self.panel.pack(fill="both", expand=True, pady=4)

        # ---- derecha: video -------------------------------------------
        der = ttk.Frame(cuerpo, padding=(4, 4))
        der.pack(side="left", fill="both", expand=True)

        barra = ttk.Frame(der)
        barra.pack(fill="x")
        ttk.Label(barra, text="Vista:").pack(side="left")
        self.var_vista = tk.StringVar(value="ambas")
        for txt, val in (("Camara", "camara"), ("Mascara", "mascara"), ("Ambas", "ambas")):
            ttk.Radiobutton(barra, text=txt, value=val,
                            variable=self.var_vista).pack(side="left")
        self.var_solo_color = tk.BooleanVar(value=True)
        ttk.Checkbutton(barra, text="solo este color", variable=self.var_solo_color).pack(
            side="left", padx=(10, 0))
        self.var_acumular = tk.BooleanVar(value=True)
        ttk.Checkbutton(barra, text="acumular clics", variable=self.var_acumular).pack(side="left")
        ttk.Button(barra, text="Limpiar muestra", command=self.limpiar_muestras).pack(side="left")
        ttk.Button(barra, text="Captura", command=self.guardar_captura).pack(side="left", padx=4)

        toma = ttk.LabelFrame(der, text="Toma de color por clic", padding=4)
        toma.pack(fill="x", pady=(4, 2))
        self.c_radio = Campo(toma, "radio px", 0, 25, 4, ancho_etiqueta=9)
        self.c_mh = Campo(toma, "margen H", 0, 40, 8, ancho_etiqueta=9)
        self.c_ms = Campo(toma, "margen S", 0, 120, 45, ancho_etiqueta=9)
        self.c_mv = Campo(toma, "margen V", 0, 120, 50, ancho_etiqueta=9)
        for i, c in enumerate((self.c_radio, self.c_mh, self.c_ms, self.c_mv)):
            c.grid(row=i // 2, column=i % 2, sticky="w", padx=(0, 14))

        if HAY_PIL:
            self.lienzo = tk.Label(der, background="#222")
            self.lienzo.pack(fill="both", expand=True)
            self.lienzo.bind("<Button-1>", self._clic_tk)
        else:
            self.lienzo = None
            ttk.Label(der, foreground="#a60",
                      text="Pillow no instalado: el video sale en ventanas de OpenCV.\n"
                           "Haz clic sobre la ventana 'Calibrador'.\n"
                           "(pip install pillow para verlo aqui dentro)").pack(pady=20)
            try:
                cv2.namedWindow("Calibrador", cv2.WINDOW_NORMAL)
                cv2.setMouseCallback("Calibrador", self._clic_cv2)
            except cv2.error:
                raise SystemExit(
                    "Tu OpenCV no tiene ventanas (paquete 'headless') y tampoco\n"
                    "hay Pillow. Instala uno de los dos:\n"
                    "   pip install pillow\n"
                    "   pip install opencv-python   (en vez de opencv-python-headless)")

        self.estado = ttk.Label(self.root, anchor="w", relief="sunken", padding=(6, 2))
        self.estado.pack(side="bottom", fill="x")

        self.root.bind("<Escape>", lambda e: self.cerrar())
        self.root.bind("<space>", lambda e: self.alternar_pausa())
        self.root.bind("s", lambda e: self.guardar_captura())

    # ------------------------------------------------------------------
    def _p(self) -> Dict[str, Any]:
        return self.colores[self.color_actual]

    def _reconstruir_panel_color(self):
        """Crea los sliders del color activo (el numero de rangos varia)."""
        for w in self.panel.interior.winfo_children():
            w.destroy()
        self._campos.clear()
        self._campos_rango.clear()
        p = cc.normalizar_color(self._p())
        self.colores[self.color_actual] = p
        self._n_rangos_ui = len(p["rangos"])
        cont = self.panel.interior

        # --- rangos HSV --------------------------------------------------
        marco_r = ttk.LabelFrame(cont, text="Rangos HSV", padding=4)
        marco_r.pack(fill="x", pady=2)
        for k, (bajo, alto) in enumerate(p["rangos"]):
            sub = ttk.LabelFrame(marco_r, text=f"rango {k + 1}", padding=3)
            sub.pack(fill="x", pady=2)
            for j, (nom, tope) in enumerate((("H", 179), ("S", 255), ("V", 255))):
                for lim, arr in (("min", bajo), ("max", alto)):
                    clave = f"r{k}_{j}_{lim}"
                    campo = Campo(sub, f"{nom} {lim}", 0, tope, arr[j],
                                  al_cambiar=self._hacer_setter_rango(k, j, lim))
                    campo.pack(fill="x")
                    self._campos_rango[clave] = campo
        botones = ttk.Frame(marco_r)
        botones.pack(fill="x", pady=(3, 0))
        ttk.Button(botones, text="+ rango", width=9,
                   command=lambda: self._cambiar_num_rangos(+1)).pack(side="left")
        ttk.Button(botones, text="- rango", width=9,
                   command=lambda: self._cambiar_num_rangos(-1)).pack(side="left", padx=3)

        # --- limpieza ----------------------------------------------------
        marco_m = ttk.LabelFrame(cont, text="Limpieza de la mascara", padding=4)
        marco_m.pack(fill="x", pady=2)
        for clave, etiqueta, mx, res, dec in (
            ("desenfoque", "desenfoque", 15, 1, 0),
            ("abrir", "abrir", 21, 1, 0),
            ("cerrar", "cerrar", 21, 1, 0),
            ("unir_huecos", "unir huecos", 30, 1, 0),
        ):
            campo = Campo(marco_m, etiqueta, 0, mx, p[clave], resolucion=res,
                          decimales=dec, al_cambiar=self._hacer_setter(clave))
            campo.pack(fill="x")
            self._campos[clave] = campo

        # --- filtros -----------------------------------------------------
        marco_f = ttk.LabelFrame(cont, text="Filtros del objeto", padding=4)
        marco_f.pack(fill="x", pady=2)
        for clave, etiqueta, mn, mx, res, dec in (
            ("area_min", "area min", 0, 20000, 25, 0),
            ("area_max", "area max", 100, 300000, 500, 0),
            ("llenado_min", "llenado min", 0.0, 1.0, 0.01, 2),
            ("aspecto_min", "aspecto min", 0.0, 10.0, 0.05, 2),
            ("aspecto_max", "aspecto max", 0.0, 20.0, 0.05, 2),
            ("ancho_min", "ancho min px", 0, 200, 1, 0),
            ("alto_min", "alto min px", 0, 200, 1, 0),
            ("roi_arriba", "ROI arriba", 0.0, 0.99, 0.01, 2),
            ("roi_abajo", "ROI abajo", 0.01, 1.0, 0.01, 2),
            ("max_objetos", "max objetos", 1, 20, 1, 0),
        ):
            campo = Campo(marco_f, etiqueta, mn, mx, p[clave], resolucion=res,
                          decimales=dec, al_cambiar=self._hacer_setter(clave))
            campo.pack(fill="x")
            self._campos[clave] = campo

        self.var_usar_asp = tk.BooleanVar(value=bool(p["usar_aspecto"]))
        ttk.Checkbutton(marco_f, text="usar filtro de aspecto",
                        variable=self.var_usar_asp,
                        command=lambda: self._fijar("usar_aspecto",
                                                    self.var_usar_asp.get())).pack(anchor="w")

    def _hacer_setter(self, clave):
        def setter(valor):
            self._fijar(clave, valor)
        return setter

    def _hacer_setter_rango(self, k, j, lim):
        def setter(valor):
            p = self._p()
            try:
                idx = 0 if lim == "min" else 1
                p["rangos"][k][idx][j] = int(valor)
            except (IndexError, KeyError):
                return
            self.vision.actualizar(self.colores)
        return setter

    def _fijar(self, clave, valor):
        self._p()[clave] = valor
        self.vision.actualizar(self.colores)

    def _cambiar_num_rangos(self, delta: int):
        p = self._p()
        r = p["rangos"]
        if delta > 0 and len(r) < 4:
            r.append([[0, 0, 0], [179, 255, 255]])
        elif delta < 0 and len(r) > 1:
            r.pop()
        else:
            return
        self._reconstruir_panel_color()
        self.vision.actualizar(self.colores)

    def _refrescar_campos(self):
        """Vuelca los valores del dict a los widgets (tras un clic o cargar)."""
        p = cc.normalizar_color(self._p())
        self.colores[self.color_actual] = p
        if len(p["rangos"]) != self._n_rangos_ui:
            self._reconstruir_panel_color()
            return
        for k, (bajo, alto) in enumerate(p["rangos"]):
            for j in range(3):
                self._campos_rango[f"r{k}_{j}_min"].fijar(bajo[j])
                self._campos_rango[f"r{k}_{j}_max"].fijar(alto[j])
        for clave, campo in self._campos.items():
            if clave in p:
                campo.fijar(p[clave])
        if hasattr(self, "var_usar_asp"):
            self.var_usar_asp.set(bool(p["usar_aspecto"]))
        self.vision.actualizar(self.colores)

    # ------------------------------------------------------------------
    # Perfiles
    # ------------------------------------------------------------------
    def cargar_perfil(self):
        perfil = cc.obtener(self.datos, self.var_perfil.get())
        self.colores = {k: dict(v) for k, v in perfil["colores"].items()}
        if self.color_actual not in self.colores:
            self.color_actual = next(iter(self.colores))
        self.combo_color.configure(values=list(self.colores))
        self.var_color.set(self.color_actual)
        self.vision = vision.Vision(self.colores)
        self.limpiar_muestras()
        self._reconstruir_panel_color()
        self._msg(f"Perfil '{perfil['nombre']}' cargado")

    def activar_perfil(self):
        cc.fijar_activo(self.datos, self.var_perfil.get())
        cc.guardar(self.datos, self.ruta_config)
        self._msg(f"Perfil activo: {self.datos['activo']}")

    def guardar_perfil(self):
        nombre = self.var_nombre.get().strip()
        if not nombre:
            messagebox.showwarning("Nombre vacio", "Escribe un nombre para el perfil.")
            return
        camara = {}
        if self.cap is not None:
            camara = {
                "indice": self.args.camara if self.args.camara is not None else 0,
                "ancho": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "alto": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "fps": int(self.cap.get(cv2.CAP_PROP_FPS) or 30),
                "fourcc": "MJPG",
            }
        cc.guardar_perfil(self.datos, nombre, self.colores, camara,
                          notas="calibrado con tools/calibrador.py")
        cc.guardar(self.datos, self.ruta_config)
        self.combo_perfil.configure(values=cc.listar(self.datos))
        self.var_perfil.set(nombre)
        self._msg(f"Guardado '{nombre}' ({len(self.datos['perfiles'])}/{cc.MAX_PERFILES}) "
                  f"en {self.ruta_config}")

    def borrar_perfil(self):
        nombre = self.var_perfil.get()
        if not messagebox.askyesno("Borrar", f"¿Borrar el perfil '{nombre}'?"):
            return
        cc.borrar_perfil(self.datos, nombre)
        cc.guardar(self.datos, self.ruta_config)
        self.combo_perfil.configure(values=cc.listar(self.datos))
        self.var_perfil.set(self.datos["activo"])
        self._msg(f"Perfil '{nombre}' borrado")

    def reset_color(self):
        base = cc.colores_por_defecto()
        self.colores[self.color_actual] = base.get(
            self.color_actual, cc.normalizar_color({}))
        self.limpiar_muestras()
        self._reconstruir_panel_color()
        self.vision.actualizar(self.colores)

    def cambiar_color(self):
        self.color_actual = self.var_color.get()
        self._reconstruir_panel_color()

    # ------------------------------------------------------------------
    # Toma de color
    # ------------------------------------------------------------------
    def limpiar_muestras(self):
        self.muestras[self.color_actual] = []
        self._msg("Muestra vaciada")

    def _clic_tk(self, e):
        self._tomar(e.x, e.y)

    def _clic_cv2(self, evento, x, y, _flags, _param):
        if evento == cv2.EVENT_LBUTTONDOWN:
            self._tomar(x, y)

    def _tomar(self, dx: int, dy: int):
        if self.hsv is None:
            return
        escala, n_paneles, ancho_panel = self._mapa_click
        if ancho_panel > 0:
            panel = min(int(dx // ancho_panel), n_paneles - 1)
            dx -= panel * ancho_panel
        x = int(dx / escala)
        y = int(dy / escala)
        h, w = self.hsv.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            return

        r = int(self.c_radio.obtener())
        x0, x1 = max(0, x - r), min(w, x + r + 1)
        y0, y1 = max(0, y - r), min(h, y + r + 1)
        ancla = self.hsv[y, x]
        mh, ms, mv = (int(self.c_mh.obtener()), int(self.c_ms.obtener()),
                      int(self.c_mv.obtener()))
        # El pixel clicado manda: descartamos del parche lo que no se le parezca
        # (borde del objeto, sombra, brillo blanco).
        parche = vision.nucleo_de_parche(
            self.hsv[y0:y1, x0:x1], ancla,
            tol_h=max(8, mh + 4), tol_s=max(60, ms + 20), tol_v=max(60, mv + 20))

        lista = self.muestras.setdefault(self.color_actual, [])
        if not self.var_acumular.get():
            lista.clear()
        lista.append(parche)

        pix = np.concatenate(lista, axis=0)
        rangos = vision.rangos_desde_pixeles(pix, margen_h=mh, margen_s=ms, margen_v=mv)
        self._p()["rangos"] = rangos
        self._refrescar_campos()
        brutos = (x1 - x0) * (y1 - y0)
        self._msg(f"Clic ({x},{y})  HSV={tuple(int(v) for v in ancla)}  "
                  f"utiles {parche.shape[0]}/{brutos} px  "
                  f"muestra total={pix.shape[0]} px  rangos={len(rangos)}")

    # ------------------------------------------------------------------
    # Bucle
    # ------------------------------------------------------------------
    def alternar_pausa(self):
        self.pausado = not self.pausado
        self._msg("PAUSA" if self.pausado else "reanudado")

    def guardar_captura(self):
        if self.frame is None:
            return
        carpeta = RAIZ / "capturas"
        carpeta.mkdir(exist_ok=True)
        ruta = carpeta / time.strftime("cap_%Y%m%d_%H%M%S.png")
        cv2.imwrite(str(ruta), self.frame)
        self._msg(f"Captura guardada: {ruta}")

    def _msg(self, texto: str):
        self.estado.configure(text=texto)

    def _bucle(self):
        try:
            self._paso()
        except Exception as e:  # no matar la GUI por un frame malo
            self._msg(f"Error: {e}")
        self.root.after(15, self._bucle)

    def _paso(self):
        if not self.pausado or self.frame is None:
            if self.imagen_fija is not None:
                self.frame = self.imagen_fija.copy()
            else:
                ok, f = self.cap.read()
                if not ok:
                    self._msg("Sin frame de la camara")
                    return
                self.frame = f
            self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        solo = [self.color_actual] if self.var_solo_color.get() else None
        dets, masks = self.vision.procesar(self.frame, solo=solo)

        salida = self.frame.copy()
        for nombre, lista in dets.items():
            vision.dibujar_detecciones(salida, lista,
                                       self.colores[nombre]["color_dibujo"])
        p = self._p()
        vision.dibujar_roi(salida, p["roi_arriba"], p["roi_abajo"])

        vista = self.var_vista.get()
        mask = masks.get(self.color_actual)
        if mask is None:
            mask = vision.combinar_mascaras(masks)
        if mask is None:
            mask = np.zeros(self.frame.shape[:2], np.uint8)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        if vista == "camara":
            paneles = [salida]
        elif vista == "mascara":
            paneles = [mask_bgr]
        else:
            paneles = [salida, mask_bgr]
        compuesto = np.hstack(paneles) if len(paneles) > 1 else paneles[0]

        ahora = time.time()
        dt = ahora - self._t_ultimo
        self._t_ultimo = ahora
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        n = len(dets.get(self.color_actual, []))
        cv2.putText(compuesto, f"{self._fps:4.1f} FPS  |  {self.color_actual}: {n} obj",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        self._mostrar(compuesto, len(paneles))

    def _mostrar(self, img: np.ndarray, n_paneles: int):
        alto_max = self.args.alto_vista
        escala = 1.0
        if img.shape[0] > alto_max:
            escala = alto_max / img.shape[0]
            img = cv2.resize(img, None, fx=escala, fy=escala,
                             interpolation=cv2.INTER_AREA)
        self._mapa_click = (escala, n_paneles, img.shape[1] // n_paneles)

        if HAY_PIL and self.lienzo is not None:
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            tkimg = ImageTk.PhotoImage(Image.fromarray(rgb))
            self.lienzo.configure(image=tkimg)
            self.lienzo.image = tkimg  # evitar que el GC se lo lleve
        else:
            try:
                cv2.imshow("Calibrador", img)
                cv2.waitKey(1)
            except cv2.error:
                pass

    # ------------------------------------------------------------------
    def cerrar(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass          # build 'headless' de OpenCV: no hay ventanas que cerrar
        self.root.destroy()

    def ejecutar(self):
        self.root.mainloop()


# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="Calibrador HSV WRO Future Engineers")
    ap.add_argument("--camara", type=int, default=None, help="indice de la camara")
    ap.add_argument("--ancho", type=int, default=None)
    ap.add_argument("--alto", type=int, default=None)
    ap.add_argument("--imagen", type=str, default=None,
                    help="calibrar sobre una foto en vez de la camara")
    ap.add_argument("--config", type=str, default=None, help="ruta de colors.json")
    ap.add_argument("--alto-vista", type=int, default=380,
                    help="alto maximo del video en pantalla (subelo si tu monitor es grande)")
    ap.add_argument("--listar-camaras", action="store_true")
    args = ap.parse_args()

    if args.listar_camaras:
        print("Camaras detectadas:", camera.listar())
        return

    Calibrador(args).ejecutar()


if __name__ == "__main__":
    main()
