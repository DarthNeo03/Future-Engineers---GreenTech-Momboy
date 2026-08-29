#!/usr/bin/env python3
"""
panel.py — Panel de pruebas de escritorio (Tkinter).

Se abre desde main.py. Muestra lo mismo que carrito.local pero en la maquina
donde corre el programa, y con los parametros finos que en el movil estorban.

    ARMAR / PARAR grandes y siempre visibles
    velocidad maxima, crucero y giro
    estrategia y ganancias del PD, en caliente
    umbrales de giro, freno y parada
    lineas de las ruedas y franja del chasis
    telemetria del ESP32 y del giroscopio

Tambien se puede lanzar suelto para trastear sin hardware:
    python3 tools/panel.py --simulado --imagen capturas/pista.png
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
for p in (str(RAIZ), str(Path(__file__).resolve().parent)):
    if p not in sys.path:
        sys.path.insert(0, p)

import tkinter as tk                     # noqa: E402
from tkinter import ttk                  # noqa: E402

from widgets import Campo, MarcoScroll   # noqa: E402

try:
    from PIL import Image, ImageTk
    HAY_PIL = True
except Exception:
    HAY_PIL = False


class Panel:
    def __init__(self, robot, alto_vista: int = 380):
        self.r = robot
        self.alto_vista = alto_vista
        self.campos: Dict[str, Campo] = {}
        self._construir()

    # ------------------------------------------------------------------
    def _construir(self):
        self.root = tk.Tk()
        self.root.title("Carro WRO — panel de pruebas")
        self.root.protocol("WM_DELETE_WINDOW", self.cerrar)
        try:
            ttk.Style().theme_use("clam")
        except Exception:
            pass

        # ---- barra de seguridad, siempre arriba -------------------------
        barra = ttk.Frame(self.root, padding=6)
        barra.pack(side="top", fill="x")

        self.btn_armar = tk.Button(barra, text="ARMAR", font=("Arial", 13, "bold"),
                                   bg="#1a7f37", fg="white", width=26, height=2,
                                   command=self.alternar_armado)
        self.btn_armar.pack(side="left", padx=(0, 6))
        tk.Button(barra, text="PARAR", font=("Arial", 13, "bold"),
                  bg="#c62828", fg="white", width=12, height=2,
                  command=self.r.emergencia).pack(side="left", padx=(0, 12))

        self.var_modo = tk.StringVar(value=self.r.modo)
        for txt, val in (("Auto", "auto"), ("Manual", "manual"), ("Parado", "parado")):
            ttk.Radiobutton(barra, text=txt, value=val, variable=self.var_modo,
                            command=lambda: self.r.fijar_modo(self.var_modo.get())
                            ).pack(side="left")

        self.var_vista = tk.StringVar(value="camara")
        ttk.Label(barra, text="   Vista:").pack(side="left")
        for txt, val in (("Anotada", "camara"), ("Mascara", "mascara")):
            ttk.Radiobutton(barra, text=txt, value=val,
                            variable=self.var_vista).pack(side="left")

        # ---- cuerpo -----------------------------------------------------
        cuerpo = ttk.Frame(self.root)
        cuerpo.pack(fill="both", expand=True)

        izq = ttk.Frame(cuerpo, padding=(6, 2))
        izq.pack(side="left", fill="y")
        self.panel = MarcoScroll(izq)
        self.panel.pack(fill="both", expand=True)
        self._construir_parametros(self.panel.interior)

        der = ttk.Frame(cuerpo, padding=(4, 2))
        der.pack(side="left", fill="both", expand=True)

        if HAY_PIL:
            self.lienzo = tk.Label(der, background="#111")
            self.lienzo.pack(fill="both", expand=True)
        else:
            self.lienzo = None
            ttk.Label(der, text="Instala pillow para ver el video aqui\n"
                                "(pip install pillow). Mientras tanto usa "
                                "http://carrito.local:8080/").pack(pady=30)

        self.txt = tk.Text(der, height=8, bg="#111", fg="#ccc", font=("Consolas", 8))
        self.txt.pack(fill="x")

        self.estado_lbl = ttk.Label(self.root, anchor="w", relief="sunken", padding=(6, 2))
        self.estado_lbl.pack(side="bottom", fill="x")

        self.root.bind("<Escape>", lambda e: self.r.emergencia())
        self.root.bind("<space>", lambda e: self.alternar_armado())
        self.root.after(50, self._refrescar)

    # ------------------------------------------------------------------
    def _campo(self, padre, clave, seccion, etiqueta, mn, mx, res=1, dec=0):
        origen = self.r.cfg[seccion]
        c = Campo(padre, etiqueta, mn, mx, origen.get(clave, mn), resolucion=res,
                  decimales=dec, ancho_etiqueta=13,
                  al_cambiar=lambda v, k=clave, s=seccion: self._fijar(s, k, v))
        c.pack(fill="x")
        self.campos[f"{seccion}.{clave}"] = c
        return c

    def _fijar(self, seccion, clave, valor):
        self.r.cfg[seccion][clave] = valor
        self.r.aplicar_config()

    def _construir_parametros(self, cont):
        f = ttk.LabelFrame(cont, text="Velocidad", padding=4)
        f.pack(fill="x", pady=2)
        self._campo(f, "vmax", "limites", "vmax (PWM)", 0, 255)
        self._campo(f, "vel_crucero", "limites", "crucero %", 0, 100)
        self._campo(f, "vel_giro", "limites", "en giro %", 0, 100)
        self._campo(f, "dir_max", "limites", "dir max %", 0, 100)

        f = ttk.LabelFrame(cont, text="Mezcla de navegaciones", padding=4)
        f.pack(fill="x", pady=2)
        ttk.Label(f, text="Pesos: se suman las tres a la vez",
                  foreground="#555").pack(anchor="w")
        self.campos_mezcla = {}
        for clave, etiqueta in (("centrado", "centrado"), ("pared", "pared ext."),
                                ("hueco", "hueco")):
            c = Campo(f, etiqueta, 0.0, 1.0,
                      float((self.r.cfg["navegacion"].get("mezcla") or {}).get(clave, 0.0)),
                      resolucion=0.05, decimales=2, ancho_etiqueta=11,
                      al_cambiar=lambda v, k=clave: self._peso(k, v))
            c.pack(fill="x")
            self.campos_mezcla[clave] = c
        botones_m = ttk.Frame(f)
        botones_m.pack(fill="x", pady=2)
        for clave in ("centrado", "pared", "hueco"):
            ttk.Button(botones_m, text=f"solo {clave}", width=11,
                       command=lambda k=clave: self._solo(k)).pack(side="left", padx=1)
        self.var_lado = tk.StringVar(value=self.r.cfg["navegacion"]["lado_pared"])
        sub = ttk.Frame(f)
        sub.pack(anchor="w")
        ttk.Label(sub, text="   pared:").pack(side="left")
        for txt, val in (("auto", "auto"), ("izq", "izq"), ("der", "der")):
            ttk.Radiobutton(sub, text=txt, value=val, variable=self.var_lado,
                            command=lambda: self._fijar("navegacion", "lado_pared",
                                                        self.var_lado.get())).pack(side="left")
        self._campo(f, "kp", "navegacion", "Kp centrado", 0, 300, 1, 0)
        self._campo(f, "kd", "navegacion", "Kd centrado", 0, 200, 1, 0)
        self._campo(f, "kp_pared", "navegacion", "Kp pared", 0, 400, 1, 0)
        self._campo(f, "kd_pared", "navegacion", "Kd pared", 0, 200, 1, 0)
        self._campo(f, "pared_objetivo", "navegacion", "distancia obj", 0.0, 1.0, 0.01, 2)

        f = ttk.LabelFrame(cont, text="Umbrales de espacio libre", padding=4)
        f.pack(fill="x", pady=2)
        self._campo(f, "frenar_bajo", "navegacion", "frenar bajo", 0.0, 1.0, 0.01, 2)
        self._campo(f, "girar_bajo", "navegacion", "girar bajo", 0.0, 1.0, 0.01, 2)
        self._campo(f, "salir_giro_sobre", "navegacion", "salir giro", 0.0, 1.0, 0.01, 2)
        self._campo(f, "parar_bajo", "navegacion", "parar bajo", 0.0, 1.0, 0.01, 2)
        self._campo(f, "dir_giro", "navegacion", "dir en giro %", 0, 100, 1, 0)
        self._campo(f, "giro_max_ms", "navegacion", "giro max ms", 200, 6000, 50, 0)
        self._campo(f, "min_recto_ms", "navegacion", "min recto ms", 0, 3000, 50, 0)

        f = ttk.LabelFrame(cont, text="Lectura del muro", padding=4)
        f.pack(fill="x", pady=2)
        self._campo(f, "px_min_columna", "navegacion", "px min columna", 1, 60, 1, 0)
        self._campo(f, "suavizado", "navegacion", "suavizado", 1, 61, 2, 0)
        self._campo(f, "ignorar_abajo", "navegacion", "chasis abajo", 0.0, 0.4, 0.01, 2)
        self._campo(f, "banda_lateral", "navegacion", "banda lateral", 0.05, 0.5, 0.01, 2)
        self._campo(f, "ruedas_izq", "navegacion", "rueda izq", 0.0, 0.5, 0.01, 2)
        self._campo(f, "ruedas_der", "navegacion", "rueda der", 0.5, 1.0, 0.01, 2)

        f = ttk.LabelFrame(cont, text="Hueco pasable y esquina interna", padding=4)
        f.pack(fill="x", pady=2)
        self._campo(f, "umbral_hueco", "navegacion", "umbral hueco", 0.0, 1.0, 0.01, 2)
        self._campo(f, "margen_hueco", "navegacion", "margen ruedas", 1.0, 2.5, 0.05, 2)
        self._campo(f, "y_horizonte", "navegacion", "horizonte", 0.0, 0.9, 0.01, 2)
        self._campo(f, "peso_siguiente", "navegacion", "peso sig. obst", 0.0, 3.0, 0.1, 1)
        self._campo(f, "salto_min", "navegacion", "salto minimo", 0.02, 0.6, 0.01, 2)
        self._campo(f, "interno_libre", "navegacion", "interno libre", 0.2, 1.0, 0.01, 2)
        self._campo(f, "retardo_giro_ms", "navegacion", "retardo giro", 0, 1500, 50, 0)
        self._campo(f, "dir_giro_abierto", "navegacion", "giro abierto %", 10, 100, 1, 0)
        self.var_esq = tk.BooleanVar(value=self.r.cfg["navegacion"]["usar_esquina_interna"])
        ttk.Checkbutton(f, text="girar cuando el muro interno desaparece",
                        variable=self.var_esq,
                        command=lambda: self._fijar("navegacion", "usar_esquina_interna",
                                                    self.var_esq.get())).pack(anchor="w")

        f = ttk.LabelFrame(cont, text="Anticipacion y escape", padding=4)
        f.pack(fill="x", pady=2)
        self._campo(f, "ttc_min", "navegacion", "seg. al muro", 0.0, 4.0, 0.1, 1)
        self._campo(f, "vel_escape", "navegacion", "vel escape %", 0, 60, 1, 0)
        self._campo(f, "escape_atras_min_ms", "navegacion", "atras minimo", 200, 3000, 50, 0)
        self._campo(f, "escape_atras_extra_ms", "navegacion", "atras extra", 0, 4000, 100, 0)
        self._campo(f, "escape_atascado_ms", "navegacion", "atascado tras", 300, 4000, 100, 0)
        self._campo(f, "escape_salir_factor", "navegacion", "salir con x", 1.0, 2.0, 0.05, 2)
        self._campo(f, "mejora_min", "navegacion", "mejora minima", 0.0, 0.2, 0.005, 3)
        self.var_auto = tk.BooleanVar(value=self.r.cfg["navegacion"]["autocalibrar_carril"])
        ttk.Checkbutton(f, text="calibrar el ancho del carril solo",
                        variable=self.var_auto,
                        command=lambda: self._fijar("navegacion", "autocalibrar_carril",
                                                    self.var_auto.get())).pack(anchor="w")

        f = ttk.LabelFrame(cont, text="Esquivar pilares de colores", padding=4)
        f.pack(fill="x", pady=2)
        ttk.Label(f, text="rojo -> por su derecha · verde -> por su izquierda",
                  foreground="#555").pack(anchor="w")
        self.var_obs = tk.BooleanVar(value=self.r.cfg["obstaculos"]["activo"])
        ttk.Checkbutton(f, text="ESQUIVAR OBSTACULOS", variable=self.var_obs,
                        command=self._alternar_obstaculos).pack(anchor="w")
        self._campo(f, "margen_lateral", "obstaculos", "separacion", 0.5, 2.5, 0.05, 2)
        self._campo(f, "activar_desde", "obstaculos", "hacer caso", 0.1, 0.9, 0.01, 2)
        self._campo(f, "mandar_desde", "obstaculos", "manda desde", 0.2, 0.99, 0.01, 2)
        self._campo(f, "area_min_pilar", "obstaculos", "area min", 50, 4000, 25, 0)
        self._campo(f, "kp", "obstaculos", "Kp pilar", 0, 300, 1, 0)

        f = ttk.LabelFrame(cont, text="Sentido de la vuelta", padding=4)
        f.pack(fill="x", pady=2)
        ttk.Label(f, text="externa a la izq = antihorario · a la der = horario",
                  foreground="#555").pack(anchor="w")
        self.var_sen = tk.IntVar(value=self.r.navegador.paredes.forzado)
        for txt, val in (("automatico", 0), ("forzar antihorario", -1),
                         ("forzar horario", 1)):
            ttk.Radiobutton(f, text=txt, value=val, variable=self.var_sen,
                            command=lambda: self.r.navegador.paredes.forzar(
                                self.var_sen.get())).pack(anchor="w")

        f = ttk.LabelFrame(cont, text="Vueltas", padding=4)
        f.pack(fill="x", pady=2)
        self._campo(f, "objetivo", "vueltas", "vueltas ida", 1, 10, 1, 0)
        self._campo(f, "esquinas_por_vuelta", "vueltas", "esq/vuelta", 1, 8, 1, 0)
        self.var_mv = tk.BooleanVar(value=self.r.cfg["vueltas"]["hacer_media_vuelta"])
        ttk.Checkbutton(f, text="media vuelta y volver", variable=self.var_mv,
                        command=lambda: self._fijar("vueltas", "hacer_media_vuelta",
                                                    self.var_mv.get())).pack(anchor="w")
        self.var_tmv = tk.StringVar(value=self.r.cfg["vueltas"]["tipo_media_vuelta"])
        for txt, val in (("en recta, 3 tiempos", "recta_3t"), ("en la esquina", "esquina")):
            ttk.Radiobutton(f, text=txt, value=val, variable=self.var_tmv,
                            command=lambda: self._fijar("vueltas", "tipo_media_vuelta",
                                                        self.var_tmv.get())).pack(anchor="w")
        bv = ttk.Frame(f)
        bv.pack(fill="x", pady=2)
        ttk.Button(bv, text="Reiniciar contador",
                   command=self.r.nueva_carrera).pack(side="left")
        ttk.Button(bv, text="Media vuelta ya",
                   command=self.r.navegador.pedir_media_vuelta).pack(side="left", padx=3)

        f = ttk.LabelFrame(cont, text="Giroscopio", padding=4)
        f.pack(fill="x", pady=2)
        self.var_yaw = tk.BooleanVar(value=self.r.cfg["navegacion"]["usar_yaw"])
        ttk.Checkbutton(f, text="usar el yaw para ir recto", variable=self.var_yaw,
                        command=lambda: self._fijar("navegacion", "usar_yaw",
                                                    self.var_yaw.get())).pack(anchor="w")
        self._campo(f, "yaw_kp", "navegacion", "yaw Kp", 0.0, 6.0, 0.1, 1)
        self._campo(f, "yaw_max", "navegacion", "yaw max %", 0, 100, 1, 0)
        self._campo(f, "giro_grados", "navegacion", "giro grados", 0, 180, 5, 0)
        botones = ttk.Frame(f)
        botones.pack(fill="x", pady=2)
        ttk.Button(botones, text="Calibrar", command=self._calibrar_imu).pack(side="left")
        ttk.Button(botones, text="Yaw a cero", command=self._cero_yaw).pack(side="left", padx=3)
        ttk.Button(botones, text="Calibrar color",
                   command=self._calibrar_color).pack(side="left")
        ttk.Button(f, text="REINTENTAR SENSORES I2C",
                   command=self._reintentar_sensores).pack(fill="x", pady=(4, 0))

        f = ttk.LabelFrame(cont, text="Manual", padding=4)
        f.pack(fill="x", pady=2)
        self.c_mvel = Campo(f, "vel %", -60, 60, 0, ancho_etiqueta=8,
                            al_cambiar=lambda v: self._manual())
        self.c_mvel.pack(fill="x")
        self.c_mdir = Campo(f, "dir %", -100, 100, 0, ancho_etiqueta=8,
                            al_cambiar=lambda v: self._manual())
        self.c_mdir.pack(fill="x")
        ttk.Button(f, text="Centrar mandos", command=self._centrar_manual).pack(fill="x")

        f = ttk.Frame(cont)
        f.pack(fill="x", pady=6)
        ttk.Button(f, text="Guardar ajustes", command=self.r.guardar_config).pack(fill="x")
        ttk.Button(f, text="Recargar colores", command=self.r.recargar_colores).pack(fill="x")

    # ------------------------------------------------------------------
    def _peso(self, clave, valor):
        mez = dict(self.r.cfg["navegacion"].get("mezcla") or {})
        mez[clave] = float(valor)
        self.r.cfg["navegacion"]["mezcla"] = mez
        self.r.aplicar_config()

    def _solo(self, clave):
        for k, campo in self.campos_mezcla.items():
            campo.fijar(1.0 if k == clave else 0.0)
        self.r.cfg["navegacion"]["mezcla"] = {
            k: (1.0 if k == clave else 0.0) for k in self.campos_mezcla}
        self.r.cfg["navegacion"]["estrategia"] = clave
        self.r.navegador.reiniciar()
        self.r.aplicar_config()

    def _calibrar_imu(self):
        self.r.sensores.calibrar_rumbo(self.r.enlace)

    def _cero_yaw(self):
        self.r.sensores.poner_cero(self.r.enlace)
        self.r.navegador.rumbo_objetivo = 0.0

    def _alternar_obstaculos(self):
        self.r.cfg["obstaculos"]["activo"] = bool(self.var_obs.get())
        self.r.esquiva.reiniciar()
        self.r.aplicar_config()

    def _reintentar_sensores(self):
        self.r.reintentar_sensores()

    def _calibrar_color(self):
        self.r.sensores.calibrar_color(self.r.enlace)

    def _manual(self):
        self.r.mando_manual(int(self.c_mvel.obtener()), int(self.c_mdir.obtener()))

    def _centrar_manual(self):
        self.c_mvel.fijar(0)
        self.c_mdir.fijar(0)
        self.r.mando_manual(0, 0)

    def alternar_armado(self):
        self.r.armar(not self.r.armado)

    def cerrar(self):
        try:
            self.r.emergencia()
        except Exception:
            pass
        self.root.destroy()

    # ------------------------------------------------------------------
    def _refrescar(self):
        try:
            self._paso()
        except Exception as e:
            self.estado_lbl.configure(text=f"error en el panel: {e}")
        self.root.after(60, self._refrescar)

    def _paso(self):
        anotado, mascara = self.r.instantanea()
        img = mascara if self.var_vista.get() == "mascara" else anotado
        if img is not None and self.lienzo is not None:
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            if img.shape[0] > self.alto_vista:
                esc = self.alto_vista / img.shape[0]
                img = cv2.resize(img, None, fx=esc, fy=esc, interpolation=cv2.INTER_AREA)
            tkimg = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
            self.lienzo.configure(image=tkimg)
            self.lienzo.image = tkimg

        armado = self.r.armado
        self.btn_armar.configure(text="ARMADO (pulsa para soltar)" if armado else "ARMAR",
                                 bg="#1a7f37" if armado else "#37474f")

        e = self.r.enlace
        t = e.telemetria
        d = self.r.decision
        est = (f"{'ARMADO' if armado else 'desarmado'} | {self.r.modo} | "
               f"{self.r.fps:.1f} fps | {d.estado} vel={d.vel:+d}% dir={d.direccion:+d}% | "
               f"ESP32 {'OK ' + e.puerto if e.conectado else 'sin conexion'} "
               f"{e.latencia_ms:.0f}ms pwm={t.pwm} ang={t.angulo}"
               f"{' FAILSAFE' if t.failsafe else ''}")
        c = self.r.contador
        est += (f" | vuelta {c.vueltas}/{c.objetivo}"
                f" esq {c.esquinas % c.esquinas_por_vuelta}/{c.esquinas_por_vuelta}")
        if self.r.sensores.hay_rumbo:
            est += f" | yaw {self.r.sensores.yaw:+.1f} ({self.r.sensores.origen_rumbo})"
        s_ = self.r.navegador.paredes.estado()
        est += f" | ext {s_['externa']} ({s_['nombre']})"
        o = self.r.esquiva.estado()
        if o.get("siguiendo"):
            est += f" | {o['motivo']}"
        if self.r.navegador.carril.listo:
            est += f" | carril {self.r.navegador.carril.ancho:.2f}"
        self.estado_lbl.configure(text=est)

        reg = self.r.registro[-8:]
        self.txt.delete("1.0", "end")
        self.txt.insert("1.0", "\n".join(reg))

    def ejecutar(self):
        self.root.mainloop()


# ---------------------------------------------------------------------------
def main():
    import argparse
    from src import color_config as cc, robot_config, robot as robot_mod

    ap = argparse.ArgumentParser()
    ap.add_argument("--simulado", action="store_true")
    ap.add_argument("--imagen", default=None)
    ap.add_argument("--perfil", default=None)
    args = ap.parse_args()

    cfg = robot_config.cargar()
    perfil = cc.obtener(cc.cargar(), args.perfil)
    r = robot_mod.Robot(cfg, perfil, simulado=args.simulado, fuente_imagen=args.imagen)
    r.iniciar()
    try:
        Panel(r).ejecutar()
    finally:
        r.cerrar()


if __name__ == "__main__":
    main()
