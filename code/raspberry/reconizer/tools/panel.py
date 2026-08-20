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
                                   bg="#1a7f37", fg="white", width=16, height=2,
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

        f = ttk.LabelFrame(cont, text="Estrategia", padding=4)
        f.pack(fill="x", pady=2)
        self.var_est = tk.StringVar(value=self.r.cfg["navegacion"]["estrategia"])
        for txt, val in (("Centrado por espacio libre", "centrado"),
                         ("Seguir una pared", "pared")):
            ttk.Radiobutton(f, text=txt, value=val, variable=self.var_est,
                            command=self._cambiar_estrategia).pack(anchor="w")
        self.var_lado = tk.StringVar(value=self.r.cfg["navegacion"]["lado_pared"])
        sub = ttk.Frame(f)
        sub.pack(anchor="w")
        ttk.Label(sub, text="   pared:").pack(side="left")
        for txt, val in (("izq", "izq"), ("der", "der")):
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
    def _cambiar_estrategia(self):
        self.r.cfg["navegacion"]["estrategia"] = self.var_est.get()
        self.r.navegador.reiniciar()
        self.r.aplicar_config()

    def _calibrar_imu(self):
        import threading
        threading.Thread(target=self.r.imu.calibrar, daemon=True).start()

    def _cero_yaw(self):
        self.r.imu.poner_cero()
        self.r.navegador.rumbo_objetivo = 0.0

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
        self.btn_armar.configure(text="ARMADO — pulsa para soltar" if armado else "ARMAR",
                                 bg="#1a7f37" if armado else "#37474f")

        e = self.r.enlace
        t = e.telemetria
        d = self.r.decision
        est = (f"{'ARMADO' if armado else 'desarmado'} | {self.r.modo} | "
               f"{self.r.fps:.1f} fps | {d.estado} vel={d.vel:+d}% dir={d.direccion:+d}% | "
               f"ESP32 {'OK ' + e.puerto if e.conectado else 'sin conexion'} "
               f"{e.latencia_ms:.0f}ms pwm={t.pwm} ang={t.angulo}"
               f"{' FAILSAFE' if t.failsafe else ''}")
        if self.r.imu.disponible:
            est += f" | yaw {self.r.imu.yaw:+.1f}"
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
