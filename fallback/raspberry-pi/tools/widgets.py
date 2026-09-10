"""
widgets.py — Trocitos de interfaz compartidos por el calibrador y el panel.

MarcoScroll: contenedor con barra de desplazamiento (la rueda del raton se
enlaza distinto en Windows que en X11, aqui van los dos casos).
Campo: slider + casilla de texto sobre el mismo valor, que es lo que pediste:
arrastrar cuando exploras y escribir el numero exacto cuando ya lo sabes.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk


class MarcoScroll(ttk.Frame):
    """Contenedor con barra de desplazamiento vertical (rueda del raton
    incluida, con el binding distinto de Windows y de X11)."""

    def __init__(self, master, **kw):
        super().__init__(master, **kw)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0, width=330)
        self.scroll = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.interior = ttk.Frame(self.canvas)
        self._win = self.canvas.create_window((0, 0), window=self.interior, anchor="nw")

        self.canvas.configure(yscrollcommand=self.scroll.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scroll.pack(side="right", fill="y")

        self.interior.bind("<Configure>", self._al_configurar)
        self.canvas.bind("<Configure>", self._ajustar_ancho)
        for w in (self.canvas, self.interior):
            w.bind("<Enter>", lambda e: self._rueda(True))
            w.bind("<Leave>", lambda e: self._rueda(False))

    def _al_configurar(self, _e=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _ajustar_ancho(self, e):
        self.canvas.itemconfigure(self._win, width=e.width)

    def _rueda(self, activar: bool):
        if activar:
            self.canvas.bind_all("<MouseWheel>", self._win_wheel)      # Windows/macOS
            self.canvas.bind_all("<Button-4>", self._x11_wheel)        # Linux arriba
            self.canvas.bind_all("<Button-5>", self._x11_wheel)        # Linux abajo
        else:
            self.canvas.unbind_all("<MouseWheel>")
            self.canvas.unbind_all("<Button-4>")
            self.canvas.unbind_all("<Button-5>")

    def _win_wheel(self, e):
        self.canvas.yview_scroll(int(-e.delta / 120), "units")

    def _x11_wheel(self, e):
        self.canvas.yview_scroll(-1 if e.num == 4 else 1, "units")


class Campo(ttk.Frame):
    """Slider + casilla de texto sincronizados sobre el mismo valor."""

    def __init__(self, master, etiqueta: str, minimo, maximo, valor,
                 resolucion=1, decimales=0, al_cambiar=None, ancho_etiqueta=11):
        super().__init__(master)
        self.decimales = decimales
        self.al_cambiar = al_cambiar
        self._bloqueo = False

        ttk.Label(self, text=etiqueta, width=ancho_etiqueta, anchor="w").grid(
            row=0, column=0, sticky="w")

        self.var = tk.DoubleVar(value=float(valor))
        self.escala = tk.Scale(self, from_=minimo, to=maximo, resolution=resolucion,
                               orient="horizontal", variable=self.var,
                               showvalue=False, length=180, sliderlength=14,
                               command=self._desde_escala)
        self.escala.grid(row=0, column=1, sticky="ew", padx=(2, 4))

        self.txt = tk.StringVar(value=self._fmt(valor))
        self.entrada = ttk.Entry(self, textvariable=self.txt, width=7, justify="right")
        self.entrada.grid(row=0, column=2, sticky="e")
        self.entrada.bind("<Return>", self._desde_texto)
        self.entrada.bind("<KP_Enter>", self._desde_texto)
        self.entrada.bind("<FocusOut>", self._desde_texto)

        self.columnconfigure(1, weight=1)

    def _fmt(self, v) -> str:
        return f"{float(v):.{self.decimales}f}" if self.decimales else str(int(round(float(v))))

    def _desde_escala(self, _valor=None):
        if self._bloqueo:
            return
        self._bloqueo = True
        self.txt.set(self._fmt(self.var.get()))
        self._bloqueo = False
        if self.al_cambiar:
            self.al_cambiar(self.obtener())

    def _desde_texto(self, _e=None):
        try:
            v = float(self.txt.get().replace(",", "."))
        except ValueError:
            self.txt.set(self._fmt(self.var.get()))
            return
        v = max(float(self.escala.cget("from")), min(float(self.escala.cget("to")), v))
        self._bloqueo = True
        self.var.set(v)
        self.txt.set(self._fmt(v))
        self._bloqueo = False
        if self.al_cambiar:
            self.al_cambiar(self.obtener())

    def obtener(self):
        v = self.var.get()
        return round(v, self.decimales) if self.decimales else int(round(v))

    def fijar(self, valor, disparar=False):
        self._bloqueo = True
        self.var.set(float(valor))
        self.txt.set(self._fmt(valor))
        self._bloqueo = False
        if disparar and self.al_cambiar:
            self.al_cambiar(self.obtener())
