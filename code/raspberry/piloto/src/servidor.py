"""
servidor.py — La web de depuracion en http://carrito.local:8080/

Solo biblioteca estandar (http.server) + OpenCV para el JPEG. La pagina vive
en src/web/index.html y se sirve desde disco (editar y F5, sin reiniciar).

Rutas:
    /                    la pagina
    /stream.mjpg         video: ?vista=normal|cruda|piso|mascara&color=rojo
    /api/estado          estado completo (JSON)
    /api/esquema         esquema de parametros (la web arma los sliders sola)
    /api/valores         valores actuales de todos los parametros
    /api/color?color=x   parametros del color x
    /api/registro        ultimas lineas del log
    /api/cmd?...         TODAS las ordenes (ver _cmd)
"""

from __future__ import annotations

import json
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional

import cv2

from . import color_config as cc
from . import params as params_mod

RUTA_WEB = Path(__file__).resolve().parent / "web"


class _Handler(BaseHTTPRequestHandler):
    servidor_ref: "Servidor" = None
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    # -- utilidades -------------------------------------------------------
    def _enviar(self, cuerpo: bytes, tipo: str = "text/html; charset=utf-8",
                codigo: int = 200):
        self.send_response(codigo)
        self.send_header("Content-Type", tipo)
        self.send_header("Content-Length", str(len(cuerpo)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(cuerpo)

    def _json(self, obj: Any, codigo: int = 200):
        self._enviar(json.dumps(obj).encode("utf-8"),
                     "application/json", codigo)

    def _stream(self, args: Dict[str, str]):
        srv = self.servidor_ref
        vista = args.get("vista", "normal")
        color = args.get("color", "negro")
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        red = srv.robot.p["red"]
        try:
            while not srv.parado.is_set():
                calidad = int(red.get("calidad_jpeg", 70))
                ancho = int(red.get("ancho_stream", 640))
                periodo = 1.0 / max(2, int(red.get("fps_stream", 15)))
                img = srv.robot.instantanea(vista, color)
                if img is None:
                    time.sleep(0.1)
                    continue
                if ancho and img.shape[1] != ancho:
                    esc = ancho / img.shape[1]
                    img = cv2.resize(img, None, fx=esc, fy=esc,
                                     interpolation=cv2.INTER_AREA)
                ok, jpg = cv2.imencode(".jpg", img,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), calidad])
                if not ok:
                    time.sleep(0.05)
                    continue
                datos = jpg.tobytes()
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(datos)}\r\n\r\n".encode())
                self.wfile.write(datos)
                self.wfile.write(b"\r\n")
                time.sleep(periodo)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    # -- rutas ------------------------------------------------------------
    def do_GET(self):
        partes = urllib.parse.urlparse(self.path)
        ruta = partes.path
        args = {k: v[0] for k, v in urllib.parse.parse_qs(partes.query).items()}
        srv = self.servidor_ref
        r = srv.robot
        try:
            if ruta == "/":
                pagina = (RUTA_WEB / "index.html").read_bytes()
                self._enviar(pagina)
            elif ruta == "/stream.mjpg":
                self._stream(args)
            elif ruta == "/api/estado":
                est = r.estado()
                est["perfiles_color"] = {
                    "activo": r.datos_colores.get("activo"),
                    "lista": cc.listar(r.datos_colores),
                }
                est["perfiles_params"] = {
                    "activo": r.datos_params.get("activo"),
                    "lista": [p["nombre"] for p in r.datos_params["perfiles"]],
                }
                self._json(est)
            elif ruta == "/api/esquema":
                self._json(params_mod.esquema_para_web())
            elif ruta == "/api/valores":
                self._json(r.p)
            elif ruta == "/api/color":
                nombre = args.get("color", "negro")
                c = r.perfil_color["colores"].get(nombre)
                if c is None:
                    self._json({"error": f"no existe el color {nombre}"}, 404)
                else:
                    self._json({"color": nombre, "params": c})
            elif ruta == "/api/registro":
                self._json(r.registro[-80:])
            elif ruta == "/api/cmd":
                self._json(srv.ejecutar(args))
            else:
                self._enviar(b"no existe", "text/plain", 404)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        except Exception as e:
            try:
                self._json({"ok": False, "error": f"{type(e).__name__}: {e}"}, 500)
            except Exception:
                pass


class Servidor:
    def __init__(self, robot):
        self.robot = robot
        self.parado = threading.Event()
        self._srv: Optional[ThreadingHTTPServer] = None
        self._hilo: Optional[threading.Thread] = None

    # -- ordenes ----------------------------------------------------------
    def ejecutar(self, args: Dict[str, str]) -> Dict[str, Any]:
        r = self.robot
        respuesta: Dict[str, Any] = {"ok": True}
        try:
            for k, v in args.items():
                if k == "armar":
                    r.armar(v not in ("0", "false", ""))
                elif k == "emergencia":
                    r.emergencia()
                elif k == "modo":
                    r.fijar_modo(v)
                elif k == "manual":
                    a, b = v.split(",")
                    r.mando_manual(int(float(a)), int(float(b)))
                elif k == "set":
                    grupo, clave = v.split(".", 1)
                    respuesta["valor"] = r.fijar_param(grupo, clave,
                                                       args.get("val", ""))
                elif k == "perfil_params_guardar":
                    r.guardar_perfil_params(v)
                elif k == "perfil_params_cargar":
                    r.cargar_perfil_params(v)
                elif k == "perfil_color_guardar":
                    r.guardar_perfil_colores(v)
                elif k == "perfil_color_cargar":
                    r.cargar_perfil_colores(v)
                elif k == "color_set":
                    r.fijar_color(args.get("color", ""), v,
                                  json.loads(args.get("val", "null")))
                elif k == "color_clic":
                    rangos = r.clic_color(args.get("color", "negro"),
                                          float(args.get("x", 0)),
                                          float(args.get("y", 0)),
                                          args.get("acumular", "0") == "1")
                    respuesta["rangos"] = rangos
                elif k == "calibrar_giro":
                    r.calibrar_giro()
                elif k == "cero_yaw":
                    r.cero_yaw()
                elif k == "redetectar":
                    r.redetectar_i2c()
                elif k == "tcs_muestrear":
                    respuesta["tcs"] = r.muestrear_tcs(v)
                elif k == "cal_fy":
                    respuesta["fy"] = r.calibrar_fy(
                        float(args.get("y", 0)), float(args.get("dist", 0)))
                elif k == "cal_fx":
                    respuesta["fx"] = r.calibrar_fx(
                        float(args.get("x", 0)), float(args.get("y", 0)),
                        float(args.get("lat", 0)))
                elif k == "reiniciar_carrera":
                    r.carrera.reiniciar()
                    if r.armado and r.modo == "auto":
                        r.carrera.arrancar()
                elif k in ("val", "color", "x", "y", "dist", "lat", "acumular"):
                    pass          # argumentos de otras ordenes
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}
        return respuesta

    # -- ciclo de vida ----------------------------------------------------
    def iniciar(self) -> str:
        _Handler.servidor_ref = self
        puerto = int(self.robot.p["red"].get("puerto_http", 8080))
        self._srv = ThreadingHTTPServer(("0.0.0.0", puerto), _Handler)
        self._srv.daemon_threads = True
        self._hilo = threading.Thread(target=self._srv.serve_forever,
                                      daemon=True, name="http")
        self._hilo.start()
        return f"http://carrito.local:{puerto}/"

    def cerrar(self):
        self.parado.set()
        if self._srv:
            self._srv.shutdown()
            self._srv.server_close()
