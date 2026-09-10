"""
servidor.py — Lo que ve el carro, en el movil, en carrito.local:8080

Solo biblioteca estandar (http.server) + OpenCV para el JPEG: nada de Flask.
Menos cosas que instalar en la Pi y menos que se rompa el dia de la competencia.

Rutas
    /                pagina de control (movil primero)
    /stream.mjpg     video anotado: detecciones, perfil del muro, ruedas, decision
    /mascara.mjpg    la mascara binaria del muro, para depurar la calibracion
    /api/estado      JSON con todo el estado
    /api/cmd?...     ordenes (armar, modo, vmax, ganancias, manual...)

El stream es MJPEG multipart: funciona en cualquier navegador con un <img>,
sin JavaScript ni WebRTC, y si la red se corta se reengancha solo.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

import cv2
import numpy as np

PAGINA = """<!DOCTYPE html>
<html lang="es"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Carrito WRO</title>
<style>
 *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
 body{margin:0;background:#0f1115;color:#e8eaed;font:14px system-ui,Arial;padding:10px}
 h1{font-size:1em;margin:0 0 8px;color:#9aa0a6;font-weight:600}
 img{width:100%;border-radius:10px;background:#000;display:block}
 .fila{display:flex;gap:8px;margin:10px 0}
 button{flex:1;padding:16px 8px;font-size:1em;font-weight:700;border:0;border-radius:10px;
        background:#2b5cd9;color:#fff;touch-action:manipulation}
 button.off{background:#2a2f3a;color:#aab}
 #btnArmar.on{background:#1a7f37}
 #btnStop{background:#c62828}
 .caja{background:#171a21;border-radius:10px;padding:10px;margin:10px 0}
 .et{display:flex;justify-content:space-between;color:#9aa0a6;font-size:.85em;margin-bottom:2px}
 input[type=range]{width:100%;height:34px}
 table{width:100%;font-size:.85em;border-collapse:collapse}
 td{padding:2px 0;color:#c8ccd2} td:first-child{color:#8b9199;width:45%}
 .ok{color:#4ade80} .mal{color:#f87171} .avi{color:#fbbf24}
 .tabs{display:flex;gap:6px;margin-bottom:8px}
 .tabs button{padding:8px;font-size:.85em}
</style></head><body>
<h1>Carrito WRO &mdash; Futuros Ingenieros</h1>
<img id="cam" src="/stream.mjpg" alt="camara">

<div class="fila">
  <button id="btnArmar" onclick="cmd('armar','1')">ARMAR</button>
  <button id="btnStop" onclick="cmd('emergencia','1')">PARAR</button>
</div>

<div class="tabs">
  <button onclick="cmd('modo','auto')" id="mAuto">AUTO</button>
  <button onclick="cmd('modo','manual')" id="mManual" class="off">MANUAL</button>
  <button onclick="verMascara()" id="mMask" class="off">MASCARA</button>
</div>

<div class="caja">
  <div class="et"><span>Velocidad maxima (PWM tope del ESP32)</span><span id="vVmax">-</span></div>
  <input type="range" id="vmax" min="0" max="255" oninput="lz('vmax',this.value)">
  <div class="et"><span>Velocidad de crucero</span><span id="vCru">-</span>
  </div><input type="range" id="crucero" min="0" max="100" oninput="lz('vel_crucero',this.value)">
  <div class="et"><span>Velocidad en giro</span><span id="vGir">-</span></div>
  <input type="range" id="giro" min="0" max="100" oninput="lz('vel_giro',this.value)">
</div>

<div class="caja">
  <div class="et"><span>Estrategia</span><span id="vEst">-</span></div>
  <div class="fila">
    <button onclick="cmd('estrategia','centrado')">Centrado</button>
    <button onclick="cmd('estrategia','pared')">Seguir pared</button>
  </div>
  <div class="et"><span>Kp</span><span id="vKp">-</span></div>
  <input type="range" id="kp" min="0" max="300" oninput="lz('kp',this.value)">
  <div class="et"><span>Kd</span><span id="vKd">-</span></div>
  <input type="range" id="kd" min="0" max="200" oninput="lz('kd',this.value)">
  <div class="et"><span>Umbral de giro</span><span id="vGb">-</span></div>
  <input type="range" id="girar" min="0" max="100" oninput="lz('girar_bajo',this.value/100)">
</div>

<div class="caja" id="cajaManual" style="display:none">
  <div class="et"><span>Manual: velocidad</span><span id="vMv">0</span></div>
  <input type="range" id="mv" min="-60" max="60" value="0" oninput="man()"
         onchange="this.value=0;man()">
  <div class="et"><span>Manual: direccion</span><span id="vMd">0</span></div>
  <input type="range" id="md" min="-100" max="100" value="0" oninput="man()"
         onchange="this.value=0;man()">
</div>

<div class="caja"><table id="tel"></table></div>
<div class="fila">
  <button onclick="cmd('calibrar_imu','1')" class="off">Calibrar giroscopio</button>
  <button onclick="cmd('guardar','1')" class="off">Guardar ajustes</button>
</div>

<script>
let ultimo = 0, tocando = 0;
function cmd(k,v){ fetch('/api/cmd?'+k+'='+encodeURIComponent(v)).then(estado); }
function lz(k,v){ tocando = Date.now(); cmd(k,v); }
function man(){
  const v = +document.getElementById('mv').value, d = +document.getElementById('md').value;
  document.getElementById('vMv').textContent = v;
  document.getElementById('vMd').textContent = d;
  tocando = Date.now();
  fetch('/api/cmd?manual='+v+','+d);
}
function verMascara(){
  const img = document.getElementById('cam');
  img.src = img.src.indexOf('mascara')>0 ? '/stream.mjpg' : '/mascara.mjpg';
}
function fila(t,k,v,c){ t.innerHTML += '<tr><td>'+k+'</td><td class="'+(c||'')+'">'+v+'</td></tr>'; }
function estado(){
  fetch('/api/estado').then(r=>r.json()).then(s=>{
    document.getElementById('btnArmar').className = s.armado ? 'on' : '';
    document.getElementById('btnArmar').textContent = s.armado ? 'ARMADO (pulsa para soltar)' : 'ARMAR';
    document.getElementById('btnArmar').setAttribute('onclick',
       "cmd('armar','"+(s.armado?'0':'1')+"')");
    document.getElementById('mAuto').className   = s.modo=='auto'   ? '' : 'off';
    document.getElementById('mManual').className = s.modo=='manual' ? '' : 'off';
    document.getElementById('cajaManual').style.display = s.modo=='manual' ? '' : 'none';

    if (Date.now() - tocando > 1500) {
      document.getElementById('vmax').value    = s.limites.vmax;
      document.getElementById('crucero').value = s.limites.vel_crucero;
      document.getElementById('giro').value    = s.limites.vel_giro;
      document.getElementById('kp').value      = s.navegacion.kp;
      document.getElementById('kd').value      = s.navegacion.kd;
      document.getElementById('girar').value   = Math.round(s.navegacion.girar_bajo*100);
    }
    document.getElementById('vVmax').textContent = s.limites.vmax;
    document.getElementById('vCru').textContent  = s.limites.vel_crucero+'%';
    document.getElementById('vGir').textContent  = s.limites.vel_giro+'%';
    document.getElementById('vKp').textContent   = s.navegacion.kp;
    document.getElementById('vKd').textContent   = s.navegacion.kd;
    document.getElementById('vGb').textContent   = s.navegacion.girar_bajo.toFixed(2);
    document.getElementById('vEst').textContent  = s.navegacion.estrategia;

    const t = document.getElementById('tel'); t.innerHTML='';
    const e = s.enlace, m = s.decision.metricas;
    fila(t,'ESP32', e.conectado ? (e.puerto+'  '+e.latencia_ms+' ms') : e.motivo,
         e.conectado?'ok':'mal');
    if (e.conectado) {
      fila(t,'PWM / angulo', e.tele.pwm+'  /  '+e.tele.angulo+'&deg;');
      fila(t,'Failsafe', e.tele.failsafe?'SI':'no', e.tele.failsafe?'avi':'ok');
      fila(t,'Tramas malas', e.tele.tramas_malas, e.tele.tramas_malas>0?'avi':'');
    }
    fila(t,'Estado', s.decision.estado+' &mdash; '+s.decision.motivo);
    fila(t,'vel / dir', s.decision.vel+'%  /  '+s.decision.dir+'%');
    fila(t,'Libre izq/pas/der',
         (m.izq!==undefined?m.izq:'-')+'  '+(m.pasillo!==undefined?m.pasillo:'-')+
         '  '+(m.der!==undefined?m.der:'-'));
    fila(t,'Giroscopio', s.imu.disponible ? ('yaw '+s.imu.yaw+'&deg;  '+s.imu.hz+' Hz')
                                          : s.imu.motivo, s.imu.disponible?'ok':'avi');
    fila(t,'FPS vision', s.fps);
    fila(t,'Perfil de color', s.perfil_color);
  }).catch(()=>{});
}
setInterval(estado, 400); estado();
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    servidor_ref = None          # se rellena al construir Servidor
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):   # silencio: el registro va al log del robot
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

    def _stream(self, mascara: bool):
        srv = self.servidor_ref
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        calidad = int(srv.cfg.get("calidad_jpeg", 70))
        ancho = int(srv.cfg.get("ancho_stream", 640))
        periodo = 1.0 / max(1, int(srv.cfg.get("fps_stream", 15)))
        try:
            while not srv.parado.is_set():
                anotado, masc = srv.robot.instantanea()
                img = masc if mascara else anotado
                if img is None:
                    time.sleep(0.1)
                    continue
                if mascara:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
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
        except (BrokenPipeError, ConnectionResetError):
            pass          # el movil cerro la pestaña; normal

    # -- rutas ------------------------------------------------------------
    def do_GET(self):
        partes = urllib.parse.urlparse(self.path)
        ruta = partes.path
        srv = self.servidor_ref
        if ruta == "/":
            self._enviar(PAGINA.encode("utf-8"))
        elif ruta == "/stream.mjpg":
            self._stream(False)
        elif ruta == "/mascara.mjpg":
            self._stream(True)
        elif ruta == "/api/estado":
            self._enviar(json.dumps(srv.robot.estado()).encode(), "application/json")
        elif ruta == "/api/registro":
            self._enviar(json.dumps(srv.robot.registro[-60:]).encode(), "application/json")
        elif ruta == "/api/cmd":
            args = urllib.parse.parse_qs(partes.query)
            res = srv.ejecutar({k: v[0] for k, v in args.items()})
            self._enviar(json.dumps(res).encode(), "application/json")
        else:
            self._enviar(b"no existe", "text/plain", 404)


class Servidor:
    def __init__(self, robot, cfg: Dict[str, Any]):
        self.robot = robot
        self.cfg = cfg
        self.parado = threading.Event()
        self._srv: Optional[ThreadingHTTPServer] = None
        self._hilo: Optional[threading.Thread] = None

    # -- ordenes ----------------------------------------------------------
    def ejecutar(self, args: Dict[str, str]) -> Dict[str, Any]:
        r = self.robot
        nav_cfg = r.cfg["navegacion"]
        lim = r.cfg["limites"]
        for k, v in args.items():
            try:
                if k == "armar":
                    r.armar(v not in ("0", "false", ""))
                elif k == "emergencia":
                    r.emergencia()
                elif k == "modo":
                    r.fijar_modo(v)
                elif k == "manual":
                    a, b = v.split(",")
                    r.mando_manual(int(float(a)), int(float(b)))
                elif k in ("vmax", "vel_crucero", "vel_giro", "dir_max"):
                    lim[k] = int(float(v))
                    r.aplicar_config()
                elif k == "estrategia":
                    nav_cfg["estrategia"] = "pared" if v.startswith("par") else "centrado"
                    r.navegador.reiniciar()
                elif k == "lado_pared":
                    nav_cfg["lado_pared"] = "izq" if v.startswith("i") else "der"
                elif k in ("kp", "kd", "kp_pared", "kd_pared", "pared_objetivo",
                           "girar_bajo", "frenar_bajo", "parar_bajo",
                           "salir_giro_sobre", "dir_giro", "yaw_kp",
                           "ruedas_izq", "ruedas_der", "banda_lateral",
                           "ignorar_abajo"):
                    nav_cfg[k] = float(v)
                elif k in ("px_min_columna", "suavizado", "giro_max_ms", "min_recto_ms"):
                    nav_cfg[k] = int(float(v))
                elif k == "usar_yaw":
                    nav_cfg["usar_yaw"] = v not in ("0", "false")
                elif k == "calibrar_imu":
                    threading.Thread(target=r.imu.calibrar, daemon=True).start()
                elif k == "cero_yaw":
                    r.imu.poner_cero()
                    r.navegador.rumbo_objetivo = 0.0
                elif k == "guardar":
                    r.guardar_config()
                elif k == "perfil_color":
                    r.recargar_colores(v)
            except Exception as e:
                return {"ok": False, "error": f"{k}: {e}"}
        return {"ok": True}

    # -- ciclo de vida ----------------------------------------------------
    def iniciar(self) -> str:
        _Handler.servidor_ref = self
        puerto = int(self.cfg.get("puerto_http", 8080))
        self._srv = ThreadingHTTPServer(("0.0.0.0", puerto), _Handler)
        self._srv.daemon_threads = True
        self._hilo = threading.Thread(target=self._srv.serve_forever,
                                      daemon=True, name="http")
        self._hilo.start()
        host = self.cfg.get("hostname", "carrito")
        return f"http://{host}.local:{puerto}/"

    def cerrar(self):
        self.parado.set()
        if self._srv:
            self._srv.shutdown()
            self._srv.server_close()
