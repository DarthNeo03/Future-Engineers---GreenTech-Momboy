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
 /* --- joystick --- */
 #pad{position:relative;width:100%;max-width:300px;aspect-ratio:1/1;margin:6px auto 10px;
      background:#0f1115;border:2px solid #2a2f3a;border-radius:16px;
      touch-action:none;overflow:hidden;user-select:none}
 #pad.act{border-color:#2b5cd9}
 #pad .ejeh,#pad .ejev{position:absolute;background:#242935}
 #pad .ejeh{left:6%;right:6%;top:50%;height:1px}
 #pad .ejev{top:6%;bottom:6%;left:50%;width:1px}
 #pad .zm{position:absolute;left:50%;top:50%;width:34%;height:34%;margin:-17% 0 0 -17%;
          border:1px dashed #242935;border-radius:50%}
 #nub{position:absolute;left:50%;top:50%;width:74px;height:74px;margin:-37px 0 0 -37px;
      border-radius:50%;background:#2b5cd9;box-shadow:0 3px 14px #000a;
      transition:background .12s}
 #pad.act #nub{background:#1a7f37}
 #pad .pista{position:absolute;left:0;right:0;bottom:4px;text-align:center;
             color:#5b6270;font-size:.75em;pointer-events:none}
</style></head><body>
<h1>Carrito WRO &mdash; Futuros Ingenieros</h1>
<img id="cam" src="/stream.mjpg" alt="camara">

<div class="fila">
  <button id="btnArmar" onclick="cmd('armar','1')">ARMAR</button>
  <button id="btnStop" onclick="cmd('emergencia','1')">PARAR</button>
</div>

<div class="tabs">
  <button onclick="cmd('reto','abierto')" id="rAbierto">OPEN CHALLENGE</button>
  <button onclick="cmd('reto','obstaculos')" id="rObs" class="off">OBSTACULOS</button>
</div>

<div class="tabs">
  <button onclick="cmd('modo','auto')" id="mAuto">AUTO</button>
  <button onclick="cmd('modo','manual')" id="mManual" class="off">MANUAL</button>
  <button onclick="verMascara()" id="mMask" class="off">MASCARA</button>
</div>

<div class="caja">
  <div class="et"><span>Carrera</span><span id="vCarrera">-</span></div>
  <div class="fila">
    <button onclick="cmd('reiniciar_carrera','1')" class="off">Reiniciar vueltas</button>
    <button onclick="cmd('lado_interno','izq')" class="off">Int. izq</button>
    <button onclick="cmd('lado_interno','der')" class="off">Int. der</button>
  </div>
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
  <div class="et"><span>Distancia al muro INTERNO (mm)</span><span id="vPar">-</span></div>
  <input type="range" id="pared" min="120" max="450" oninput="lz('pared_objetivo_mm',this.value)">
  <div class="et"><span>Girar cuando la esquina interna este a (mm)</span><span id="vGz">-</span></div>
  <input type="range" id="girz" min="150" max="700" oninput="lz('giro_z_mm',this.value)">
  <div class="et"><span>Minimo permitido al muro EXTERNO (mm)</span><span id="vExt">-</span></div>
  <input type="range" id="ext" min="80" max="400" oninput="lz('min_externo_mm',this.value)">
</div>

<div class="caja">
  <div class="et"><span>Angulo de aproximacion max (grados)</span><span id="vKl">-</span></div>
  <input type="range" id="kpl" min="5" max="45" oninput="lz('aprox_max_grados',this.value)">
  <div class="et"><span>Grados de aproximacion por mm de error</span><span id="vKdl">-</span></div>
  <input type="range" id="kdl" min="1" max="20" oninput="lz('aprox_grados_por_mm',this.value/100)">
  <div class="et"><span>Kp de rumbo (%/grado)</span><span id="vKr">-</span></div>
  <input type="range" id="kpr" min="0" max="60" oninput="lz('kp_rumbo',this.value/10)">
  <div class="et"><span>Kp del giro (%/grado)</span><span id="vKg">-</span></div>
  <input type="range" id="kpg" min="0" max="80" oninput="lz('giro_kp',this.value/10)">
</div>

<div class="caja" id="cajaObs" style="display:none">
  <div class="et"><span>Margen de esquive de la señal (mm)</span><span id="vMar">-</span></div>
  <input type="range" id="mar" min="100" max="350" value="190"
         oninput="lz('senal_margen_mm',this.value);document.getElementById('vMar').textContent=this.value">
  <div class="et"><span>ROJO: paso por su derecha &middot; VERDE: por su izquierda</span><span></span></div>
</div>

<div class="caja" id="cajaManual" style="display:none">
  <div class="et"><span>Joystick</span><span id="vMan">0% / 0%</span></div>
  <div id="pad">
    <div class="ejeh"></div><div class="ejev"></div><div class="zm"></div>
    <div id="nub"></div>
    <div class="pista">arriba = avanzar &middot; suelta = para</div>
  </div>
  <div class="et"><span>Tope del joystick (% de vmax)</span><span id="vTope">60</span></div>
  <input type="range" id="tope" min="10" max="100" value="60"
         oninput="document.getElementById('vTope').textContent=this.value">
  <div class="et"><span>Manual: velocidad</span><span id="vMv">0</span></div>
  <input type="range" id="mv" min="-100" max="100" value="0" oninput="man()"
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
  document.getElementById('vMan').textContent = v+'% / '+d+'%';
  tocando = Date.now();
  fetch('/api/cmd?manual='+v+','+d).catch(()=>{});
}

/* ---------------------------------------------------------------------
   Joystick. Escribe en los mismos sliders mv/md y llama a man(), asi que
   hay un solo camino hacia el robot y los sliders sirven de lectura.

   Tres cosas que lo hacen seguro y no son adorno:
     - pointer capture: si el dedo sale del recuadro el evento sigue
       llegando, no se queda el stick "pegado" a fondo;
     - al soltar (o si el navegador cancela el toque) se centra y se manda
       0,0 SIN throttle, para que la orden de parar no se pierda;
     - mientras esta pulsado se reenvia cada 150 ms aunque no te muevas,
       porque el robot caduca el mando manual a los 400 ms y si no
       llegasen refrescos se pararia solo mientras lo sujetas.
   --------------------------------------------------------------------- */
const pad = document.getElementById('pad'), nub = document.getElementById('nub');
let jAct = false, jUlt = 0;
const J_ZONA_MUERTA = 0.12;     // fraccion del radio que se ignora

function jTope(){ return +document.getElementById('tope').value; }

function jAplicar(x, y, forzar){
  x = Math.max(-1, Math.min(1, x));
  y = Math.max(-1, Math.min(1, y));
  const r = Math.hypot(x, y);
  let ex = 0, ey = 0;
  if (r >= J_ZONA_MUERTA) {
    // Reescala fuera de la zona muerta para que no haya salto al entrar, y
    // acota el radio a 1: en las diagonales hypot vale 1.41 y sin esto el
    // tope se pasaba (un tope de 60 daba 62 en las esquinas del pad).
    const k = Math.min(1, (r - J_ZONA_MUERTA) / (1 - J_ZONA_MUERTA)) / r;
    ex = x * k; ey = y * k;
  }
  nub.style.left = (50 + x * 34) + '%';
  nub.style.top  = (50 - y * 34) + '%';
  const tope = jTope();
  document.getElementById('md').value = Math.max(-100, Math.min(100, Math.round(ex * 100)));
  document.getElementById('mv').value = Math.max(-tope, Math.min(tope, Math.round(ey * tope)));
  const ahora = Date.now();
  if (forzar || ahora - jUlt >= 60) { jUlt = ahora; man(); }
}

function jDesde(ev){
  const c = pad.getBoundingClientRect();
  jAplicar(((ev.clientX - c.left) / c.width) * 2 - 1,
           -((((ev.clientY - c.top) / c.height) * 2) - 1), false);
}

function jSoltar(){
  if (!jAct) return;
  jAct = false;
  pad.classList.remove('act');
  jAplicar(0, 0, true);          // parar es prioritario: sin throttle
}

pad.addEventListener('pointerdown', ev => {
  ev.preventDefault();
  jAct = true;
  pad.classList.add('act');
  try { pad.setPointerCapture(ev.pointerId); } catch(e) {}
  jDesde(ev);
});
pad.addEventListener('pointermove', ev => { if (jAct) { ev.preventDefault(); jDesde(ev); } });
['pointerup','pointercancel'].forEach(t => pad.addEventListener(t, jSoltar));
window.addEventListener('blur', jSoltar);
document.addEventListener('visibilitychange', () => { if (document.hidden) jSoltar(); });

// Latido: refresca el mando mientras se sujeta el stick.
setInterval(() => { if (jAct) man(); }, 150);
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

    const obs = s.reto=='obstaculos';
    document.getElementById('rAbierto').className = obs ? 'off' : '';
    document.getElementById('rObs').className     = obs ? '' : 'off';
    document.getElementById('cajaObs').style.display = obs ? '' : 'none';

    const n = s.navegacion, c = s.carrera;
    const lado = c.lado_interno==0 ? 'buscando' : (c.lado_interno<0?'izquierda':'derecha');
    document.getElementById('vCarrera').textContent =
      'vuelta '+c.vueltas+'/3 · '+c.giros+' giros · int '+lado+' · yaw '+c.yaw_acum+'°';

    if (Date.now() - tocando > 1500) {
      document.getElementById('vmax').value    = s.limites.vmax;
      document.getElementById('crucero').value = s.limites.vel_crucero;
      document.getElementById('giro').value    = s.limites.vel_giro;
      document.getElementById('pared').value   = n.pared_objetivo_mm;
      document.getElementById('girz').value    = n.giro_z_mm;
      document.getElementById('ext').value     = n.min_externo_mm;
      document.getElementById('kpl').value     = Math.round(n.aprox_max_grados);
      document.getElementById('kdl').value     = Math.round(n.aprox_grados_por_mm*100);
      document.getElementById('kpr').value     = Math.round(n.kp_rumbo*10);
      document.getElementById('kpg').value     = Math.round(n.giro_kp*10);
    }
    document.getElementById('vVmax').textContent = s.limites.vmax;
    document.getElementById('vCru').textContent  = s.limites.vel_crucero+'%';
    document.getElementById('vGir').textContent  = s.limites.vel_giro+'%';
    document.getElementById('vPar').textContent  = n.pared_objetivo_mm+' mm';
    document.getElementById('vGz').textContent   = n.giro_z_mm+' mm';
    document.getElementById('vExt').textContent  = n.min_externo_mm+' mm';
    document.getElementById('vKl').textContent   = n.aprox_max_grados.toFixed(0)+'°';
    document.getElementById('vKdl').textContent  = n.aprox_grados_por_mm.toFixed(2);
    document.getElementById('vKr').textContent   = n.kp_rumbo.toFixed(1);
    document.getElementById('vKg').textContent   = n.giro_kp.toFixed(1);

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
    const mm = v => (v===undefined||v<0) ? '-' : Math.round(v)+' mm';
    fila(t,'Izq / frente / der', mm(m.izq_mm)+'  '+mm(m.frente_mm)+'  '+mm(m.der_mm));
    fila(t,'Esquina interna',
         m.esquina_z===undefined ? 'no visible'
           : (Math.round(m.esquina_z)+' mm  ('+(m.esquina_lado<0?'izq':'der')+')'),
         m.esquina_z===undefined ? 'avi' : 'ok');
    fila(t,'Cobertura del muro', (m.cobertura!==undefined? m.cobertura : '-'),
         (m.cobertura!==undefined && m.cobertura < 0.25) ? 'mal' : 'ok');
    fila(t,'Suelo', s.suelo.calibrado ? 'homografia medida' : 'SIN CALIBRAR (aproximado)',
         s.suelo.calibrado ? 'ok' : 'avi');
    if (s.senal) fila(t,'Señal activa',
         s.senal.color+' a '+s.senal.z+' mm, paso por '+s.senal.lado,
         s.senal.color=='rojo'?'mal':'ok');
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
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass          # el movil cerro la pestaña; normal (en Windows llega
                          # como ConnectionAborted en vez de BrokenPipe)

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
                elif k == "reto":
                    r.fijar_reto("obstaculos" if v.startswith("obs") else "abierto")
                elif k == "reiniciar_carrera":
                    r.navegador.reiniciar()
                elif k == "lado_interno":
                    # Escape manual por si el sensor de color o la primera
                    # esquina fallan durante las pruebas. En competencia el
                    # sentido tiene que deducirse solo (regla 9.9).
                    r.navegador.lado_interno = 0
                    r.navegador.fijar_lado_interno(-1 if v.startswith("i") else 1)
                elif k in ("pared_objetivo_mm", "aprox_grados_por_mm",
                           "aprox_max_grados", "kd_rumbo",
                           "kp_rumbo", "kp_centrado", "min_externo_mm",
                           "kp_externo", "giro_z_mm", "giro_frente_mm",
                           "giro_grados", "giro_kp", "giro_tolerancia",
                           "dir_giro", "yaw_kp", "yaw_max", "frenar_mm",
                           "parar_mm", "salir_bloqueo_mm", "cobertura_min",
                           "salto_min_mm", "semiancho_carro_mm",
                           "mm_por_seg_a_100", "parada_tras_giro_mm",
                           "ignorar_abajo", "ruedas_izq", "ruedas_der"):
                    nav_cfg[k] = float(v)
                elif k in ("alto_min_muro_px", "suavizado_mm", "giro_max_ms",
                           "min_recto_ms", "salto_corrida",
                           "frames_para_fijar_lado"):
                    nav_cfg[k] = int(float(v))
                elif k in ("senal_margen_mm", "senal_z_max_mm", "senal_z_soltar_mm",
                           "senal_desvio_max_mm", "senal_aspecto_min"):
                    r.cfg["obstaculos"][k] = float(v)
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
