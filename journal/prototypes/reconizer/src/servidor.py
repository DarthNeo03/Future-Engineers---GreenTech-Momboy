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
 .chk{display:block;margin:6px 0;color:#c8ccd2;font-size:.9em}
 .chk input{width:20px;height:20px;vertical-align:-4px;margin-right:6px}
 i{font-style:normal;color:#4ade80;font-size:.85em}
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
  <div class="et"><span>Mezcla de navegaciones</span><span id="vEst">-</span></div>
  <div class="et"><span>Centrado por espacio libre</span><span id="vMc">-</span></div>
  <input type="range" id="mc" min="0" max="100" oninput="lz('peso_centrado',this.value/100)">
  <div class="et"><span>Seguir pared externa</span><span id="vMp">-</span></div>
  <input type="range" id="mp" min="0" max="100" oninput="lz('peso_pared',this.value/100)">
  <div class="et"><span>Hueco pasable (ancho de ruedas)</span><span id="vMh">-</span></div>
  <input type="range" id="mh" min="0" max="100" oninput="lz('peso_hueco',this.value/100)">
  <div class="fila">
    <button onclick="solo('centrado')" class="off">Solo centrado</button>
    <button onclick="solo('pared')" class="off">Solo pared</button>
    <button onclick="solo('hueco')" class="off">Solo hueco</button>
  </div>
  <div class="et"><span>Kp</span><span id="vKp">-</span></div>
  <input type="range" id="kp" min="0" max="300" oninput="lz('kp',this.value)">
  <div class="et"><span>Kd</span><span id="vKd">-</span></div>
  <input type="range" id="kd" min="0" max="200" oninput="lz('kd',this.value)">
  <div class="et"><span>Umbral de giro <i id="vAuto"></i></span><span id="vGb">-</span></div>
  <input type="range" id="girar" min="0" max="100" oninput="lz('girar_bajo',this.value/100)">
  <div class="et"><span>Anticipacion: segundos hasta el muro</span><span id="vTtc">-</span></div>
  <input type="range" id="ttc" min="0" max="40" oninput="lz('ttc_min',this.value/10)">
  <label class="chk"><input type="checkbox" id="cEsq"
    onchange="cmd('usar_esquina_interna',this.checked?1:0)"> girar cuando el muro interno desaparece</label>
  <label class="chk"><input type="checkbox" id="cAuto"
    onchange="cmd('autocalibrar_carril',this.checked?1:0)"> calibrar el ancho del carril solo</label>
</div>

<div class="caja">
  <div class="et"><span>Esquivar pilares (rojo por la derecha, verde por la izquierda)</span>
    <span id="vObs">-</span></div>
  <label class="chk"><input type="checkbox" id="cObs"
    onchange="cmd('esquivar',this.checked?1:0)"> esquivar obstaculos de colores</label>
  <div class="et"><span>Separacion lateral</span><span id="vMl">-</span></div>
  <input type="range" id="ml" min="50" max="250" oninput="lz('margen_lateral',this.value/100)">
  <div class="et"><span>Empieza a hacerle caso</span><span id="vAd">-</span></div>
  <input type="range" id="ad" min="10" max="90" oninput="lz('activar_desde',this.value/100)">
  <div class="et"><span>Ya manda del todo</span><span id="vMd2">-</span></div>
  <input type="range" id="md2" min="20" max="99" oninput="lz('mandar_desde',this.value/100)">
</div>

<div class="caja">
  <div class="et"><span>Sentido de la vuelta</span><span id="vSen">-</span></div>
  <div class="fila">
    <button onclick="cmd('forzar_sentido','0')" id="sAuto">Automatico</button>
    <button onclick="cmd('forzar_sentido','-1')" id="sAnti" class="off">Externa izq<br>(antihorario)</button>
    <button onclick="cmd('forzar_sentido','1')" id="sHor" class="off">Externa der<br>(horario)</button>
  </div>
</div>

<div class="caja">
  <div class="et"><span>Vueltas</span><span id="vVta">-</span></div>
  <input type="range" id="objetivo" min="1" max="10" oninput="lz('objetivo',this.value)">
  <label class="chk"><input type="checkbox" id="cMv"
    onchange="cmd('hacer_media_vuelta',this.checked?1:0)"> media vuelta y volver</label>
  <div class="fila">
    <button onclick="cmd('tipo_media_vuelta','recta_3t')" id="mvR">En recta (3 tiempos)</button>
    <button onclick="cmd('tipo_media_vuelta','esquina')" id="mvE" class="off">En la esquina</button>
  </div>
  <div class="fila">
    <button onclick="cmd('nueva_carrera','1')" class="off">Reiniciar contador</button>
    <button onclick="cmd('media_vuelta_ya','1')" class="off">Media vuelta ya</button>
  </div>
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
<div class="fila">
  <button onclick="cmd('reintentar_sensores','1')" class="off">Reintentar sensores I2C</button>
  <button onclick="cmd('calibrar_color','1')" class="off">Calibrar piso blanco</button>
</div>

<script>
let ultimo = 0, tocando = 0;
function cmd(k,v){ fetch('/api/cmd?'+k+'='+encodeURIComponent(v)).then(estado); }
function lz(k,v){ tocando = Date.now(); cmd(k,v); }
function solo(n){ tocando = 0; cmd('solo',n); }
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

    const mz = s.navegacion.mezcla || {};
    if (Date.now() - tocando > 1500) {
      document.getElementById('vmax').value    = s.limites.vmax;
      document.getElementById('crucero').value = s.limites.vel_crucero;
      document.getElementById('giro').value    = s.limites.vel_giro;
      document.getElementById('kp').value      = s.navegacion.kp;
      document.getElementById('kd').value      = s.navegacion.kd;
      document.getElementById('girar').value   = Math.round(s.navegacion.girar_bajo*100);
      document.getElementById('ttc').value     = Math.round(s.navegacion.ttc_min*10);
      document.getElementById('mc').value      = Math.round((mz.centrado||0)*100);
      document.getElementById('mp').value      = Math.round((mz.pared||0)*100);
      document.getElementById('mh').value      = Math.round((mz.hueco||0)*100);
      document.getElementById('objetivo').value= s.vueltas.objetivo;
      document.getElementById('cEsq').checked  = !!s.navegacion.usar_esquina_interna;
      document.getElementById('cAuto').checked = !!s.navegacion.autocalibrar_carril;
      document.getElementById('cMv').checked   = !!s.vueltas_cfg.hacer_media_vuelta;
      document.getElementById('cObs').checked  = !!s.obstaculos_cfg.activo;
      document.getElementById('ml').value  = Math.round(s.obstaculos_cfg.margen_lateral*100);
      document.getElementById('ad').value  = Math.round(s.obstaculos_cfg.activar_desde*100);
      document.getElementById('md2').value = Math.round(s.obstaculos_cfg.mandar_desde*100);
    }
    document.getElementById('vMl').textContent  = s.obstaculos_cfg.margen_lateral.toFixed(2);
    document.getElementById('vAd').textContent  = s.obstaculos_cfg.activar_desde.toFixed(2);
    document.getElementById('vMd2').textContent = s.obstaculos_cfg.mandar_desde.toFixed(2);
    document.getElementById('vObs').textContent = s.obstaculos.activo
        ? (s.obstaculos.siguiendo ? s.obstaculos.motivo : 'activa, sin pilares')
        : 'apagada';
    document.getElementById('vVmax').textContent = s.limites.vmax;
    document.getElementById('vCru').textContent  = s.limites.vel_crucero+'%';
    document.getElementById('vGir').textContent  = s.limites.vel_giro+'%';
    document.getElementById('vKp').textContent   = s.navegacion.kp;
    document.getElementById('vKd').textContent   = s.navegacion.kd;
    document.getElementById('vTtc').textContent  = s.navegacion.ttc_min.toFixed(1)+' s';
    document.getElementById('vMc').textContent   = (mz.centrado||0).toFixed(2);
    document.getElementById('vMp').textContent   = (mz.pared||0).toFixed(2);
    document.getElementById('vMh').textContent   = (mz.hueco||0).toFixed(2);
    document.getElementById('vVta').textContent  =
        s.vueltas.vueltas+' / '+s.vueltas.objetivo+'  ('+s.vueltas.tramo+')';
    const fz = (s.decision.metricas.sentido||{}).forzado || 0;
    document.getElementById('sAuto').className = fz===0 ? '' : 'off';
    document.getElementById('sAnti').className = fz<0  ? '' : 'off';
    document.getElementById('sHor').className  = fz>0  ? '' : 'off';
    const sen = s.decision.metricas.sentido || {};
    document.getElementById('vSen').textContent =
        (sen.nombre||'?') + '  externa ' + (sen.externa||'?') + (fz?'  (fijado)':'');

    const mvr = s.vueltas_cfg.tipo_media_vuelta === 'esquina';
    document.getElementById('mvR').className = mvr ? 'off' : '';
    document.getElementById('mvE').className = mvr ? '' : 'off';

    const m = s.decision.metricas;
    const auto = m.umbrales || {};
    const esAuto = s.navegacion.autocalibrar_carril && m.carril && m.carril.listo;
    document.getElementById('vGb').textContent =
        (esAuto ? auto.girar_bajo : s.navegacion.girar_bajo).toFixed(2);
    document.getElementById('vAuto').textContent = esAuto ? '(auto)' : '';
    let activas = [];
    for (const k in mz) if (mz[k] > 0) activas.push(k+' '+mz[k].toFixed(1));
    document.getElementById('vEst').textContent = activas.join(' + ') || s.navegacion.estrategia;

    const t = document.getElementById('tel'); t.innerHTML='';
    const e = s.enlace, sn = s.sensores;
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
    if (m.ttc!==undefined) fila(t,'Segundos al muro', m.ttc, m.ttc<1.2?'avi':'ok');
    if (m.sentido) fila(t,'Sentido', m.sentido.nombre+'  externa '+m.sentido.externa+
                        (m.sentido.bloqueado?'  [bloqueado]':'')+
                        (m.sentido.forzado?'  [fijado]':''),
                        m.sentido.sentido?'ok':'avi');
    if (m.sentido) fila(t,'Presencia izq/der', m.sentido.presencia[0]+'  /  '+m.sentido.presencia[1]);
    if (m.muros_vistos) fila(t,'Muro visible', 'izq '+(m.muros_vistos.izq?'si':'NO')+
                             '   der '+(m.muros_vistos.der?'si':'NO'));
    if (s.obstaculos.siguiendo) fila(t,'Pilar', s.obstaculos.motivo+
        '  peso '+s.obstaculos.peso, 'ok');
    fila(t,'Esquinas ignoradas', s.vueltas.ignoradas);
    if (m.carril) fila(t,'Ancho de carril',
         m.carril.listo ? m.carril.ancho : ('midiendo '+m.carril.muestras));
    if (m.hueco) fila(t,'Mejor hueco','x'+m.hueco.x+'  margen '+m.hueco.margen+
         (m.hueco.pasable?' (cabe)':' (NO cabe)'), m.hueco.pasable?'ok':'avi');
    fila(t,'Vueltas', s.vueltas.vueltas+'/'+s.vueltas.objetivo+'  esquina '+
         s.vueltas.esquinas+'/'+s.vueltas.esquinas_por_vuelta+
         (s.vueltas.ultima_por?('  ['+s.vueltas.ultima_por+']'):''));
    fila(t,'Rumbo', sn.rumbo.origen==='ninguno' ? 'sin giroscopio'
         : ('yaw '+sn.rumbo.yaw+'&deg;  ('+sn.rumbo.origen+')'),
         sn.rumbo.origen==='ninguno'?'avi':'ok');
    fila(t,'Sensor de color', sn.color.origen+'  linea: '+sn.color.linea,
         sn.color.origen==='esp32'?'ok':'');
    fila(t,'Sensores ESP32','MPU '+(sn.esp32.mpu?'si':'no')+
         '   TCS '+(sn.esp32.tcs?'si':'no'));
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
                    nav_cfg["estrategia"] = v
                    r.navegador.reiniciar()
                elif k == "solo":
                    nav_cfg["mezcla"] = {n: (1.0 if n == v else 0.0)
                                         for n in ("centrado", "pared", "hueco")}
                    nav_cfg["estrategia"] = v
                    r.navegador.reiniciar()
                elif k.startswith("peso_") and k[5:] in ("centrado", "pared", "hueco"):
                    mez = dict(nav_cfg.get("mezcla") or {})
                    mez[k[5:]] = max(0.0, float(v))
                    nav_cfg["mezcla"] = mez
                elif k == "lado_pared":
                    nav_cfg["lado_pared"] = ("auto" if v.startswith("a")
                                             else ("izq" if v.startswith("i") else "der"))
                elif k in ("usar_esquina_interna", "autocalibrar_carril"):
                    nav_cfg[k] = v not in ("0", "false", "")
                elif k in ("objetivo", "esquinas_por_vuelta"):
                    r.cfg["vueltas"][k] = int(float(v))
                elif k == "hacer_media_vuelta":
                    r.cfg["vueltas"][k] = v not in ("0", "false", "")
                elif k == "tipo_media_vuelta":
                    r.cfg["vueltas"][k] = "esquina" if v.startswith("esq") else "recta_3t"
                elif k == "nueva_carrera":
                    r.nueva_carrera()
                elif k == "media_vuelta_ya":
                    r.navegador.pedir_media_vuelta()
                elif k == "calibrar_color":
                    r.sensores.calibrar_color(r.enlace)
                elif k == "reintentar_sensores":
                    r.reintentar_sensores()
                elif k == "forzar_sentido":
                    r.navegador.paredes.forzar(int(float(v)))
                elif k == "esquivar":
                    r.cfg["obstaculos"]["activo"] = v not in ("0", "false", "")
                    r.esquiva.reiniciar()
                    r.aplicar_config()
                elif k in ("margen_lateral", "activar_desde", "mandar_desde",
                           "soltar_en", "peso_max", "sesgo_siguiente"):
                    r.cfg["obstaculos"][k] = float(v)
                    r.aplicar_config()
                elif k in ("area_min_pilar", "frames_perdido"):
                    r.cfg["obstaculos"][k] = int(float(v))
                    r.aplicar_config()
                elif k in ("refractario_ms", "una_linea_basta", "dominancia_linea"):
                    r.cfg["vueltas"][k] = (v not in ("0", "false", "")
                                           if k == "una_linea_basta" else float(v))
                elif k in ("origen_rumbo", "origen_color"):
                    r.cfg["sensores"][k] = v
                elif k in ("kp", "kd", "kp_pared", "kd_pared", "pared_objetivo",
                           "girar_bajo", "frenar_bajo", "parar_bajo",
                           "salir_giro_sobre", "dir_giro", "yaw_kp",
                           "ruedas_izq", "ruedas_der", "banda_lateral",
                           "ignorar_abajo", "ttc_min", "umbral_hueco",
                           "margen_hueco", "y_horizonte", "kp_hueco", "kd_hueco",
                           "salto_min", "interno_libre", "dir_giro_abierto",
                           "vel_escape", "mejora_min", "peso_margen",
                           "peso_profundidad", "peso_siguiente", "peso_alineacion"):
                    nav_cfg[k] = float(v)
                elif k in ("escape_atras_min_ms", "escape_atras_extra_ms",
                           "escape_atascado_ms", "escape_salir_factor",
                           "cobertura_alta", "cobertura_baja", "kp_diagonal",
                           "peso_diagonal", "bloqueo_sentido_ms"):
                    nav_cfg[k] = float(v)
                elif k == "giro_diagonal":
                    nav_cfg[k] = v not in ("0", "false", "")
                elif k in ("px_min_columna", "suavizado", "giro_max_ms",
                           "min_recto_ms", "retardo_giro_ms", "ventana_salto",
                           "escape_evaluar_ms", "validez_esquina_ms"):
                    nav_cfg[k] = int(float(v))
                elif k == "usar_yaw":
                    nav_cfg["usar_yaw"] = v not in ("0", "false")
                elif k == "calibrar_imu":
                    r.sensores.calibrar_rumbo(r.enlace)
                elif k == "cero_yaw":
                    r.sensores.poner_cero(r.enlace)
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
