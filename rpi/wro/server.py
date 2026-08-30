# -*- coding: utf-8 -*-
"""
Servidor web de depuracion y calibracion.

IMPORTANTE (regla 11.10): durante las rondas de competencia NO se permite Wi-Fi
ni ninguna comunicacion inalambrica en el vehiculo. Este servidor es solo para
las pruebas. Arranca con --no-web (o apaga la Wi-Fi de la Pi) para competir.
"""

from __future__ import annotations

import os
import time

from flask import (Flask, Response, jsonify, request,
                   send_from_directory)

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
BOUNDARY = b"--frame"

# Werkzeug responde HTTP/1.0 por defecto, y eso CIERRA la conexion despues de
# cada peticion. Con el movil eso significa abrir un TCP nuevo para cada
# comando del joystick (20 por segundo) mientras el stream MJPEG ya ocupa una
# conexion: las peticiones se encolan, dejan de llegar comandos y el dead-man
# frena el coche a rachas. Con HTTP/1.1 hay keep-alive y el problema desaparece.
try:
    from werkzeug.serving import WSGIRequestHandler
    WSGIRequestHandler.protocol_version = "HTTP/1.1"
except Exception:                                        # pragma: no cover
    pass


def create_app(robot, cfg):
    app = Flask(__name__, static_folder=None)
    app.config["JSON_SORT_KEYS"] = False

    # ------------------------------------------------------------- estaticos
    @app.route("/")
    def index():
        return send_from_directory(STATIC, "index.html")

    @app.route("/static/<path:name>")
    def static_files(name):
        return send_from_directory(STATIC, name)

    # ---------------------------------------------------------------- video
    @app.route("/stream.mjpg")
    def stream():
        view = request.args.get("view", "overlay")
        if view not in ("overlay", "mask", "bev", "raw"):
            view = "overlay"

        def gen():
            # Solo un cliente dibuja a la vez. Si otro tiene tomada la vista
            # (caso tipico: este stream quedo colgado al cambiar de pestana y
            # el navegador no cerro la conexion), este se retira solo en 3 s
            # en lugar de seguir forzando dibujado para siempre.
            last = None
            denied_since = None
            try:
                while True:
                    if robot.request_view(view):
                        denied_since = None
                        buf = robot.get_jpeg(view)
                        if buf is not None and buf is not last:
                            last = buf
                            yield (BOUNDARY + b"\r\nContent-Type: image/jpeg\r\n"
                                   b"Content-Length: " + str(len(buf)).encode() +
                                   b"\r\n\r\n" + buf + b"\r\n")
                    else:
                        if denied_since is None:
                            denied_since = time.time()
                        elif time.time() - denied_since > 3.0:
                            return
                    time.sleep(0.045)
            finally:
                robot.release_view(view)

        return Response(gen(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/snapshot.jpg")
    def snapshot():
        view = request.args.get("view", "overlay")
        try:
            for _ in range(30):
                robot.request_view(view)
                buf = robot.get_jpeg(view)
                if buf:
                    return Response(buf, mimetype="image/jpeg")
                time.sleep(0.05)
        finally:
            robot.release_view(view)
        return ("sin imagen", 503)

    # ------------------------------------------------------------------ api
    @app.route("/api/status")
    def api_status():
        return jsonify(robot.status())

    @app.route("/api/schema")
    def api_schema():
        return jsonify({"params": cfg.schema(), "values": cfg.snapshot()})

    @app.route("/api/config", methods=["GET", "POST"])
    def api_config():
        if request.method == "GET":
            return jsonify(cfg.snapshot())
        data = request.get_json(force=True, silent=True) or {}
        persist = bool(data.pop("__save", True))
        changed = cfg.set_many(data)
        if changed:
            robot.on_config_changed(changed)
            if persist:
                cfg.save()
        return jsonify({"changed": changed, "values": cfg.snapshot()})

    @app.route("/api/config/reset", methods=["POST"])
    def api_reset():
        data = request.get_json(force=True, silent=True) or {}
        keys = data.get("keys")
        cfg.reset(keys)
        cfg.save()
        robot.on_config_changed(cfg.snapshot())
        robot.push_esp_params()
        return jsonify({"values": cfg.snapshot()})

    @app.route("/api/command", methods=["POST"])
    def api_command():
        d = request.get_json(force=True, silent=True) or {}
        cmd = str(d.get("cmd", ""))

        if cmd == "arm":
            robot.arm()
        elif cmd == "disarm":
            robot.disarm()
        elif cmd == "mode":
            robot.set_mode(str(d.get("mode", "open")))
        elif cmd == "manual":
            robot.set_manual(float(d.get("steer", 0)), float(d.get("speed", 0)))
        elif cmd == "zero_yaw":
            robot.esp.zero_yaw()
        elif cmd == "reset_lines":
            robot.esp.reset_lines()
        elif cmd == "push_params":
            robot.push_esp_params()
        elif cmd == "save":
            cfg.save()
        elif cmd == "calibrate_pitch":
            robot.request_pitch_calibration(float(d.get("distance_mm", 400)))
        elif cmd == "calibrate_hfov":
            robot.request_hfov_calibration(float(d.get("corridor_mm", 1000)))
        elif cmd == "auto_threshold":
            robot.request_auto_threshold()
        elif cmd == "esp_raw":
            robot.esp.raw(str(d.get("line", "")))
        else:
            return jsonify({"ok": False, "error": "comando desconocido"}), 400
        return jsonify({"ok": True, "msg": robot.msg})

    @app.route("/api/camera_info")
    def api_camera_info():
        return jsonify({"device": robot.cam.device,
                        "negotiated": robot.cam.negotiated,
                        "ctrl": robot.cam.ctrl_note,
                        "info": robot.cam.formats()})

    @app.route("/api/esp_log")
    def api_esp_log():
        return jsonify({"lines": list(robot.esp.rx_lines)[-60:]})

    @app.after_request
    def no_cache(resp):
        resp.headers["Cache-Control"] = "no-store"
        return resp

    return app
