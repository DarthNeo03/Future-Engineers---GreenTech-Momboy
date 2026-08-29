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
            last = None
            while True:
                robot.request_view(view)
                buf = robot.get_jpeg(view)
                if buf is not None and buf is not last:
                    last = buf
                    yield (BOUNDARY + b"\r\nContent-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(buf)).encode() +
                           b"\r\n\r\n" + buf + b"\r\n")
                time.sleep(0.045)

        return Response(gen(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/snapshot.jpg")
    def snapshot():
        view = request.args.get("view", "overlay")
        robot.request_view(view)
        for _ in range(30):
            buf = robot.get_jpeg(view)
            if buf:
                return Response(buf, mimetype="image/jpeg")
            time.sleep(0.05)
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
        elif cmd == "esp_raw":
            robot.esp.raw(str(d.get("line", "")))
        else:
            return jsonify({"ok": False, "error": "comando desconocido"}), 400
        return jsonify({"ok": True, "msg": robot.msg})

    @app.route("/api/esp_log")
    def api_esp_log():
        return jsonify({"lines": list(robot.esp.rx_lines)[-60:]})

    @app.after_request
    def no_cache(resp):
        resp.headers["Cache-Control"] = "no-store"
        return resp

    return app
