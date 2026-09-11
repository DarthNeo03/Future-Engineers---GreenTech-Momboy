#!/usr/bin/env bash
# ===========================================================================
# piloto.sh — el piloto como demonio: sin web y sin terminal atada.
#
#   ./piloto.sh arrancar [args]   en segundo plano SIN la web (competencia)
#   ./piloto.sh parar             SIGTERM: para el motor y cierra limpio
#   ./piloto.sh reiniciar [args]
#   ./piloto.sh estado            si corre, desde cuando y con que argumentos
#   ./piloto.sh log [-f]          el registro (todo lo que imprime main.py)
#   ./piloto.sh web [args]        igual pero CON la web (pruebas por VNC/SSH)
#   ./piloto.sh consola [args]    en primer plano (lo que usa systemd)
#   ./piloto.sh servicio          arranque automatico al encender (systemd)
#   ./piloto.sh quitar-servicio
#
# POR SSH (el carro sin pantalla, desde el portatil):
#   ssh pi@carrito.local '~/piloto/piloto.sh arrancar'
#   ssh pi@carrito.local '~/piloto/piloto.sh estado'
#   ssh pi@carrito.local '~/piloto/piloto.sh parar'
# El proceso queda en su propia sesion (setsid) con la salida al log y la
# entrada en /dev/null, asi que el comando vuelve enseguida y cerrar el SSH
# no se lleva el carro por delante.
#
# POR VNC: exactamente los mismos comandos en cualquier terminal del
# escritorio. Para mirar la telemetria con el navegador de la Pi, arranca con
# './piloto.sh web' y abre http://localhost:8080/.
#
# EN COMPETENCIA se corre 'arrancar' (o el servicio de systemd): sin web, sin
# terminal y sin nadie mirando. El start y la emergencia son EL MISMO
# PULSADOR, que cuelga del ESP32 y llega por la trama de sensores; el LED azul
# de la placa dice en que estado esta el carro (params: grupo 'botones'; ver
# src/botones.py y esp32_carro/botones.h).
#
# Variables que se pueden exportar antes de llamar:
#   PILOTO_PYTHON   python a usar (por defecto .venv/bin/python o python3)
#   PILOTO_ARGS     argumentos fijos extra (p.ej. "--perfil pabellon")
#   PILOTO_ESTADO   carpeta del pid y del log
#   PILOTO_LOG      ruta del log
# ===========================================================================
set -u

RAIZ="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ESTADO_DIR="${PILOTO_ESTADO:-${XDG_STATE_HOME:-$HOME/.local/state}/piloto}"
PIDFILE="$ESTADO_DIR/piloto.pid"
LOG="${PILOTO_LOG:-$ESTADO_DIR/piloto.log}"
LOG_MAX_KB="${PILOTO_LOG_MAX_KB:-8192}"
ARGS_FIJOS="${PILOTO_ARGS:-}"
UNIDAD="/etc/systemd/system/piloto.service"

# -- utilidades -------------------------------------------------------------
python_bin() {
  if [ -n "${PILOTO_PYTHON:-}" ]; then echo "$PILOTO_PYTHON"
  elif [ -x "$RAIZ/.venv/bin/python" ]; then echo "$RAIZ/.venv/bin/python"
  elif command -v python3 >/dev/null 2>&1; then echo "python3"
  else echo "python"; fi
}

# Imprime el PID si el proceso del pidfile sigue vivo Y sigue siendo NUESTRO
# main.py (los PID se reciclan: matar al inquilino nuevo seria peor).
pid_vivo() {
  [ -f "$PIDFILE" ] || return 1
  local p; p="$(cat "$PIDFILE" 2>/dev/null || true)"
  case "$p" in ''|*[!0-9]*) return 1 ;; esac
  kill -0 "$p" 2>/dev/null || return 1
  if [ -r "/proc/$p/cmdline" ]; then
    tr '\0' ' ' < "/proc/$p/cmdline" | grep -q "main.py" || return 1
  fi
  echo "$p"
}

systemd_activo() {
  command -v systemctl >/dev/null 2>&1 || return 1
  systemctl is-active --quiet piloto.service 2>/dev/null
}

rotar_log() {
  mkdir -p "$(dirname "$LOG")"
  [ -f "$LOG" ] || return 0
  local kb; kb="$(du -k "$LOG" 2>/dev/null | cut -f1)"
  [ -n "$kb" ] && [ "$kb" -ge "$LOG_MAX_KB" ] && mv -f "$LOG" "$LOG.1"
  return 0
}

# -- ordenes ----------------------------------------------------------------
arrancar() {                     # arrancar <con_web:0|1> [args...]
  local con_web="$1"; shift
  if systemd_activo; then
    echo "el servicio piloto.service ya lo esta corriendo."
    echo "usa: sudo systemctl restart|stop piloto"
    return 1
  fi
  local p
  if p="$(pid_vivo)"; then
    echo "ya esta corriendo (PID $p). Log: $LOG"
    return 0
  fi
  rm -f "$PIDFILE"
  mkdir -p "$ESTADO_DIR"
  rotar_log

  local PY; PY="$(python_bin)"
  local args=()
  [ "$con_web" = "0" ] && args+=(--sin-web)
  # shellcheck disable=SC2206
  [ -n "$ARGS_FIJOS" ] && args+=($ARGS_FIJOS)
  args+=("$@")

  {
    echo
    echo "== $(date '+%F %T') arrancando: $PY -u main.py ${args[*]} =="
  } >> "$LOG"

  # setsid: sesion propia, no la del SSH -> al cerrar la conexion sigue vivo.
  # (si la maquina no tiene setsid, nohup solo ya aguanta el cierre)
  # -u: sin buffer, para que 'log -f' vaya en directo.
  # OJO: se lanza como orden SUELTA, no dentro de un 'cd X && ...', porque
  # entonces $! seria el PID de la subshell y no el del piloto: se apuntaria
  # un proceso que no es, y 'parar' mataria al padre dejando el carro vivo.
  cd "$RAIZ" || { echo "no se puede entrar en $RAIZ"; return 1; }
  local lanza=(setsid)
  command -v setsid >/dev/null 2>&1 || lanza=()
  "${lanza[@]}" nohup "$PY" -u "$RAIZ/main.py" "${args[@]}" \
      </dev/null >>"$LOG" 2>&1 &
  echo $! > "$PIDFILE"

  sleep 2
  # Si setsid tuvo que duplicarse, el PID apuntado no es el del piloto:
  # antes de darlo por muerto se busca el proceso de verdad.
  if ! pid_vivo >/dev/null && command -v pgrep >/dev/null 2>&1; then
    local real; real="$(pgrep -f "$RAIZ/main.py" 2>/dev/null | head -n1)"
    [ -n "$real" ] && echo "$real" > "$PIDFILE"
  fi
  if p="$(pid_vivo)"; then
    echo "piloto en marcha (PID $p)$([ "$con_web" = 1 ] && echo ' CON web' || echo ', sin web')"
    echo "log: $LOG   (./piloto.sh log -f para verlo en directo)"
    [ "$con_web" = 1 ] && echo "web: http://$(hostname).local:8080/  o  http://localhost:8080/"
    return 0
  fi
  echo "NO arranco. Ultimas lineas del log:"
  tail -n 25 "$LOG"
  rm -f "$PIDFILE"
  return 1
}

parar() {
  if systemd_activo; then
    echo "lo maneja systemd: sudo systemctl stop piloto"
    return 1
  fi
  local p
  if ! p="$(pid_vivo)"; then
    echo "no estaba corriendo"
    rm -f "$PIDFILE"
    return 0
  fi
  # SIGTERM lo atiende main.py: emergencia() -> motor a cero y servo al
  # centro -> cierra el puerto serie. Matar a lo bruto deja el ESP32 con la
  # ultima orden hasta que salte su vigilante de 300 ms.
  echo "parando (PID $p): parada de emergencia y cierre limpio..."
  kill -TERM "$p" 2>/dev/null
  local i
  for i in $(seq 1 40); do
    if ! pid_vivo >/dev/null; then
      echo "parado limpiamente"
      rm -f "$PIDFILE"
      return 0
    fi
    sleep 0.25
  done
  echo "no responde en 10 s: SIGKILL"
  kill -KILL "$p" 2>/dev/null
  kill -KILL -- "-$p" 2>/dev/null
  rm -f "$PIDFILE"
}

estado() {
  local p
  if systemd_activo; then
    echo "servicio systemd ACTIVO (systemctl status piloto)"
  fi
  if p="$(pid_vivo)"; then
    local cmd="?" desde="?"
    [ -r "/proc/$p/cmdline" ] && cmd="$(tr '\0' ' ' < "/proc/$p/cmdline")"
    desde="$(ps -o etime= -p "$p" 2>/dev/null | tr -d ' ')"
    echo "CORRIENDO   PID $p   desde hace ${desde:-?}"
    echo "orden:      $cmd"
    case "$cmd" in
      *--sin-web*) echo "web:        NO (modo competencia)" ;;
      *)           echo "web:        si, http://localhost:8080/" ;;
    esac
  else
    echo "PARADO"
  fi
  echo "log:        $LOG"
  if [ -f "$LOG" ]; then
    echo "--- ultimas lineas ---"
    tail -n 8 "$LOG"
  fi
}

ver_log() {
  [ -f "$LOG" ] || { echo "todavia no hay log en $LOG"; return 0; }
  if [ "${1:-}" = "-f" ]; then tail -n 40 -f "$LOG"; else tail -n 60 "$LOG"; fi
}

servicio() {
  command -v systemctl >/dev/null 2>&1 || { echo "esta Pi no usa systemd"; return 1; }
  local usuario="${SUDO_USER:-$USER}"
  local PY; PY="$(python_bin)"
  echo "instalando $UNIDAD (usuario $usuario)..."
  sudo tee "$UNIDAD" >/dev/null <<UNIT
[Unit]
Description=Piloto WRO 2026 (GreenTech Momboy)
After=network.target

[Service]
Type=simple
User=$usuario
WorkingDirectory=$RAIZ
ExecStart=$PY -u $RAIZ/main.py --sin-web $ARGS_FIJOS
# SIGTERM es la parada de emergencia de main.py: hay que dejarle terminar.
KillSignal=SIGTERM
TimeoutStopSec=15
Restart=on-failure
RestartSec=3
# Puerto serie del ESP32 (por ahi llega tambien el boton) y camara USB.
SupplementaryGroups=dialout video
StandardOutput=append:$LOG
StandardError=append:$LOG

[Install]
WantedBy=multi-user.target
UNIT
  sudo systemctl daemon-reload
  sudo systemctl enable piloto.service
  echo "listo. Arranca ya con:  sudo systemctl start piloto"
  echo "y en cada encendido se lanza solo, sin web."
}

quitar_servicio() {
  command -v systemctl >/dev/null 2>&1 || return 0
  sudo systemctl disable --now piloto.service 2>/dev/null
  sudo rm -f "$UNIDAD"
  sudo systemctl daemon-reload
  echo "servicio quitado"
}

ayuda() {
  # El bloque de comentarios de la cabecera, sin los '#'. Se corta solo en la
  # primera linea de codigo, asi que no hay que ajustar numeros al editarlo.
  awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "${BASH_SOURCE[0]}"
}

# -- despacho ---------------------------------------------------------------
orden="${1:-ayuda}"; shift || true
case "$orden" in
  arrancar|start)        arrancar 0 "$@" ;;
  web)                   arrancar 1 "$@" ;;
  parar|stop)            parar ;;
  reiniciar|restart)     parar; sleep 0.5; arrancar 0 "$@" ;;
  estado|status)         estado ;;
  log|registro)          ver_log "${1:-}" ;;
  consola|run)           cd "$RAIZ" && exec "$(python_bin)" -u "$RAIZ/main.py" --sin-web $ARGS_FIJOS "$@" ;;
  servicio|instalar)     servicio ;;
  quitar-servicio)       quitar_servicio ;;
  ayuda|-h|--help|help)  ayuda ;;
  *) echo "orden desconocida: $orden"; echo; ayuda; exit 2 ;;
esac
