#!/usr/bin/env bash
# ===========================================================================
# carrito_wifi.sh — Deja la Raspberry Pi como punto de acceso "Carrito-WRO"
# y accesible en http://carrito.local:8080/
#
#   sudo bash tools/carrito_wifi.sh instalar     # una sola vez
#   sudo bash tools/carrito_wifi.sh ap           # encender el AP
#   sudo bash tools/carrito_wifi.sh cliente      # volver a tu WiFi de casa
#   sudo bash tools/carrito_wifi.sh servicio     # arrancar main.py al encender
#   sudo bash tools/carrito_wifi.sh estado
#
# Pensado para Raspberry Pi OS Bookworm (Pi 5), que usa NetworkManager.
# El AP se crea con nmcli, sin hostapd ni dnsmasq a mano: NetworkManager ya
# levanta el DHCP y el DNS del lado del AP.
#
# En el pabellon de la competencia NO uses la red del recinto: hay decenas de
# equipos y el WiFi se satura. Con el AP propio el movil se conecta directo a
# la Pi y la latencia baja muchisimo.
# ===========================================================================
set -euo pipefail

SSID="${SSID:-Carrito-WRO}"
PASS="${PASS:-wro2026carro}"       # minimo 8 caracteres
NOMBRE="${NOMBRE:-carrito}"
IFAZ="${IFAZ:-wlan0}"
CONEXION="carrito-ap"
USUARIO="${SUDO_USER:-$(whoami)}"
PROYECTO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

necesita_root() {
  if [ "$(id -u)" -ne 0 ]; then echo "Ejecuta con sudo."; exit 1; fi
}

instalar() {
  necesita_root
  echo ">> Instalando avahi (mDNS) y utilidades"
  apt-get update -qq
  apt-get install -y -qq avahi-daemon avahi-utils network-manager python3-tk i2c-tools

  echo ">> Poniendo el nombre de la maquina en '$NOMBRE'"
  hostnamectl set-hostname "$NOMBRE"
  # /etc/hosts tiene que concordar o sudo se queja en cada comando
  sed -i "s/^127.0.1.1.*/127.0.1.1\t$NOMBRE/" /etc/hosts || true
  systemctl enable --now avahi-daemon

  echo ">> Habilitando el UART de los GPIO 14/15 y liberando la consola serie"
  # En la Pi 5 el UART de la cabecera es /dev/ttyAMA0.
  CONF=/boot/firmware/config.txt
  [ -f "$CONF" ] || CONF=/boot/config.txt
  grep -q "^enable_uart=1" "$CONF" || echo "enable_uart=1" >> "$CONF"
  # Quitar la consola del puerto serie: si no, el kernel escribe basura por TX
  # y el ESP32 recibe ruido constante.
  systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
  sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt 2>/dev/null || true
  sed -i 's/console=ttyAMA0,115200 //' /boot/firmware/cmdline.txt 2>/dev/null || true
  usermod -aG dialout,i2c,video "$USUARIO" || true

  echo ">> Habilitando I2C para el MPU6050"
  grep -q "^dtparam=i2c_arm=on" "$CONF" || echo "dtparam=i2c_arm=on" >> "$CONF"

  echo
  echo "Listo. REINICIA para que el UART y el I2C queden activos:  sudo reboot"
  echo "Despues:  sudo bash tools/carrito_wifi.sh ap"
}

ap() {
  necesita_root
  nmcli connection delete "$CONEXION" 2>/dev/null || true
  nmcli connection add type wifi ifname "$IFAZ" con-name "$CONEXION" \
      autoconnect yes ssid "$SSID"
  nmcli connection modify "$CONEXION" \
      802-11-wireless.mode ap \
      802-11-wireless.band bg \
      802-11-wireless.channel 6 \
      ipv4.method shared \
      ipv4.addresses 192.168.50.1/24 \
      wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk "$PASS"
  nmcli connection up "$CONEXION"
  echo
  echo "AP levantado."
  echo "  Red:        $SSID"
  echo "  Clave:      $PASS"
  echo "  Entra a:    http://carrito.local:8080/   o   http://192.168.50.1:8080/"
  echo
  echo "Si carrito.local no resuelve en tu movil (Android antiguo no habla mDNS),"
  echo "usa la IP 192.168.50.1 directamente."
}

cliente() {
  necesita_root
  nmcli connection down "$CONEXION" 2>/dev/null || true
  nmcli connection modify "$CONEXION" autoconnect no 2>/dev/null || true
  echo "AP apagado. Reconecta a tu WiFi normal con: nmcli device wifi connect <SSID> password <clave>"
}

servicio() {
  necesita_root
  cat > /etc/systemd/system/carrito.service <<EOF
[Unit]
Description=Carro WRO Future Engineers
After=network.target

[Service]
Type=simple
User=$USUARIO
WorkingDirectory=$PROYECTO
ExecStart=$PROYECTO/.venv/bin/python3 $PROYECTO/main.py --sin-panel
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable carrito.service
  echo "Servicio creado. Arrancar ahora:  sudo systemctl start carrito"
  echo "Ver el log:                       journalctl -u carrito -f"
  echo
  echo "OJO: el carro arranca SIEMPRE DESARMADO. Aunque el servicio este activo"
  echo "el motor no se mueve hasta que pulses ARMAR en carrito.local:8080."
}

estado() {
  echo "--- hostname ---";  hostname
  echo "--- avahi ---";     systemctl is-active avahi-daemon || true
  echo "--- conexiones ---"; nmcli -t -f NAME,TYPE,DEVICE connection show --active || true
  echo "--- ip ---";        ip -4 addr show "$IFAZ" 2>/dev/null | grep inet || true
  echo "--- serie ---";     ls -l /dev/serial0 /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
  echo "--- i2c ---";       i2cdetect -y 1 2>/dev/null || echo "i2c no disponible"
}

case "${1:-}" in
  instalar) instalar ;;
  ap)       ap ;;
  cliente)  cliente ;;
  servicio) servicio ;;
  estado)   estado ;;
  *) echo "uso: sudo bash $0 {instalar|ap|cliente|servicio|estado}"; exit 1 ;;
esac
