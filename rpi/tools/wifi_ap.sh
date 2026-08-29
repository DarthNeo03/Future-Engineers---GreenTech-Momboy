#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_ap.sh - Punto de acceso wifi del robot (Raspberry Pi OS Bookworm)
#
# Levanta una red wifi propia en la Pi para conectarte con el movil o el
# portatil y abrir el panel de depuracion, sin depender de la wifi del local.
#
#   sudo ./tools/wifi_ap.sh crear        # una sola vez: crea el perfil
#   sudo ./tools/wifi_ap.sh on           # enciende el AP
#   sudo ./tools/wifi_ap.sh off          # lo apaga y vuelve a wifi normal
#   sudo ./tools/wifi_ap.sh estado       # que hay levantado y con que IP
#   sudo ./tools/wifi_ap.sh competencia  # apaga wifi Y bluetooth (regla 11.10)
#
# Personalizar sin editar el fichero:
#   sudo WRO_SSID=MiCarro WRO_PASS=miclave123 ./tools/wifi_ap.sh crear
#
# AVISO DE REGLAMENTO (11.10): durante las rondas de competencia no se permite
# NINGUNA comunicacion inalambrica en el vehiculo. Usa "competencia" antes de
# cada intento y arranca el robot con:  python3 run.py --no-web
# ---------------------------------------------------------------------------
set -euo pipefail

SSID="${WRO_SSID:-WRO-CAR}"
PASS="${WRO_PASS:-wro2026robot}"        # minimo 8 caracteres (WPA2)
CON="${WRO_CON:-wro-ap}"
IFACE="${WRO_IFACE:-wlan0}"
IPADDR="${WRO_IP:-192.168.4.1}"
COUNTRY="${WRO_COUNTRY:-VE}"            # dominio regulatorio (VE = Venezuela)
PORT="${WRO_PORT:-8000}"

need_root() {
  if [ "$(id -u)" -ne 0 ]; then
    echo "Ejecutalo con sudo." >&2
    exit 1
  fi
}

case "${1:-}" in

  crear)
    need_root
    if [ "${#PASS}" -lt 8 ]; then
      echo "La clave debe tener al menos 8 caracteres (WPA2)." >&2
      exit 1
    fi

    # NetworkManager reparte IPs con dnsmasq cuando ipv4.method=shared.
    if ! dpkg -s dnsmasq-base >/dev/null 2>&1; then
      echo "==> instalando dnsmasq-base"
      apt-get update && apt-get install -y dnsmasq-base
    fi

    # Sin dominio regulatorio la radio queda bloqueada y el AP no arranca.
    echo "==> dominio regulatorio: $COUNTRY"
    raspi-config nonint do_wifi_country "$COUNTRY" || true
    rfkill unblock wifi || true

    echo "==> creando perfil '$CON' (SSID: $SSID)"
    nmcli con delete "$CON" >/dev/null 2>&1 || true
    nmcli con add type wifi ifname "$IFACE" mode ap con-name "$CON" \
          ssid "$SSID" autoconnect no
    # Banda 2.4 GHz: mas alcance y la aceptan todos los moviles.
    nmcli con modify "$CON" \
          802-11-wireless.band bg \
          802-11-wireless.channel 6 \
          802-11-wireless.hidden no \
          wifi-sec.key-mgmt wpa-psk \
          wifi-sec.proto rsn \
          wifi-sec.pairwise ccmp \
          wifi-sec.group ccmp \
          wifi-sec.psk "$PASS" \
          ipv4.method shared \
          ipv4.addresses "$IPADDR/24" \
          ipv6.method disabled
    echo "==> listo. Enciendelo con: sudo $0 on"
    ;;

  on)
    need_root
    rfkill unblock wifi || true
    nmcli con up "$CON"
    echo
    echo "  Red wifi : $SSID"
    echo "  Clave    : $PASS"
    echo "  Panel    : http://$IPADDR:$PORT"
    echo
    echo "  El movil dira 'sin acceso a internet': es normal, la Pi no tiene"
    echo "  salida a internet mientras hace de AP. Acepta mantener la conexion."
    ;;

  off)
    need_root
    nmcli con down "$CON" || true
    echo "AP apagado. La Pi vuelve a conectarse a su wifi habitual."
    ;;

  auto)
    # Que el AP se levante solo al encender la Pi (comodo en pruebas).
    need_root
    nmcli con modify "$CON" connection.autoconnect yes
    echo "El AP se levantara solo en cada arranque."
    echo "Para desactivarlo: sudo $0 noauto"
    ;;

  noauto)
    need_root
    nmcli con modify "$CON" connection.autoconnect no
    echo "El AP ya no se levanta solo."
    ;;

  competencia)
    need_root
    nmcli con modify "$CON" connection.autoconnect no || true
    nmcli con down "$CON" >/dev/null 2>&1 || true
    rfkill block wifi
    rfkill block bluetooth || true
    echo "Wifi y bluetooth BLOQUEADOS (regla 11.10)."
    echo "Arranca el robot con:  cd rpi && python3 run.py --no-web"
    echo "Para volver a pruebas: sudo $0 on"
    ;;

  estado)
    echo "--- rfkill ---";        rfkill list || true
    echo "--- conexiones ---";    nmcli con show --active || true
    echo "--- direcciones ---";   ip -brief addr show "$IFACE" || true
    echo "--- clientes DHCP ---"
    ip neigh show dev "$IFACE" 2>/dev/null | grep -v FAILED || echo "  (ninguno)"
    ;;

  *)
    sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
