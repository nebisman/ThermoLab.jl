#!/usr/bin/env bash
# encontrar_puerto.sh — Detecta el puerto USB-serial donde está conectada
# la placa ESP32 de la plataforma ThermoLab, sin instalar ni grabar nada.
# Util para diagnosticar problemas de detección antes de usar
# flashear_firmware.sh o cargar_firmware.sh.
#
# Uso:
#   ./encontrar_puerto.sh
#
# Requiere: bash, udevadm (paquete udev, presente en la mayoría de las
# distribuciones Linux).

set -euo pipefail

log() { echo "==> $1"; }
die() { echo "ERROR: $1" >&2; exit 1; }

command -v udevadm &>/dev/null || die "Se requiere 'udevadm', pero no está instalado."

# ── Detectar el puerto de la placa ────────────────────────────────────────
log "Buscando el puerto de la plataforma ThermoLab..."

# Mismo criterio de detección (VID:PID y palabras clave) que
# cargar_firmware.sh, pero implementado en bash puro con udevadm para no
# depender de arduino-cli ni de Python en este instalador liviano.
KNOWN_VID_PID=("10c4:ea60" "1a86:7523" "1a86:55d4" "0403:6001" "303a:1001")
KEYWORDS="cp210|ch340|ch9102|wch|qinheng|silicon labs|espressif|esp32|usb-serial|usb serial|uart"

is_known_board() {
    local vid="$1" pid="$2" text="$3" pair
    for pair in "${KNOWN_VID_PID[@]}"; do
        [[ "$vid:$pid" == "$pair" ]] && return 0
    done
    [[ "$text" =~ $KEYWORDS ]] && return 0
    return 1
}

PORT_LIST=()
for dev in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$dev" ]] || continue
    props="$(udevadm info -q property -n "$dev" 2>/dev/null)" || continue
    vid="$(grep -oP '(?<=^ID_VENDOR_ID=).*' <<<"$props" | tr '[:upper:]' '[:lower:]')"
    pid="$(grep -oP '(?<=^ID_MODEL_ID=).*' <<<"$props" | tr '[:upper:]' '[:lower:]')"
    text="$(grep -oP '(?<=^ID_VENDOR=).*|(?<=^ID_MODEL=).*|(?<=^ID_SERIAL=).*' <<<"$props" | tr '[:upper:]\n' '[:lower:] ')"
    if is_known_board "$vid" "$pid" "$text"; then
        PORT_LIST+=("$dev")
    fi
done

if [[ ${#PORT_LIST[@]} -eq 0 ]]; then
    die "No se pudo detectar ningún puerto USB-serial compatible con la placa ESP32. Verifique que la plataforma esté conectada y encendida."
elif [[ ${#PORT_LIST[@]} -gt 1 ]]; then
    die "Se detectaron varios puertos candidatos: ${PORT_LIST[*]}. Edite este script para indicar manualmente cuál usar."
fi
PORT="${PORT_LIST[0]}"
log "Puerto detectado: $PORT"
