#!/usr/bin/env bash
# flashear_firmware.sh — Instala esptool (binario independiente, sin
# depender de Python/pip) y escribe en la placa el firmware ya
# compilado de la plataforma ThermoLab. A diferencia de cargar_firmware.sh,
# este script NO instala Arduino CLI ni el core ESP32: solo descarga el
# binario ya compilado del repositorio y lo graba con esptool. Es la vía
# más rápida y liviana para usuarios que no necesitan modificar el
# código del firmware.
#
# Uso:
#   ./flashear_firmware.sh
#
# Requiere: bash, curl, tar, udevadm (paquete udev, presente en la
# mayoría de las distribuciones Linux).

set -euo pipefail

REPO_RAW_URL="https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main"
FW_BIN_URL="$REPO_RAW_URL/firmware/instalador/bin/esp32_ThermoLab.bin"
CHIP="esp32s3"
ESPTOOL_VERSION="5.3.1"
WORKDIR="$HOME/.local/share/thermolab-firmware"
BINDIR="$HOME/.local/bin"

mkdir -p "$WORKDIR" "$BINDIR"

log() { echo "==> $1"; }
die() { echo "ERROR: $1" >&2; exit 1; }

[[ "$(uname -s)" == "Linux" ]] || die "Este instalador solo es compatible con Linux."

for dep in curl tar udevadm; do
    command -v "$dep" &>/dev/null || die "Se requiere '$dep', pero no está instalado."
done

# ── 1. Instalar esptool (binario independiente, sin Python) ─────────────
case "$(uname -m)" in
    x86_64)          ESPTOOL_ARCH="amd64" ;;
    aarch64|arm64)   ESPTOOL_ARCH="aarch64" ;;
    armv7l)          ESPTOOL_ARCH="armv7" ;;
    *) die "Arquitectura '$(uname -m)' no soportada por los binarios de esptool." ;;
esac

if command -v esptool &>/dev/null && esptool version 2>/dev/null | grep -q "$ESPTOOL_VERSION"; then
    log "esptool $ESPTOOL_VERSION ya está instalado: $(command -v esptool)"
elif [[ -x "$BINDIR/esptool" ]] && "$BINDIR/esptool" version 2>/dev/null | grep -q "$ESPTOOL_VERSION"; then
    log "esptool $ESPTOOL_VERSION ya está instalado en $BINDIR."
else
    log "Descargando esptool $ESPTOOL_VERSION (linux-$ESPTOOL_ARCH)..."
    ESPTOOL_ASSET="esptool-v$ESPTOOL_VERSION-linux-$ESPTOOL_ARCH.tar.gz"
    ESPTOOL_URL="https://github.com/espressif/esptool/releases/download/v$ESPTOOL_VERSION/$ESPTOOL_ASSET"
    TMP_ESPTOOL="$(mktemp -d)"
    trap 'rm -rf "$TMP_ESPTOOL"' EXIT
    curl -fsSL -o "$TMP_ESPTOOL/esptool.tar.gz" "$ESPTOOL_URL" ||
        die "No se pudo descargar esptool desde $ESPTOOL_URL."
    tar -xzf "$TMP_ESPTOOL/esptool.tar.gz" -C "$TMP_ESPTOOL"
    install -m 755 "$TMP_ESPTOOL/esptool-linux-$ESPTOOL_ARCH/esptool" "$BINDIR/esptool"
    rm -rf "$TMP_ESPTOOL"
    trap - EXIT
    log "esptool instalado correctamente en $BINDIR/esptool."
fi
export PATH="$BINDIR:$PATH"

# ── 2. Descargar el firmware precompilado desde el repositorio ──────────
log "Descargando el firmware precompilado..."
FW_BIN="$WORKDIR/esp32_ThermoLab.bin"
curl -fsSL -o "$FW_BIN" "$FW_BIN_URL" ||
    die "No se pudo descargar el firmware desde $FW_BIN_URL."
[[ -s "$FW_BIN" ]] || die "El firmware descargado está vacío."
log "Firmware descargado en $FW_BIN"

# ── 3. Detectar el puerto de la placa ─────────────────────────────────────
log "Buscando el puerto de la placa ESP32..."

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

# ── 4. Grabar el firmware en la placa ─────────────────────────────────────
log "Grabando el firmware en el puerto $PORT..."
esptool --chip "$CHIP" --port "$PORT" write-flash  0x0 "$FW_BIN"

log "Firmware cargado exitosamente."
