#!/usr/bin/env bash
# cargar_firmware.sh — Instala Arduino CLI y el core ESP32 de Espressif,
# descarga el firmware de la plataforma ThermoLab desde GitHub, lo compila,
# detecta el puerto donde está conectada la placa y lo graba con esptool
# (binario independiente, sin depender de Python).
#
# Uso:
#   ./cargar_firmware.sh
#
# Requiere: bash, curl, git, tar, udevadm (paquete udev, presente en la
# mayoría de las distribuciones Linux).

set -euo pipefail

REPO_URL="https://github.com/nebisman/ThermoLab.jl.git"
FQBN="esp32:esp32:esp32s3usbotg"
CHIP="esp32s3"
ESP32_BOARD_URL="https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"
SKETCH_NAME="esp32_ThermoLab"
ESPTOOL_VERSION="5.3.1"
WORKDIR="$HOME/.local/share/thermolab-firmware"
BINDIR="$HOME/.local/bin"

mkdir -p "$WORKDIR" "$BINDIR"

log() { echo "==> $1"; }
die() { echo "ERROR: $1" >&2; exit 1; }

[[ "$(uname -s)" == "Linux" ]] || die "Este instalador solo es compatible con Linux."

for dep in curl git tar udevadm; do
    command -v "$dep" &>/dev/null || die "Se requiere '$dep', pero no está instalado."
done

# ── 1. Instalar Arduino CLI ──────────────────────────────────────────────
if command -v arduino-cli &>/dev/null; then
    log "Arduino CLI ya está instalado: $(arduino-cli version | head -1)"
else
    log "Arduino CLI no está instalado. Iniciando instalación..."
    mkdir -p "$BINDIR"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$BINDIR" sh
    export PATH="$BINDIR:$PATH"
    command -v arduino-cli &>/dev/null || die "No se pudo instalar Arduino CLI."
    log "Arduino CLI instalado correctamente: $(arduino-cli version | head -1)"
fi

# ── 2. Instalar el core (librerías) ESP32 de Espressif ──────────────────
if arduino-cli core list 2>/dev/null | grep -q "^esp32:esp32"; then
    log "Las librerías de ESP32 (core esp32:esp32) ya están instaladas."
else
    log "Las librerías de ESP32 (core esp32:esp32) no están instaladas. Iniciando instalación..."
    arduino-cli core update-index --additional-urls "$ESP32_BOARD_URL"
    arduino-cli core install esp32:esp32 --additional-urls "$ESP32_BOARD_URL"
    log "Librerías de ESP32 instaladas correctamente."
fi

# ── 3. Instalar esptool (binario independiente, sin Python) ─────────────
# El firmware se graba con el binario autocontenido de esptool en lugar
# del esptool.py que trae el core ESP32 (el cual depende de Python y del
# módulo 'pyserial', problemático de instalar en equipos "frescos" por la
# restricción PEP 668 de pip).
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

# ── 4. Instalar las librerías Arduino requeridas por el firmware ────────
REQUIRED_LIBS=("ESP32Encoder" "ArduinoJson" "Adafruit NeoPixel" "OneWire" "DallasTemperature")
log "Instalando las librerías Arduino requeridas por el firmware..."
arduino-cli lib install "${REQUIRED_LIBS[@]}"
log "Librerías Arduino instaladas correctamente."

# ── 5. Descargar el firmware desde el repositorio de GitHub ─────────────
log "Descargando firmware/$SKETCH_NAME desde $REPO_URL..."
TMP_CLONE="$(mktemp -d)"
trap 'rm -rf "$TMP_CLONE"' EXIT

git clone --quiet --depth 1 --filter=blob:none --sparse "$REPO_URL" "$TMP_CLONE"
git -C "$TMP_CLONE" sparse-checkout set "firmware/$SKETCH_NAME"

[[ -d "$TMP_CLONE/firmware/$SKETCH_NAME" ]] ||
    die "No se encontró firmware/$SKETCH_NAME en el repositorio."

rm -rf "$WORKDIR/$SKETCH_NAME"
cp -r "$TMP_CLONE/firmware/$SKETCH_NAME" "$WORKDIR/$SKETCH_NAME"
log "Firmware descargado en $WORKDIR/$SKETCH_NAME"

# ── 6. Compilar el firmware ───────────────────────────────────────────────
log "Compilando el firmware ($FQBN)..."
BUILD_DIR="$WORKDIR/build"
arduino-cli compile --verbose --fqbn "$FQBN" --export-binaries --output-dir "$BUILD_DIR" "$WORKDIR/$SKETCH_NAME"
MERGED_BIN="$BUILD_DIR/$SKETCH_NAME.ino.merged.bin"
[[ -s "$MERGED_BIN" ]] || die "No se generó el binario fusionado esperado en $MERGED_BIN."

# ── 7. Detectar el puerto de la placa ─────────────────────────────────────
log "Buscando el puerto de la placa ESP32..."

# Se identifica la placa por su VID:PID (o, en su defecto, por palabras
# clave del fabricante) entre los dispositivos serie del sistema, sin
# depender de arduino-cli ni de Python para esta tarea.
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

# ── 8. Grabar el firmware en la placa ─────────────────────────────────────
log "Grabando el firmware en el puerto $PORT..."
esptool --chip "$CHIP" --port "$PORT" write-flash 0x0 "$MERGED_BIN"

log "Firmware cargado exitosamente."




