#!/usr/bin/env bash
# =============================================================================
#  esp32_diag.sh — Diagnóstico y corrección sistemática de ESP32 en Linux
#
#  Uso:
#     ./esp32_diag.sh          # solo diagnóstico (no modifica nada)
#     ./esp32_diag.sh --fix    # diagnóstico + ofrece aplicar correcciones
#
#  Pruebas:
#     T01 Sistema y kernel
#     T02 Detección USB (lsusb, con prueba desconectar/conectar)
#     T03 Identificación del chip USB-serial
#     T04 Módulo del kernel (existe / cargado)
#     T05 Mensajes del kernel (cable, energía, brltty)
#     T06 Nodo de dispositivo /dev/tty*
#     T07 Conflicto con brltty
#     T08 Conflicto con ModemManager
#     T09 Permisos (grupo dialout/uucp, permisos del nodo)
#     T10 Puerto ocupado por otro proceso
#     T11 IDE en Snap / Flatpak
#     T12 Comunicación real con esptool
# =============================================================================

set -u

FIX=0
[[ "${1:-}" == "--fix" ]] && FIX=1

# ---------- Presentación ----------------------------------------------------
if [[ -t 1 ]]; then
  R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[34m'; W=$'\e[1m'; N=$'\e[0m'
else
  R=""; G=""; Y=""; B=""; W=""; N=""
fi

ok()    { echo "  ${G}[OK]${N}    $*"; }
warn()  { echo "  ${Y}[AVISO]${N} $*"; }
fail()  { echo "  ${R}[FALLA]${N} $*"; }
info()  { echo "  ${B}[INFO]${N}  $*"; }
titulo(){ echo; echo "${W}== $* ==${N}"; }

PROBLEMAS=()      # descripción de cada problema encontrado
ARREGLOS=()       # comando de corrección asociado (mismo índice)

registrar() {     # registrar "descripción" "comando de corrección"
  PROBLEMAS+=("$1")
  ARREGLOS+=("$2")
}

tiene() { command -v "$1" >/dev/null 2>&1; }

confirmar() {     # confirmar "pregunta" -> 0 si sí
  local r
  read -r -p "  ¿$1? [s/N] " r
  [[ "$r" =~ ^[sSyY]$ ]]
}

# Lectura de mensajes del kernel (dmesg puede estar restringido)
leer_kernel() {
  if dmesg >/dev/null 2>&1; then
    dmesg 2>/dev/null | tail -n 300
  elif tiene journalctl; then
    journalctl -k -n 300 --no-pager 2>/dev/null
  else
    sudo dmesg 2>/dev/null | tail -n 300
  fi
}

# Tabla de chips conocidos:  VID:PID | nombre | módulo | patrón de nodo
CHIPS=(
  "10c4:ea60|Silicon Labs CP210x|cp210x|ttyUSB"
  "10c4:ea70|Silicon Labs CP2105|cp210x|ttyUSB"
  "1a86:7523|WCH CH340|ch341|ttyUSB"
  "1a86:5523|WCH CH341|ch341|ttyUSB"
  "1a86:55d4|WCH CH9102/CH343|cdc_acm|ttyACM"
  "1a86:55d3|WCH CH343|cdc_acm|ttyACM"
  "0403:6001|FTDI FT232R|ftdi_sio|ttyUSB"
  "0403:6010|FTDI FT2232|ftdi_sio|ttyUSB"
  "0403:6015|FTDI FT231X|ftdi_sio|ttyUSB"
  "303a:*|Espressif USB nativo (S2/S3/C3/C6)|cdc_acm|ttyACM"
)

CHIP_ID=""; CHIP_NOMBRE=""; CHIP_MOD=""; CHIP_NODO=""; PUERTO=""
USUARIO="${USER:-$(id -un)}"

echo "${W}Diagnóstico ESP32 en Linux${N}  —  $(date '+%Y-%m-%d %H:%M')"
[[ $FIX -eq 1 ]] && echo "Modo: diagnóstico + corrección" || echo "Modo: solo diagnóstico (use --fix para corregir)"

# ---------- T01 Sistema ------------------------------------------------------
titulo "T01  Sistema y kernel"
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  info "Distribución: ${PRETTY_NAME:-desconocida}"
  DISTRO_ID="${ID:-}"; DISTRO_LIKE="${ID_LIKE:-}"
else
  DISTRO_ID=""; DISTRO_LIKE=""
fi
KERNEL="$(uname -r)"
info "Kernel: $KERNEL"
case "$KERNEL" in
  *-kvm|*-aws|*-azure|*-gcp|*-oracle|*-cloud*)
    warn "Kernel reducido (cloud): puede no incluir módulos USB-serial." ;;
esac
tiene lsusb || { fail "Falta 'lsusb' (paquete usbutils)."; registrar "Instalar usbutils" "sudo apt install -y usbutils || sudo dnf install -y usbutils || sudo pacman -S --noconfirm usbutils"; }

# ---------- T02 Detección USB -----------------------------------------------
titulo "T02  Detección USB"
detectar_chip() {
  local linea vid pid entrada patron
  CHIP_ID=""
  while read -r linea; do
    vid="${linea%%:*}"; pid="${linea##*:}"
    for entrada in "${CHIPS[@]}"; do
      patron="${entrada%%|*}"
      if [[ "$vid:$pid" == $patron ]]; then
        CHIP_ID="$vid:$pid"
        IFS='|' read -r _ CHIP_NOMBRE CHIP_MOD CHIP_NODO <<<"$entrada"
        return 0
      fi
    done
  done < <(lsusb 2>/dev/null | grep -oE 'ID [0-9a-f]{4}:[0-9a-f]{4}' | awk '{print $2}')
  return 1
}

if tiene lsusb && detectar_chip; then
  ok "Encontrado $CHIP_NOMBRE ($CHIP_ID)"
else
  warn "No se reconoce ningún chip USB-serial conocido."
  if tiene lsusb && [[ -t 0 ]]; then
    echo "  Prueba de conexión: DESCONECTE el ESP32 y presione Enter."
    read -r _; ANTES="$(lsusb)"
    echo "  Ahora CONÉCTELO, espere 3 s y presione Enter."
    read -r _; sleep 1; DESPUES="$(lsusb)"
    NUEVO="$(diff <(echo "$ANTES") <(echo "$DESPUES") | grep '^>' | sed 's/^> //')"
    if [[ -z "$NUEVO" ]]; then
      fail "El sistema NO ve ningún dispositivo nuevo al conectar."
      info "Causas: cable de solo carga, puerto USB, hub sin energía o placa dañada."
      registrar "Hardware: probar otro cable de DATOS, otro puerto, sin hub" ""
    else
      info "Dispositivo nuevo: $NUEVO"
      detectar_chip && ok "Ahora reconocido: $CHIP_NOMBRE ($CHIP_ID)" \
                    || warn "Chip no listado; revise el VID:PID manualmente."
    fi
  fi
fi

# ---------- T03 Identificación del chip -------------------------------------
titulo "T03  Identificación del chip"
if [[ -n "$CHIP_ID" ]]; then
  info "Chip: $CHIP_NOMBRE  |  módulo esperado: $CHIP_MOD  |  nodo: /dev/${CHIP_NODO}*"
  [[ "$CHIP_ID" == 1a86:55d4 ]] && info "CH9102: en kernels antiguos puede requerir el driver ch343 de WCH."
  [[ "$CHIP_ID" == 303a:* ]] && info "USB nativo: si no aparece, entre a bootloader (BOOT + RESET)."
else
  info "Sin chip identificado; se omiten pruebas específicas del módulo."
fi

# ---------- T04 Módulo del kernel -------------------------------------------
titulo "T04  Módulo del kernel"
modulo_estado() {   # imprime: cargado | integrado | disponible | ausente
  local m="$1"
  if [[ -d /sys/module/$m ]]; then
    if grep -qw "$m" /proc/modules 2>/dev/null; then echo cargado; else echo integrado; fi
  elif grep -qE "/$m\.ko" "/lib/modules/$KERNEL/modules.builtin" 2>/dev/null; then
    echo integrado
  elif modinfo "$m" >/dev/null 2>&1 || \
       find "/lib/modules/$KERNEL" -name "$m.ko*" 2>/dev/null | grep -q .; then
    echo disponible
  else
    echo ausente
  fi
}
for m in ${CHIP_MOD:-cp210x ch341 ftdi_sio cdc_acm}; do
  case "$(modulo_estado "$m")" in
    cargado)    ok "Módulo $m cargado." ;;
    integrado)  ok "Driver $m integrado en el kernel." ;;
    disponible)
      if [[ -n "$CHIP_MOD" ]]; then
        warn "Módulo $m disponible pero NO cargado."
        registrar "Cargar el módulo $m" "sudo modprobe $m"
      else
        info "Módulo $m disponible (se carga al conectar la placa)."
      fi ;;
    ausente)
      fail "Módulo $m NO existe en el kernel $KERNEL."
      if [[ "$DISTRO_ID $DISTRO_LIKE" == *ubuntu* || "$DISTRO_ID $DISTRO_LIKE" == *debian* ]]; then
        [[ " ${PROBLEMAS[*]-} " == *"Instalar módulos extra"* ]] || \
          registrar "Instalar módulos extra del kernel" "sudo apt install -y linux-modules-extra-$KERNEL && sudo modprobe ${CHIP_MOD:-cp210x}"
      else
        registrar "Instalar el paquete de módulos del kernel que incluya $m" ""
      fi ;;
  esac
done

# ---------- T05 Mensajes del kernel -----------------------------------------
titulo "T05  Mensajes del kernel"
KLOG="$(leer_kernel)"
if [[ -z "$KLOG" ]]; then
  warn "No se pudieron leer los mensajes del kernel."
else
  if grep -qiE 'brltty' <<<"$KLOG"; then
    fail "brltty aparece en los mensajes del kernel (captura el puerto)."
  fi
  if grep -qiE 'error -71|error -110|unable to enumerate|device descriptor read' <<<"$KLOG"; then
    fail "Errores de enumeración USB: típico de cable malo, falta de energía o hub."
    registrar "Hardware: cambiar cable/puerto, conectar sin hub" ""
  fi
  if grep -qiE 'over-current' <<<"$KLOG"; then
    fail "Sobrecorriente en el puerto USB."
  fi
  ULT="$(grep -iE 'ttyUSB|ttyACM|cp210x|ch341|ch343|ftdi|cdc_acm' <<<"$KLOG" | tail -n 5)"
  if [[ -n "$ULT" ]]; then
    info "Últimos eventos relevantes:"
    sed 's/^/          /' <<<"$ULT"
    grep -qiE 'now disconnected' <<<"$(tail -n 1 <<<"$ULT")" && \
      warn "El último evento es una desconexión del puerto."
  else
    info "Sin eventos de puertos serie recientes."
  fi
fi

# ---------- T06 Nodo de dispositivo -----------------------------------------
titulo "T06  Nodo de dispositivo"
NODOS=( /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB* )
EXISTEN=()
for n in "${NODOS[@]}"; do [[ -e "$n" ]] && EXISTEN+=("$n"); done
if [[ ${#EXISTEN[@]} -gt 0 ]]; then
  for n in "${EXISTEN[@]}"; do ok "Existe $n  ($(stat -c '%U:%G %a' "$n"))"; done
  # Elegir el que coincide con el chip, si se puede
  for n in "${EXISTEN[@]}"; do
    [[ -n "$CHIP_NODO" && "$n" == *"$CHIP_NODO"* ]] && { PUERTO="$n"; break; }
  done
  PUERTO="${PUERTO:-${EXISTEN[0]}}"
  info "Puerto a probar: $PUERTO"
  [[ -d /dev/serial/by-id ]] && info "Nombres estables:" && ls -1 /dev/serial/by-id | sed 's/^/          /'
else
  fail "No existe /dev/ttyUSB* ni /dev/ttyACM*."
fi

# ---------- T07 brltty -------------------------------------------------------
titulo "T07  Conflicto con brltty"
if tiene brltty || dpkg -s brltty >/dev/null 2>&1 || rpm -q brltty >/dev/null 2>&1; then
  if systemctl is-active --quiet brltty-udev.service 2>/dev/null || pgrep -x brltty >/dev/null; then
    fail "brltty está instalado y activo."
  else
    warn "brltty está instalado (puede activarse al conectar)."
  fi
  registrar "Eliminar brltty (si no usa lector braille)" \
    "sudo systemctl stop brltty-udev.service brltty.service 2>/dev/null; sudo systemctl mask brltty-udev.service 2>/dev/null; (sudo apt remove -y brltty || sudo dnf remove -y brltty || sudo pacman -R --noconfirm brltty)"
else
  ok "brltty no está instalado."
fi

# ---------- T08 ModemManager ------------------------------------------------
titulo "T08  Conflicto con ModemManager"
if systemctl is-active --quiet ModemManager 2>/dev/null; then
  warn "ModemManager activo: puede abrir /dev/ttyACM* y bloquear la carga."
  registrar "Detener y deshabilitar ModemManager" "sudo systemctl disable --now ModemManager"
else
  ok "ModemManager inactivo."
fi

# ---------- T09 Permisos -----------------------------------------------------
titulo "T09  Permisos"
GRUPO="dialout"
getent group dialout >/dev/null || GRUPO="uucp"
if [[ $(id -u) -eq 0 ]]; then
  info "Ejecutando como root: ejecute el script con su usuario normal para evaluar el grupo."
elif id -nG "$USUARIO" | grep -qw "$GRUPO"; then
  ok "El usuario $USUARIO pertenece a '$GRUPO'."
elif getent group "$GRUPO" | grep -qw "$USUARIO"; then
  warn "Agregado a '$GRUPO' pero la sesión no lo refleja: cierre sesión o reinicie."
else
  fail "El usuario $USUARIO NO pertenece a '$GRUPO'."
  registrar "Agregar $USUARIO al grupo $GRUPO (requiere cerrar sesión)" "sudo usermod -aG $GRUPO $USUARIO"
fi
if [[ -n "$PUERTO" ]]; then
  if [[ -r "$PUERTO" && -w "$PUERTO" ]]; then
    ok "Lectura/escritura permitida en $PUERTO."
  else
    fail "Sin permiso de lectura/escritura en $PUERTO."
  fi
fi
REGLA=/etc/udev/rules.d/99-esp32.rules
if [[ -f "$REGLA" ]]; then
  ok "Existe la regla udev $REGLA."
else
  info "No hay regla udev para ESP32 (opcional, recomendada)."
  registrar "Crear regla udev (permisos, ignorar ModemManager, enlace /dev/esp32)" "CREAR_REGLA_UDEV"
fi

# ---------- T10 Puerto ocupado ----------------------------------------------
titulo "T10  Puerto ocupado"
if [[ -n "$PUERTO" ]]; then
  OCUP=""
  tiene fuser && OCUP="$(fuser "$PUERTO" 2>/dev/null)"
  [[ -z "$OCUP" ]] && tiene lsof && OCUP="$(lsof -t "$PUERTO" 2>/dev/null)"
  if [[ -n "${OCUP// /}" ]]; then
    fail "$PUERTO está en uso por PID(s):${OCUP}"
    for p in $OCUP; do info "  $p: $(ps -o comm= -p "$p" 2>/dev/null)"; done
    registrar "Liberar $PUERTO (termina los procesos que lo usan)" "sudo fuser -k $PUERTO"
  else
    ok "$PUERTO libre."
  fi
else
  info "Sin puerto que revisar."
fi




# ---------- Resumen y corrección --------------------------------------------
titulo "Resumen"
if [[ ${#PROBLEMAS[@]} -eq 0 ]]; then
  ok "No se encontraron problemas."
  exit 0
fi
for i in "${!PROBLEMAS[@]}"; do
  echo "  $((i+1)). ${PROBLEMAS[$i]}"
  [[ -n "${ARREGLOS[$i]}" && "${ARREGLOS[$i]}" != CREAR_REGLA_UDEV ]] && echo "       → ${ARREGLOS[$i]}"
done

[[ $FIX -eq 0 ]] && { echo; info "Ejecute con --fix para aplicar las correcciones."; exit 1; }

crear_regla_udev() {
  sudo tee "$REGLA" >/dev/null <<'EOF'
# Reglas ESP32: permisos, excluir de ModemManager, enlace /dev/esp32*
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="esp32-%n"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="esp32-%n"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="esp32-%n"
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="esp32-%n"
EOF
  sudo udevadm control --reload-rules && sudo udevadm trigger
}

titulo "Corrección"
RELOGIN=0
for i in "${!PROBLEMAS[@]}"; do
  cmd="${ARREGLOS[$i]}"
  [[ -z "$cmd" ]] && { warn "Manual: ${PROBLEMAS[$i]}"; continue; }
  if confirmar "Aplicar: ${PROBLEMAS[$i]}"; then
    if [[ "$cmd" == CREAR_REGLA_UDEV ]]; then
      crear_regla_udev && ok "Regla creada en $REGLA" || fail "No se pudo crear la regla."
    else
      bash -c "$cmd" && ok "Aplicado." || fail "Falló: $cmd"
    fi
    [[ "$cmd" == *usermod* ]] && RELOGIN=1
  fi
done

echo
info "Desconecte y vuelva a conectar el ESP32, y ejecute de nuevo este script para verificar."
[[ $RELOGIN -eq 1 ]] && warn "Cierre sesión (o reinicie) para que el cambio de grupo tenga efecto."
