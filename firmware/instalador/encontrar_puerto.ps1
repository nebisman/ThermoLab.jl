<#
.SYNOPSIS
    encontrar_puerto.ps1 - Detecta el puerto COM donde esta conectada la
    placa ESP32 de la plataforma ThermoLab, sin instalar ni grabar nada.

.DESCRIPTION
    Equivalente en Windows de encontrar_puerto.sh. Util para diagnosticar
    problemas de deteccion antes de usar flashear_firmware.ps1 o
    cargar_firmware.ps1.

.NOTES
    Requiere PowerShell 5.1 (incluido en Windows 10/11) o PowerShell 7 (pwsh).
    Si la ejecucion de scripts esta restringida, ejecute antes:
        Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass

.EXAMPLE
    .\encontrar_puerto.ps1
#>

$ErrorActionPreference = "Stop"

function Log($msg) { Write-Host "==> $msg" }
function Die($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

if ([System.Environment]::OSVersion.Platform -ne "Win32NT") {
    Die "Este script solo es compatible con Windows."
}

# -- Detectar el puerto de la placa ---------------------------------------
Log "Buscando el puerto de la plataforma ThermoLab..."

# Mismo criterio de deteccion (VID:PID y palabras clave) que
# encontrar_puerto.sh, pero usando Win32_PnPEntity en lugar de udevadm.
$KnownIds = @(
    "VID_10C4&PID_EA60",
    "VID_1A86&PID_7523",
    "VID_1A86&PID_55D4",
    "VID_0403&PID_6001",
    "VID_303A&PID_1001"
)
$Keywords = "CP210|CH340|CH9102|WCH|QinHeng|Silicon Labs|Espressif|ESP32|USB-SERIAL|USB Serial|UART"

$Candidates = @()
Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match "\(COM\d+\)" } | ForEach-Object {
    $isKnown = $false
    foreach ($id in $KnownIds) {
        if ($_.PNPDeviceID -match [regex]::Escape($id)) { $isKnown = $true; break }
    }
    if (-not $isKnown -and $_.Name -match $Keywords) { $isKnown = $true }
    if ($isKnown -and ($_.Name -match "\((COM\d+)\)")) {
        $Candidates += $Matches[1]
    }
}

if ($Candidates.Count -eq 0) {
    Die "No se pudo detectar ningun puerto COM compatible con la placa ESP32. Verifique que la plataforma este conectada y encendida."
} elseif ($Candidates.Count -gt 1) {
    Die "Se detectaron varios puertos candidatos: $($Candidates -join ', '). Edite este script para indicar manualmente cual usar."
}
$Port = $Candidates[0]
Log "Puerto detectado: $Port"
