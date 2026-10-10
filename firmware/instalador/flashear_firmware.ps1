<#
.SYNOPSIS
    flashear_firmware.ps1 - Instala esptool y graba en la placa el firmware
    ya compilado de la plataforma ThermoLab.

.DESCRIPTION
    Equivalente en Windows de flashear_firmware.sh. No compila nada: solo
    descarga esptool (binario independiente, sin depender de Python) y el
    binario ya compilado del firmware desde el repositorio, y lo graba
    directamente en la placa. Es la via mas rapida y liviana para usuarios
    que no necesitan modificar el codigo del firmware.

.NOTES
    Requiere PowerShell 5.1 (incluido en Windows 10/11) o PowerShell 7 (pwsh).
    Si la ejecucion de scripts esta restringida, ejecute antes:
        Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass

.EXAMPLE
    .\flashear_firmware.ps1
#>

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"  # descargas mas rapidas con Invoke-WebRequest

function Log($msg) { Write-Host "==> $msg" }
function Die($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

if ([System.Environment]::OSVersion.Platform -ne "Win32NT") {
    Die "Este instalador solo es compatible con Windows."
}

$RepoRawUrl     = "https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main"
$FwBinUrl       = "$RepoRawUrl/firmware/instalador/bin/esp32_ThermoLab.bin"
$Chip           = "esp32s3"
$EsptoolVersion = "5.3.1"
$WorkDir        = Join-Path $env:LOCALAPPDATA "thermolab-firmware"
$BinDir         = Join-Path $WorkDir "bin"
$EsptoolExe     = Join-Path $BinDir "esptool.exe"

New-Item -ItemType Directory -Force -Path $WorkDir, $BinDir | Out-Null
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

# -- 1. Instalar esptool (binario independiente, sin Python) -------------
$needsInstall = $true
if (Test-Path $EsptoolExe) {
    try {
        $verOutput = & $EsptoolExe version 2>$null
        if ($verOutput -match [regex]::Escape($EsptoolVersion)) {
            Log "esptool $EsptoolVersion ya esta instalado en $EsptoolExe."
            $needsInstall = $false
        }
    } catch { }
}

if ($needsInstall) {
    Log "Descargando esptool $EsptoolVersion (windows-amd64)..."
    $asset  = "esptool-v$EsptoolVersion-windows-amd64.zip"
    $url    = "https://github.com/espressif/esptool/releases/download/v$EsptoolVersion/$asset"
    $tmpDir = Join-Path ([System.IO.Path]::GetTempPath()) ([System.IO.Path]::GetRandomFileName())
    New-Item -ItemType Directory -Force -Path $tmpDir | Out-Null
    $zipPath = Join-Path $tmpDir $asset
    try {
        Invoke-WebRequest -Uri $url -OutFile $zipPath
    } catch {
        Remove-Item -Recurse -Force $tmpDir -ErrorAction SilentlyContinue
        Die "No se pudo descargar esptool desde $url."
    }
    Expand-Archive -Path $zipPath -DestinationPath $tmpDir -Force
    Copy-Item -Path (Join-Path $tmpDir "esptool-windows-amd64\esptool.exe") -Destination $EsptoolExe -Force
    Remove-Item -Recurse -Force $tmpDir
    Log "esptool instalado correctamente en $EsptoolExe."
}

# -- 2. Descargar el firmware precompilado desde el repositorio ----------
Log "Descargando el firmware precompilado..."
$FwBin = Join-Path $WorkDir "esp32_ThermoLab.bin"
try {
    Invoke-WebRequest -Uri $FwBinUrl -OutFile $FwBin
} catch {
    Die "No se pudo descargar el firmware desde $FwBinUrl."
}
if ((Get-Item $FwBin).Length -eq 0) {
    Die "El firmware descargado esta vacio."
}
Log "Firmware descargado en $FwBin"

# -- 3. Detectar el puerto de la placa ------------------------------------
Log "Buscando el puerto de la placa ESP32..."

# Mismo criterio de deteccion (VID:PID y palabras clave) que
# flashear_firmware.sh, pero usando Win32_PnPEntity en lugar de udevadm.
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

# -- 4. Grabar el firmware en la placa ------------------------------------
Log "Grabando el firmware en el puerto $Port..."
& $EsptoolExe --chip $Chip --port $Port write-flash 0x0 $FwBin
if ($LASTEXITCODE -ne 0) {
    Die "esptool termino con errores (codigo $LASTEXITCODE)."
}

Log "Firmware cargado exitosamente."
