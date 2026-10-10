<#
.SYNOPSIS
    cargar_firmware.ps1 - Instala Arduino CLI y el core ESP32 de Espressif,
    descarga el firmware de la plataforma ThermoLab desde GitHub, lo compila,
    detecta el puerto donde esta conectada la placa y lo graba con esptool.

.DESCRIPTION
    Equivalente en Windows de cargar_firmware.sh. Compila el firmware a
    partir de su codigo fuente y es la opcion indicada para quienes van a
    modificarlo (a diferencia de flashear_firmware.ps1, que solo graba el
    binario ya compilado).

.NOTES
    Requiere PowerShell 5.1 (incluido en Windows 10/11) o PowerShell 7 (pwsh),
    y Git para Windows (https://git-scm.com/download/win).
    Si la ejecucion de scripts esta restringida, ejecute antes:
        Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass

.EXAMPLE
    .\cargar_firmware.ps1
#>

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"  # descargas mas rapidas con Invoke-WebRequest

function Log($msg) { Write-Host "==> $msg" }
function Die($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }
function Assert-LastExit($description) {
    if ($LASTEXITCODE -ne 0) {
        Die "$description fallo (codigo $LASTEXITCODE)."
    }
}

if ([System.Environment]::OSVersion.Platform -ne "Win32NT") {
    Die "Este instalador solo es compatible con Windows."
}

foreach ($dep in @("git")) {
    if (-not (Get-Command $dep -ErrorAction SilentlyContinue)) {
        Die "Se requiere '$dep', pero no esta instalado. Descarguelo de https://git-scm.com/download/win."
    }
}

$RepoUrl        = "https://github.com/nebisman/ThermoLab.jl.git"
$Fqbn           = "esp32:esp32:esp32s3usbotg"
$Chip           = "esp32s3"
$Esp32BoardUrl  = "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"
$SketchName     = "esp32_ThermoLab"
$EsptoolVersion = "5.3.1"
$WorkDir        = Join-Path $env:LOCALAPPDATA "thermolab-firmware"
$BinDir         = Join-Path $WorkDir "bin"
$ArduinoCliExe  = Join-Path $BinDir "arduino-cli.exe"
$EsptoolExe     = Join-Path $BinDir "esptool.exe"

New-Item -ItemType Directory -Force -Path $WorkDir, $BinDir | Out-Null
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
$env:PATH = "$BinDir;$env:PATH"

# ── 1. Instalar Arduino CLI ───────────────────────────────────────────────
$systemArduinoCli = Get-Command arduino-cli -ErrorAction SilentlyContinue
if ($systemArduinoCli) {
    $ArduinoCli = "arduino-cli"
    Log "Arduino CLI ya esta instalado: $((& $ArduinoCli version) -join ' ')"
} else {
    $ArduinoCli = $ArduinoCliExe
    if (Test-Path $ArduinoCliExe) {
        Log "Arduino CLI ya esta instalado en $ArduinoCliExe."
    } else {
        Log "Arduino CLI no esta instalado. Descargando la ultima version..."
        $release = Invoke-RestMethod -Uri "https://api.github.com/repos/arduino/arduino-cli/releases/latest" -Headers @{ "User-Agent" = "thermolab-installer" }
        $asset = $release.assets | Where-Object { $_.name -match "_Windows_64bit\.zip$" } | Select-Object -First 1
        if (-not $asset) {
            Die "No se encontro el binario de Arduino CLI para Windows en la ultima release."
        }
        $tmpDir = Join-Path ([System.IO.Path]::GetTempPath()) ([System.IO.Path]::GetRandomFileName())
        New-Item -ItemType Directory -Force -Path $tmpDir | Out-Null
        $zipPath = Join-Path $tmpDir $asset.name
        try {
            Invoke-WebRequest -Uri $asset.browser_download_url -OutFile $zipPath
        } catch {
            Remove-Item -Recurse -Force $tmpDir -ErrorAction SilentlyContinue
            Die "No se pudo descargar Arduino CLI desde $($asset.browser_download_url)."
        }
        Expand-Archive -Path $zipPath -DestinationPath $tmpDir -Force
        Copy-Item -Path (Join-Path $tmpDir "arduino-cli.exe") -Destination $ArduinoCliExe -Force
        Remove-Item -Recurse -Force $tmpDir
        Log "Arduino CLI instalado correctamente en $ArduinoCliExe."
    }
}

# ── 2. Instalar el core (librerias) ESP32 de Espressif ────────────────────
$coreList = & $ArduinoCli core list 2>$null
if ($coreList -match "^esp32:esp32") {
    Log "Las librerias de ESP32 (core esp32:esp32) ya estan instaladas."
} else {
    Log "Las librerias de ESP32 (core esp32:esp32) no estan instaladas. Iniciando instalacion..."
    & $ArduinoCli core update-index --additional-urls $Esp32BoardUrl
    Assert-LastExit "La actualizacion del indice de cores"
    & $ArduinoCli core install esp32:esp32 --additional-urls $Esp32BoardUrl
    Assert-LastExit "La instalacion del core esp32:esp32"
    Log "Librerias de ESP32 instaladas correctamente."
}

# ── 3. Instalar esptool (binario independiente, sin Python) ──────────────
$needsEsptoolInstall = $true
if (Test-Path $EsptoolExe) {
    try {
        $verOutput = & $EsptoolExe version 2>$null
        if ($verOutput -match [regex]::Escape($EsptoolVersion)) {
            Log "esptool $EsptoolVersion ya esta instalado en $EsptoolExe."
            $needsEsptoolInstall = $false
        }
    } catch { }
}

if ($needsEsptoolInstall) {
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

# ── 4. Instalar las librerias Arduino requeridas por el firmware ─────────
$RequiredLibs = @("ESP32Encoder", "ArduinoJson", "Adafruit NeoPixel", "OneWire", "DallasTemperature")
Log "Instalando las librerias Arduino requeridas por el firmware..."
& $ArduinoCli lib install @RequiredLibs
Assert-LastExit "La instalacion de librerias Arduino"
Log "Librerias Arduino instaladas correctamente."

# ── 5. Descargar el firmware desde el repositorio de GitHub ──────────────
Log "Descargando firmware/$SketchName desde $RepoUrl..."
$tmpClone = Join-Path ([System.IO.Path]::GetTempPath()) ([System.IO.Path]::GetRandomFileName())

& git clone --quiet --depth 1 --filter=blob:none --sparse $RepoUrl $tmpClone
if ($LASTEXITCODE -ne 0) {
    Remove-Item -Recurse -Force $tmpClone -ErrorAction SilentlyContinue
    Die "No se pudo clonar el repositorio $RepoUrl."
}
& git -C $tmpClone sparse-checkout set "firmware/$SketchName"
if ($LASTEXITCODE -ne 0) {
    Remove-Item -Recurse -Force $tmpClone -ErrorAction SilentlyContinue
    Die "No se pudo configurar el sparse-checkout de firmware/$SketchName."
}

$sketchSrc = Join-Path $tmpClone "firmware\$SketchName"
if (-not (Test-Path $sketchSrc)) {
    Remove-Item -Recurse -Force $tmpClone -ErrorAction SilentlyContinue
    Die "No se encontro firmware/$SketchName en el repositorio."
}

$sketchDst = Join-Path $WorkDir $SketchName
if (Test-Path $sketchDst) { Remove-Item -Recurse -Force $sketchDst }
Copy-Item -Path $sketchSrc -Destination $sketchDst -Recurse -Force
Remove-Item -Recurse -Force $tmpClone
Log "Firmware descargado en $sketchDst"

# ── 6. Compilar el firmware ────────────────────────────────────────────────
Log "Compilando el firmware ($Fqbn)..."
$BuildDir = Join-Path $WorkDir "build"
& $ArduinoCli compile --verbose --fqbn $Fqbn --export-binaries --output-dir $BuildDir $sketchDst
Assert-LastExit "La compilacion del firmware"
$MergedBin = Join-Path $BuildDir "$SketchName.ino.merged.bin"
if (-not (Test-Path $MergedBin) -or (Get-Item $MergedBin).Length -eq 0) {
    Die "No se genero el binario fusionado esperado en $MergedBin."
}

# ── 7. Detectar el puerto de la placa ──────────────────────────────────────
Log "Buscando el puerto de la placa ESP32..."

# Mismo criterio de deteccion (VID:PID y palabras clave) que
# cargar_firmware.sh, pero usando Win32_PnPEntity en lugar de udevadm.
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

# ── 8. Grabar el firmware en la placa ──────────────────────────────────────
Log "Grabando el firmware en el puerto $Port..."
& $EsptoolExe --chip $Chip --port $Port write-flash --erase-all 0x0 $MergedBin
Assert-LastExit "esptool"

Log "Firmware cargado exitosamente."

