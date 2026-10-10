

```@raw html
<div style="font-size: 1em; font-weight: bold; color: rgba(202, 60, 50, 1); background-color: #e9ddaf; border: 4px solid rgba(202, 60, 50, 1); border-radius: 8px; padding: 1em; margin: 1em 0;">
⚠️ La carga de firmware solo es necesaria una vez si el controlador por defecto no funciona. También en el caso en que el firmware de la plataforma <code>ThermoLab</code> deba ser actualizado.
</div>
```
# Instalación del Firmware



## Instalación rápida en Linux (firmware precompilado)

Esta es la forma más simple y rápida de cargar el firmware. Se ejecuta un script que  requiere que `bash`, `curl`, `tar` y `udevadm` estén
disponibles en su instalación de Linux (este último forma parte de
`udev`, presente en la gran mayoría de las distribuciones). Además, el
usuario debe pertenecer al grupo `dialout` para tener permisos sobre el
puerto serial (ver [Permisos del puerto serial en Linux](@ref)). Ejecute los siguientes comandos en la terminal: 

```bash
curl -fsSLO https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/flashear_firmware.sh
chmod +x flashear_firmware.sh
./flashear_firmware.sh
```

Si la placa está conectada por USB, el script detecta el puerto y graba
el firmware automáticamente. Si no se detecta ningún puerto compatible,
o se detecta más de uno, el script se detiene con un mensaje de error
indicando cómo proceder.

## Instalación rápida en Windows (firmware precompilado)

Equivalente en Windows del instalador rápido de Linux: descarga
únicamente `esptool`  y el binario ya
compilado del firmware, y lo graba directamente en la placa. Abra
PowerShell y ejecute:

```powershell
Invoke-WebRequest -Uri https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/flashear_firmware.ps1 -OutFile flashear_firmware.ps1
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\flashear_firmware.ps1
```

Igual que en Linux, si la placa está conectada por USB el script
detecta el puerto (`COMx`) y graba el firmware automáticamente; si no
detecta ningún puerto compatible, o detecta más de uno, se detiene con
un mensaje de error indicando cómo proceder. Si no se detecta ningún
puerto, lo más probable es que falte instalar el driver del chip
USB-serial de la placa (ver [Instalación del driver CH340 en Windows](@ref)).

## Instalación desde el código fuente en Linux

El código fuente del firmware del ESP32 usado por la plataforma ThermoLab se
encuentra en [`firmware/esp32_ThermoLab`](https://github.com/nebisman/ThermoLab.jl/tree/main/firmware/esp32_ThermoLab), como parte del repositorio del paquete.


Esta alternativa compila el firmware a partir del código fuente y es
la indicada para quienes van a modificarlo. Para ello se provee un
script de instalación que instala todas las dependencias necesarias:
Arduino CLI y las librerías de programación del ESP32. Este script
requiere que `bash`, `curl`, `git`, `tar` y `udevadm` estén actualmente en
su instalación de Linux (este último forma parte de `udev`, presente en
la gran mayoría de las distribuciones). Además, el usuario debe
pertenecer al grupo `dialout` para tener permisos sobre el puerto serial
(ver [Permisos del puerto serial en Linux](@ref)).

**Note que el proceso demora porque debe descargarse la libreria completa de programación del ESP32, la cual es muy extensa. Después de instalada por primera vez, el proceso es muy rápido.**


Descargue y ejecute  el script de instalación desde la
terminal con:

```bash
curl -fsSLO https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/cargar_firmware.sh
chmod +x cargar_firmware.sh
./cargar_firmware.sh
```

## Instalación desde el código fuente en Windows

Equivalente en Windows del instalador desde el código fuente de Linux.
Instala Arduino CLI y las librerías de programación del ESP32,
descarga el código fuente del firmware, lo compila y lo graba en la
placa. Requiere PowerShell 5.1 (incluido en Windows 10/11) o
PowerShell 7, y [Git para Windows](https://git-scm.com/download/win).

**Note que el proceso demora porque debe descargarse la libreria completa de programación del ESP32, la cual es muy extensa. Después de instalada por primera vez, el proceso es muy rápido.**

Abra PowerShell y ejecute:

```powershell
Invoke-WebRequest -Uri https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/cargar_firmware.ps1 -OutFile cargar_firmware.ps1
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\cargar_firmware.ps1
```


