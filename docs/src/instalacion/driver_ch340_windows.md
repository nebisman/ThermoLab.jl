# Instalación del driver CH340 en Windows

La placa ESP32 de la plataforma ThermoLab se comunica con el computador por
USB a través de un chip conversor USB-serial CH340 (o CH341). Algunas veces **Windows no trae este driver instalado por defecto**, así que la placa no aparece como un puerto COM disponible hasta que se instale
el drive manualmente.

## Síntomas

Si el driver no está instalado, al conectar la placa por USB ocurre alguna
de estas situaciones:


- En el **Administrador de dispositivos** (`Win + X` → *Administrador de
  dispositivos*), la placa aparece bajo **Otros dispositivos** como
  `USB2.0-Serial` o `CH340`, con un ícono de advertencia amarillo, y no bajo
  **Puertos (COM y LPT)**.

- Al ejecutar estas instrucciones en el Powershell no se detecta ningún puerto de conexión:

```powershell
Invoke-WebRequest -Uri https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/encontrar_puerto.ps1 -OutFile encontrar_puerto.ps1
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\encontrar_puerto.ps1
```

- Los scripts de instalación del firmware (`flashear_firmware.ps1`,
  `cargar_firmware.ps1`, ver [Instalación del Firmware](@ref)) no detectan
  ningún puerto compatible.




## Instalación del driver

1. Descargue el instalador oficial del fabricante del chip (WCH) desde:
   [https://www.wch-ic.com/downloads/CH341SER_ZIP.html](https://www.wch-ic.com/downloads/CH341SER_ZIP.html).
   El archivo `CH341SER.ZIP` incluye el driver para CH340 y CH341.
2. Descomprima el archivo descargado.
3. Desconecte la placa. 
4. Ejecute `CH341SER.EXE` **como administrador** (clic derecho →
   *Ejecutar como administrador*).
5. En la ventana del instalador, haga clic en **Install** (Instalar).
   Debe aparecer un mensaje confirmando que la instalación fue exitosa.
6. Vuelva a conectar la placa por USB.

## Verificación

Abra el **Administrador de dispositivos** y expanda **Puertos (COM y
LPT)**. Debe aparecer una entrada como:

```
USB-SERIAL CH340 (COM3)
```


Alternativamente, puede verificar la detección desde PowerShell con estas instrucciones:

```powershell
Invoke-WebRequest -Uri https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/encontrar_puerto.ps1 -OutFile encontrar_puerto.ps1
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\encontrar_puerto.ps1
```

Este script identifica la placa entre los dispositivos del sistema  y
reporta el puerto al cual esta conectada, por ejemplo `Puerto detectado: COM3`. 

## Si la placa sigue sin detectarse

- **Pruebe con otro cable USB.** Esta es la causa más común y se suele
  pasar por alto: muchos cables USB son solo de carga y no tienen las
  líneas de datos conectadas.
- **Pruebe con otro puerto USB**, preferiblemente uno USB 2.0 conectado
  directamente a la tarjeta madre, evitando hubs USB.
- **Reinstale el driver desde cero** si el dispositivo sigue apareciendo
  con advertencia en el Administrador de dispositivos después de instalar:
  clic derecho sobre el dispositivo → *Desinstalar dispositivo* → marque
  la casilla *Eliminar el software de controlador para este dispositivo* →
  desconecte la placa, y repita la instalación del driver.
- **Verifique que Windows no esté en modo S** (*S mode*), disponible en
  algunas instalaciones de Windows 10/11, ya que impide instalar drivers
  que no provengan de Microsoft Store.
