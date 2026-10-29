# Permisos del puerto serial en Linux

En Linux, el acceso a los puertos seriales (por ejemplo, `/dev/ttyUSB0`) suele
estar restringido a los usuarios que pertenecen al grupo `dialout`. Si al
conectar la plataforma se obtiene un error de permisos (`Permission denied`),
añada su usuario a dicho grupo, de la siguiente manera:

```bash
sudo usermod -a -G dialout Mi_Usuario
```

En donde `Mi_Usuario` representa su usuario actual en linux. Luego cierre sesión y vuelva a iniciarla (o reinicie el equipo) para que el
cambio de grupo surta efecto. Puede verificar que el usuario quedó incluido
con:

```bash
groups Mi_Usuario
```

## Verificación

Para identificar en Linux el nombre exacto del puerto asignado a la placa
ESP32, conéctela por USB y ejecute en la terminal:

```bash
curl -fsSLO https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/encontrar_puerto.sh
chmod +x encontrar_puerto.sh
./encontrar_puerto.sh
```

Este script identifica la placa entre los dispositivos serie del sistema, y
reporta el puerto detectado, por ejemplo `Puerto detectado: /dev/ttyUSB0`,
el cual debe usarse para la conexión serial. Si no detecta ningún puerto, o
detecta más de uno, verifique que la placa esté conectada y encendida.

## Diagnóstico y corrección automática

Si la placa sigue sin detectarse, o el puerto aparece pero no se puede usar,
el script `esp32_diag.sh` revisa de forma sistemática las causas más comunes
en Linux. Con la placa conectada por USB, descárguelo y ejecútelo en la terminal:

```bash
curl -fsSLO https://raw.githubusercontent.com/nebisman/ThermoLab.jl/main/firmware/instalador/esp32_diag.sh
chmod +x esp32_diag.sh
./esp32_diag.sh
```

Ejecutado así, el script **solo diagnostica y no modifica nada**: revisa el
sistema y el kernel, si el chip USB-serial de la placa es reconocido (CP210x,
CH340/CH9102, FTDI o el USB nativo del ESP32), si su módulo del kernel está
cargado, los mensajes del kernel (cable o puerto USB defectuoso, falta de
energía), si existe el puerto `/dev/ttyUSB*` o `/dev/ttyACM*`, los conflictos
con `brltty` y `ModemManager`, los permisos del usuario (grupo `dialout`) y si
otro programa tiene ocupado el puerto. Al final muestra un resumen con cada
problema encontrado y el comando que lo corrige.

Para que además aplique las correcciones, ejecútelo con la opción `--fix`:

```bash
./esp32_diag.sh --fix
```

En este modo el script pide confirmación (`s/N`) antes de aplicar cada
corrección, y solicita la contraseña de `sudo` cuando la necesita. Ejecútelo
siempre con su usuario normal, no con `sudo`, para que pueda revisar los
permisos de su usuario.

Después de aplicar las correcciones:

1. Desconecte y vuelva a conectar la placa.
2. Si el script agregó su usuario al grupo `dialout`, cierre sesión y vuelva a
   iniciarla (o reinicie el equipo).
3. Ejecute de nuevo `./esp32_diag.sh` para verificar que ya no se reportan
   problemas.

Si el script indica que el sistema no ve ningún dispositivo nuevo al conectar
la placa, el problema suele ser de hardware: pruebe con otro cable USB que
transmita datos (no solo de carga), con otro puerto USB, y sin hubs
intermedios.
