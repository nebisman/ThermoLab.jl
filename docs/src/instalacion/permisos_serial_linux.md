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