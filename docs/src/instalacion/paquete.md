
# Instalación del paquete

Este paquete requiere Julia 1.12 o superior. Se instala directamente desde este
repositorio, así:

```julia
using Pkg
Pkg.add(url="https://github.com/nebisman/ThermoLab.jl")
```

Luego, para empezar a usarlo, ejecute el siguiente código:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
set_reference(sys, 50)
```
El sistema debe reaccionar y llevar la temperatura a una referencia de $$50\,^oC$$. Los LEDs de
la plataforma muestran con un mapa de color la temperatura, la referencia y la potencia aplicada.

## Notas
Cuando el usuario carga el paquete por primera vez en un directorio de trabajo, se crean automáticamente allí las dos siguientes carpetas:

- `ejemplos/`: scripts, notebooks Pluto y notebooks Jupyter con ejemplos de
  identificación y control (PI, PID, diseño de uno y dos grados de libertad, entre otros).
  Se recomienda iniciar revisando los ejemplos en el directorio `scripts/`.
- `datafiles/`: archivos CSV con los datos generados por los experimentos
  (identificación, respuesta al escalón, PRBS, etc.), usados como entrada y salida  por las funciones del paquete.

Tenga en cuenta que el sistema térmico es lento (su constante de tiempo es de decenas de
segundos), por lo que los experimentos de identificación pueden durar varios minutos.

 ## Problemas

 Si el ejemplo anterior no funciona como se espera, vaya a [Permisos del puerto serial en Linux](@ref) o [Instalación del driver CH340 en Windows](@ref), o intente la [Instalación del Firmware](@ref).
