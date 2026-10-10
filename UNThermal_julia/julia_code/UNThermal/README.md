# UNThermal.jl

Paquete de Julia para control e identificación del **sistema térmico UNThermal** — controlado por un ESP32 — vía comunicación serial (USB).

Equivalente funcional del paquete Python (`thermalsys.py`, `controlsys.py`, `identsys.py`).

---

## Requisitos previos

| Requisito | Versión mínima | Notas |
|-----------|---------------|-------|
| **Julia** | 1.9+ | Descargar de [julialang.org](https://julialang.org/downloads/) |
| **ESP32** | — | Con firmware `esp32_unthermal.ino` cargado |
| **Cable USB** | — | Conectado al ESP32 |
| **Permisos serial** (Linux) | — | El usuario debe pertenecer al grupo `dialout` |

### Permisos del puerto serial en Linux

```bash
sudo usermod -a -G dialout $USER
# Reiniciar sesión o ejecutar:
newgrp dialout
```

---

## Instalación

### Opción A: Script automático

```bash
cd UNThermal
julia setup.jl
```

### Opción B: Manual desde el REPL de Julia

```julia
import Pkg
Pkg.add([
    "LibSerialPort", "JSON3", "Interpolations", "DelimitedFiles",
    "ControlSystems", "ControlSystemsBase", "Plots", "Statistics",
    "Printf", "LinearAlgebra", "MatrixEquations", "Optim", "Polynomials",
    "RobustAndOptimalControl", "ControlSystemIdentification", "DSP",
])
cd("ruta/a/UNThermal")
Pkg.activate(".")
Pkg.instantiate()
```

### Opción C: Desde Jupyter con IJulia

```julia
import Pkg
Pkg.activate("ruta/a/UNThermal")
Pkg.instantiate()
using UNThermal
```

---

## Paquetes de Julia utilizados

| Paquete | Equivalente Python | Propósito |
|---------|--------------------|-----------|
| `LibSerialPort` | `pyserial` | Comunicación serial con ESP32 |
| `JSON3` | `json` | Parsing de tramas JSON |
| `ControlSystems` | `python-control` | Funciones de transferencia, `lsim`, `c2d` |
| `MatrixEquations` | `scipy.linalg (dare)` | Ecuación de Riccati para anti-windup |
| `ControlSystemIdentification` | `sippy` | Identificación ARX desde PRBS |
| `DSP` | `scipy.signal` | Filtros para procesamiento de señales |
| `Polynomials` | `numpy.polynomial` | Manejo de polinomios (diseño 2-DOF) |
| `Interpolations` | `scipy.interpolate` | Interpolación de curva estática |
| `Plots` | `matplotlib` | Gráficas en tiempo real |
| `DelimitedFiles` | `csv` | Lectura/escritura CSV |
| `LinearAlgebra` | `numpy.linalg` | Operaciones matriciales |
| `Statistics` | `numpy (mean, std)` | Estadísticas básicas |

---

## Estructura del paquete

```
UNThermal/
├── Project.toml            # Manifiesto de dependencias
├── setup.jl                # Script de instalación
├── README.md               # Este archivo
├── src/
│   ├── UNThermal.jl        # Módulo principal
│   ├── thermalsys.jl       # Comunicación serial + conversiones hex
│   ├── controlsys.jl       # Funciones de control
│   └── identsys.jl         # Funciones de identificación
├── ejemplos/
│   ├── ControlUN.jl                # Funciones de diseño de controladores
│   ├── ejemplo_basico.jl           # PID, escalón, escaleras, perfil
│   ├── ejemplo_identificacion.jl   # Curva estática, FOTD, PRBS
│   ├── ejemplo_control_PI.jl       # Diseño y carga de controlador PI
│   ├── ejemplo_control_PID.jl      # Diseño y carga de controlador PID
│   └── ejemplo_dise2P.jl           # Diseño de controlador 2-DOF
├── datafiles/              # Modelos y datos guardados (generados)
└── experiment_files/       # Archivos CSV de experimentos (generados)
```

---

## Protocolo serial

El firmware del ESP32 usa el protocolo de dos líneas:

```
<topic>\n
<json_payload>\n
```

**Respuestas del ESP32** (una por tiempo de muestreo, h = 0.8 s):
```json
{"np":"00000000","r":"42200000","y":"42240000","u":"41a00000"}
```

Campo `np` = número de punto de muestreo (equivale a `frame` del motor DC).
Los valores float se codifican como hexadecimal IEEE-754 big-endian de 32 bits.

---

## Uso rápido

### 1. Crear el sistema y configurar PID

```julia
using UNThermal

sys = ThermalSystem(
    port  = "/dev/ttyUSB0",   # Linux
    # port = "COM3",          # Windows
    bauds = 115200,
)

set_pid!(sys; kp=16.796, ki=2.0, kd=16.441, N=27.38, beta=0.1)
```

### 2. Escalón en lazo cerrado

```julia
result = step_closed(sys; r0=40, r1=55, t0=60, t1=120)
stepinfo_exp(result)
```

### 3. Escaleras en lazo cerrado

```julia
t, r, y, u = stairs_closed(sys;
    stairs   = (40, 48, 55, 48),
    duration = 60.0)
```

### 4. Perfil de referencia

```julia
t, r, y, u = profile_closed(sys;
    timevalues = (0, 120, 240, 360, 480),
    refvalues  = (40, 40, 55, 55, 40))
```

### 5. Escalón en lazo abierto

```julia
t, u, y = step_open(sys; yop=50, amplitude=10, t1=600)
```

### 6. Señal PRBS en lazo abierto

```julia
t, u, y = prbs_open(sys; yop=50, amplitude=4, divider=35)
```

### 7. Modelos FO y FOTD desde PRBS

```julia
G1, G2, L = get_models_prbs(sys; yop=50, amplitude=4)
```

### 8. Modelo FOTD desde escalón

```julia
G, L = get_fomodel_step(sys; yop=50, amplitude=10)
```

### 9. Curva estática

```julia
uee, yee = get_static_model(sys; step=5.0)
```

### 10. Controlador general 1-DOF

```julia
s = tf("s")
C = 10.0 + 0.5/s
set_controller!(sys, C)
t, r, y, u = step_closed(sys; r0=40, r1=55, t0=60, t1=180)
```

### 11. Controlador 2-DOF (desde ControlUN.jl)

```julia
include("ejemplos/ControlUN.jl")
G  = transfer_function(sys)
T  = 0.06^2 / (s^2 + 2*0.7*0.06*s + 0.06^2)
C2 = dise2p(G, T, 1, [-0.15])
set_controller!(sys, C2)
```

### 12. Función de transferencia nominal

```julia
G = transfer_function(sys)
bodeplot(G)
```

---

## Correspondencia Python → Julia

| Python | Julia | Notas |
|--------|-------|-------|
| `ThermalSystemIoT(plant_number=..., broker_address=...)` | `ThermalSystem(port=..., bauds=...)` | Serial en lugar de MQTT |
| `set_pid(system, kp=1, ...)` | `set_pid!(sys; kp=1, ...)` | `!` indica mutación |
| `set_reference(system, 50)` | `set_reference!(sys, 50)` | |
| `set_controller(system, C)` | `set_controller!(sys, C)` | |
| `step_closed(system, r0=40, r1=55)` | `step_closed(sys; r0=40, r1=55)` | |
| `stairs_closed(system, signal=[40,50])` | `stairs_closed(sys; stairs=(40,50))` | Tupla o vector |
| `profile_closed(system, ...)` | `profile_closed(sys; ...)` | |
| `step_open(system, yop=50, ...)` | `step_open(sys; yop=50, ...)` | |
| `prbs_open(system, yop=50, ...)` | `prbs_open(sys; yop=50, ...)` | |
| `get_static_model(system)` | `get_static_model(sys)` | |
| `get_fomodel_step(system, yop=50)` | `get_fomodel_step(sys; yop=50)` | |
| `get_models_prbs(system, yop=50)` | `get_models_prbs(sys; yop=50)` | |

---

## Solución de problemas

**"No se recibieron datos del ESP32"**
- Verificar cable USB y puerto correcto (`ls /dev/tty*` en Linux)
- Verificar que el firmware `esp32_unthermal.ino` esté cargado
- El sistema térmico usa **115200 baudios** (distinto al motor DC: 460800)

**"Permission denied" al abrir el puerto**
- Linux: `sudo usermod -a -G dialout $USER` y reiniciar sesión
- Windows: instalar el driver CH340 o CP210x del ESP32

**Los experimentos de lazo abierto tardan mucho**
- Normal: el firmware primero estabiliza la temperatura en `yop` antes
  de aplicar el escalón o la PRBS (fase de estabilización ≈ 60 s).

**Las gráficas no se actualizan en tiempo real**
- Usar el backend `gr()` de Plots.jl (por defecto)
- En Jupyter, probar `plotlyjs()` o `gr()`

---

## Licencia

MIT License — LB 2026
