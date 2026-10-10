# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_basico.jl – Uso básico del paquete UNThermal.jl
#
#  Muestra cómo:
#    1. Crear el sistema y configurar el PID
#    2. Ejecutar un escalón en lazo cerrado con stepinfo_exp
#    3. Ejecutar escaleras en lazo cerrado
#    4. Ejecutar un perfil de referencia
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNThermal

# definición del sistema
sys = ThermalSystem(port="/dev/ttyUSB0")

s = tf("s")

# configurar PID presintonizado
set_pid!(sys; kp=16.796, ki=2.0, kd=16.441, N=27.38, beta=0.1)

# escalón en lazo cerrado
result = step_closed(sys; r0=40.0, r1=55.0, t0=60.0, t1=120.0)
stepinfo_exp(result)

# escaleras en lazo cerrado
stairs_closed(sys; stairs=(40, 48, 55, 48), duration=60.0)

# perfil de referencia en lazo cerrado
profile_closed(sys;
    timevalues = (0, 120, 240, 360, 480),
    refvalues  = (40, 40, 55, 55, 40))
