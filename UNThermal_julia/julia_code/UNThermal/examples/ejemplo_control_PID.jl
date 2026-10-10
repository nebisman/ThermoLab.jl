# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_control_PID.jl – Diseño de controlador PID para UNThermal
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNThermal
include("ControlUN.jl")

# definición del sistema
sys = ThermalSystem(port="/dev/ttyUSB0")

s = tf("s")

# función de transferencia del sistema
G = transfer_function(sys)

# parámetros del modelo G(s) = b / (a·s + 1)
b = numvec(G)[1][1]
a = denvec(G)[1][2]

# especificaciones transitorias
tee = 120.0    # tiempo de establecimiento [s]
tr  = 40.0     # tiempo de subida [s]
SP  = 0.05     # sobreimpulso máximo 5 %

# valores iniciales
n   = 3
ζ0  = abs(log(SP)) / sqrt(pi^2 + (log(SP))^2)
ωn1 = (1.197 - 0.43*ζ0 + 2.553*ζ0^2) / tr
ωn2 = 5 / tee
ωn3 = a / (2*ζ0 + n)
ωn0 = maximum([ωn1, ωn2, ωn3])

# iteración final
ωn = 0.07
ζ  = 0.7

# Cálculo de las constantes del PID de segundo orden
T   = n*ωn^3 / ((s + n*ωn) * (s^2 + 2*ζ0*ωn*s + ωn^2))
Kd  = ((2ζ + n) * ωn - a) / b
Kp  = (1 + 2n*ζ) * ωn^2 / b
Ki  = n * ωn^3 / b

# Cálculo de las constantes del PID con criterio ITAE
T   = ωn^3 / (s^3 + 1.75*s^2*ωn + 2.15*s*ωn^2 + ωn^3)
Kd  = (1.78*ωn - a) / b
Kp  = 2.15 * ωn^2 / b
Ki  = ωn^3 / b

set_pid!(sys; kp=Kp, ki=Ki, kd=Kd, N=20.0, beta=0.5)

result = step_closed(sys; r0=40.0, r1=55.0, t0=60.0, t1=180.0)
stepinfo_exp(result; T)
using Printf

numero = 3.141592653589793
@printf("%.8g\n", numero)
@printf("%.8g\n", Kd)