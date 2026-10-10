# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_control_PI.jl – Diseño de controlador PI para UNThermal
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
#Pkg.instantiate()
using UNThermal
include("ControlUN.jl")

# definición del sistema
sys = ThermalSystem(port="/dev/ttyUSB0", bauds = 460800)

s = tf("s")

# identificación del sistema
G, L = get_model_prbs(sys, usefile = true)
# parametros del sistema

a = 1/35# denvec(G)[1][2]
b = 1.16/35# numvec(G)[1][1]

# especificaciones transitorias
SP  = 0.05     # sobreimpulso máximo 5 %
tee = 20.0    # tiempo de establecimiento [s]
tr  = 7.0     # tiempo de subida [s]



# iteración 
ωn = 0.3
ζ  = 0.7
T  = ωn^2 / (s^2 + 2*ζ*ωn*s + ωn^2)

# cálculo de las constantes PI
Kp = (2*ζ*ωn - a) / b
Ki = ωn^2 / b


# experimentos
set_pid(sys; kp=Kp, ki=Ki, kd=0, N=10.0, beta=0)
result = step_closed(sys; r0=50.0, r1=60.0, t0=30, t1=30); 
stepinfo_exp(result; T)