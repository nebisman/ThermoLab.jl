# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_dise2P.jl – Diseño de controlador 2-DOF para UNThermal
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNThermal

include("ControlUN.jl")

# definición del sistema
sys = ThermalSystem()

s = tf("s")

# función de transferencia del sistema térmico
G = transfer_function(sys)

# función de lazo cerrado deseada (ITAE primer orden)



z=0.9
ωn = 0.5
T = ωn^2 / (s^2 + 2*z*ωn*s  + ωn^2)



# diseño y carga del controlador 2-DOF
C2 = dise2p(G, T, 2, [-5])
set_controller(sys, C2)

# respuesta del controlador
result = step_closed(sys; r0=40.0, r1=50.0, t0=40, t1=40)
stepinfo_exp(result; T)

