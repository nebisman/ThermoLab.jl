# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_identificacion.jl – Identificación del sistema UNThermal
#
#  Muestra cómo:
#    1. Obtener la curva estática (temperatura vs potencia)
#    2. Obtener un modelo FOTD desde un escalón en lazo abierto
#    3. Obtener modelos FO y FOTD desde PRBS
#    4. Recuperar la función de transferencia nominal
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using UNThermal
using ControlSystems
using Plots

# definición del sistema
sys = ThermalSystem(port="/dev/ttyUSB0")

# curva estática
uee, yee = get_static_model(sys; step=5.0)
println("Curva estática: $(length(uee)) puntos")

# escalón en lazo abierto
t, u, y = step_open(sys; yop=50.0, amplitude=10.0, t1=600.0)
println("Step open: $(length(t)) muestras")

# modelo FOTD desde escalón
G, L = get_fomodel_step(sys; yop=50.0, usefile=true)
println("Modelo FOTD: $G   L=$(round(L, digits=2)) s")

# modelos FO y FOTD desde PRBS
G1, G2, tau2 = get_models_prbs(sys; yop=50.0, amplitude=4.0, divider=35)
println("Modelo FO  (PRBS): $G1")
println("Modelo FOTD (PRBS): $G2  L=$(round(tau2, digits=2)) s")

# función de transferencia nominal (del archivo guardado)
G_nom = transfer_function(sys)
println("Función de transferencia nominal: $G_nom")
bodeplot(G_nom, title="Bode – Sistema Térmico")
