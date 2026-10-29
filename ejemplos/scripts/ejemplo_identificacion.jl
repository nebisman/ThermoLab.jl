# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_identificacion.jl – Identificación del sistema ThermoLab
#
# ═══════════════════════════════════════════════════════════════════════════════


## 1. importamos el paquete ThermoLab y creamos el sistema
using ThermoLab
sys = ThermalSystem();


## 2. Ahora obtenemos el modelo estático (tarda unos 11 minutos)
uee, yee = get_static_model(sys);

## 3. Podemos obtener la respuesta al escalón en lazo abierto con:
t, u, y = step_open(sys; yop=50, amplitude=10, t1=100)


## 4. Podemos estimar los parámetros de la planta por respuesta al escalón
##    (con usefile=true se reutilizan los datos del paso anterior):
G, L = get_model_step(sys; t1=600)

##  5. Podemos estimar los parámetros de la planta por respuesta a una señal PRBS
G, L = get_model_prbs(sys; yop=50, numpar=3)

##  6. Con los mismos datos PRBS podemos estimar un modelo de dos polos
G2, L = get_model_prbs(sys; numpar=2, usefile=true)


