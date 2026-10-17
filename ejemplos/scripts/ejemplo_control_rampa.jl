# Ejemplo: controlador de dos grados de libertad (2-GDL)
# Supongamos que queremos diseñar e implementar un controlador de dos
# grados de libertad por métodos algebraicos.
# Para ello usamos el siguiente código:


## definiciones
using ThermoLab
sys = ThermalSystem();
G = tf(sys; numpar=3)


## Vamos a implementar la siguiente funcion itae
ω₀ = 0.6
T = itaemodel(3,ω₀;type=:speed)
C = cont2dof(G, T, 2, [-5*ω₀])

# respuesta del controlador
set_controller(sys, C)
profile_closed(sys; timevalues = [0.0, 45, 90],
                        refvalues  = [40.0, 55.0, 75.0])



stairs_closed(sys; stairs   = ( 50, 55,60),
                       duration= 30.0)
