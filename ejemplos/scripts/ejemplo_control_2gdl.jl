# Ejemplo: controlador de dos grados de libertad (2-GDL)
# Supongamos que queremos diseñar e implementar un controlador de dos
# grados de libertad por métodos algebraicos.
# Para ello usamos el siguiente código:


## definiciones
using ThermoLab
sys = ThermalSystem();
G = tf(sys; numpar=3)

## Vamos a implementar la siguiente funcion itae
ω₀ = 0.3
s=tf("s")
T = ω₀^3/(s^3 + 1.75*s^2*ω₀ + 2.15*s*ω₀^2 + ω₀^3)
C = cont2dof(G, T, 2, [-4*ω₀])

# respuesta del controlador
set_controller(sys, C)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, T)




## Ahora vamos a implementar una función cuadrática óptima
Tlq, Gur = lqmodel(G, 50)
C = cont2dof(G, Tlq, 2 , [-1.0, -1.2])
set_controller(sys, C)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, Tlq)
