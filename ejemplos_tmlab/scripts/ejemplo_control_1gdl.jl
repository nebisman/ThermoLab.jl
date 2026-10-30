## Ejemplo 1: controlador de  un grado de libertad (1-GDL)
# Supongamos que queremos diseñar e implementar un controlador de 1-GDL
# usando un método algebraico
# Para ello usamos el siguiente código:


## definiciones
using ThermoLab
sys = ThermalSystem();
G = tf(sys, numpar=3)


## diseño y carga del controlador
ω= 0.35
ζ = 0.7
p1 = -ω*ζ + ω*sqrt(1-ζ^2)im
p2 = conj(p1)

polos_lc = [p1, p2, -5ω, -5ω ]
C = cont1dof(G, polos_lc)
set_controller(sys, C)


# Aqui vemos la respuesta al escalon en lazo cerrado

T = feedback(C*G, 1)
result = step_closed(sys; r0 = 40, r1 = 46,  t0 = 40, t1 = 40);
stepinfo(result, T)
