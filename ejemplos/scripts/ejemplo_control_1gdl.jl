## Ejemplo 1: controlador de  un grado de libertad (1-GDL)
# Supongamos que queremos diseñar e implementar un controlador de 1-GDL
# usando un método algebraico
# Para ello usamos el siguiente código:


## definiciones
using ThermoLab
sys = ThermalSystem();
G = tf(sys; numpar=3)


## diseño y carga del controlador
polos_lc = [-0.15+0.1im, -0.15-0.1im, -1.0, -1.0]
C = cont1dof(G, polos_lc)
set_controller(sys, C)


# Aqui vemos la respuesta al escalon en lazo cerrado

T = feedback(C*G, 1)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, T)
