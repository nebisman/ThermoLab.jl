## definicion del sistema: modelo de dos polos b/((s+a1)(s+a2))
using ThermoLab
sys = ThermalSystem();
G2 = tf(sys; numpar=3)

b  = numvec(G2)[1][1]
a1a2_suma, a1a2_prod = denvec(G2)[1][2], denvec(G2)[1][3]

## Calculo de las constantes del PID por asignación de polos
s = tf("s")
ωn = 0.4
ζ = 0.7
n = 2

T  = n*ωn^3/((s+n*ωn)*(s^2 + 2*ζ*ωn*s + ωn^2))
Kd = ((2ζ + n) * ωn - a1a2_suma) / b
Kp = ((1 + 2n * ζ) * ωn^2 - a1a2_prod) / b
Ki = n * ωn^3 / b
set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, N=10, beta=0)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, T)



#%% calculo de las constantes del PID con polinomio ITAE

ωn = 0.75
T = (ωn^3)/(s^3 + 1.75*s^2*ωn + 2.15*s*ωn^2 + ωn^3)
Kd = (1.75 * ωn - a1a2_suma) / b
Kp = (2.15 * ωn^2 - a1a2_prod) / b
Ki = ωn^3/b

set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, beta=0,  N=10)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, T)
