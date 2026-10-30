## definicion del sistema: modelo de dos polos
##   G(s) = b/(s^2 + a1*s + a2)
using ThermoLab
sys = ThermalSystem();
G2 = tf(sys; numpar=3)
b  = numvec(G2)[1][1]
a1, a2 = denvec(G2)[1][2], denvec(G2)[1][3]


## Calculo de las constantes del PID por asignación de polos
s = tf("s")
ωn = 0.4
ζ = 0.7
n = 3

T  = n*ωn^3/((s+n*ωn)*(s^2 + 2*ζ*ωn*s + ωn^2))
Kd = ((2ζ + n) * ωn - a1) / b
Kp = ((1 + 2n * ζ) * ωn^2 - a2) / b
Ki = n * ωn^3 / b
set_pid(sys;  kp=Kp, ki=Ki, kd=Kd, N=10, beta=0)
result = step_closed(sys; r0 = 45, r1 = 55,  t0 = 30, t1 = 30);
stepinfo(result, T)