## definicion del sistema y parametros
using ThermoLab
sys = ThermalSystem();
G = tf(sys)

## Diseño por localización de polos
# Frecuencia natural y factor de amortiguamiento
ωn = 0.1
ζ = 0.8
s = tf("s")
T = ωn^2/(s^2 + 2*ζ*ωn*s + ωn^2)

# calculo de las constantes del PI
_, Kp, Ki = placePI(G, ωn, ζ; form=:parallel)
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=0)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = max(120, 12/(ζ * ωn)));
stepinfo(result, T)


## Ahora diseñamos un PI con Loopshaping,

ωgc = 0.1
# Note que incluimos el retardo del muestreo digital, por lo cual se requiere
# el paquete completo ControlSystems
using ControlSystems
Gd = G*delay(SAMPLING_TIME)
C, Kp, Ki, fig, CF = loopshapingPI(Gd, ωgc; rl=1,  phasemargin=60, form=:parallel)

T1 = feedback(C*Gd, 1)
# y lo probamos
set_pid(sys;  kp=Kp, ki=Ki, kd=0, beta=1)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = max(120, 16/ωgc));
stepinfo(result, T1)
