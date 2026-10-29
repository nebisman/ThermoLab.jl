# ═══════════════════════════════════════════════════════════════════════════════
#  controlsys.jl – Funciones de control para el sistema ThermoLab.
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using ControlSystemsBase
using LinearAlgebra
using Plots
using Printf
using Statistics
using RobustAndOptimalControl
using LaTeXStrings
import ControlSystemsBase: stepinfo




DEFAULT_REF = 40

# ═══════════════════════════════════════════════════════════════════════════════
#  set_reference!
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_reference(sys::ThermalSystem, ref::Real)

Fija la referencia de temperatura de la plataforma ThermoLab en `ref` (°C). El
controlador cargado en la plataforma (PID o general, ver [`set_pid`](@ref) y
[`set_controller`](@ref)) lleva la temperatura a ese valor y la mantiene allí.

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.
- `ref::Real`: temperatura de referencia, en °C. Debe estar entre la
  temperatura ambiente y 90 °C (`MAX_TEMPERATURE`).

# Retorna
- `nothing`. Imprime en consola la referencia fijada.

# Notas
- Al recibir una nueva referencia el firmware reinicia el estado del
  controlador (por ejemplo, la acción integral del PID).
- Todos los experimentos de este paquete fijan la referencia en 40 °C
  (`DEFAULT_REFERENCE`) al terminar.

# Ejemplo
```julia
using ThermoLab
sys = ThermalSystem();
set_reference(sys, 50.0)

    Referencia fijada en 50.0 °C
```
"""
function set_reference(sys::ThermalSystem, ref::Real)
    connect!(sys)
    send_command!(sys, "set_ref", Dict("reference" => float2hex(Float64(ref))))
    disconnect!(sys)
    println("Referencia fijada en $(@sprintf("%.1f", ref)) °C")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  set_pid!
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_pid(sys::ThermalSystem; kp=10.0, ki=2.32, kd=0.0, N=10.0, beta=0.0)

Configura los cinco parámetros `kp`, `ki`, `kd`, `N` y `beta` de un
controlador PID de 2-GDL (dos grados de libertad) en la plataforma ThermoLab,
como el mostrado en la siguiente figura:

![Diagrama de bloques del controlador PID de dos grados de libertad](../../assets/pid2gdl.png)

La ley de control implementada es

``u = k_p\\,(\\beta\\,r - y) + k_i \\displaystyle\\int_0^t (r - y)\\,d\\tau - \\dfrac{k_d\\,s}{1 + (k_d/N)\\,s}\\, y``

donde ``r`` es la referencia (°C), ``y`` la temperatura medida (°C) y ``u`` la
potencia aplicada (%), saturada al intervalo ``[0, 100]``.

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `kp::Real=10.0`: ganancia proporcional.
- `ki::Real=2.32`: ganancia integral.
- `kd::Real=0.0`: ganancia derivativa.
- `N::Real=10.0`: coeficiente del filtro del derivador.
- `beta::Real=0.0`: ponderación del punto de ajuste (*setpoint weighting*)
  en la acción proporcional.

# Retorna
- `nothing`. Imprime en consola los parámetros configurados.

# Notas
- Con `beta=1.0` se obtiene el PID estándar de un grado de libertad (1-GDL).
  La acción derivativa actúa solo sobre la temperatura medida.
- Esta implementación se basa en el algoritmo del libro [*Feedback
  Systems: An Introduction for Scientists and Engineers*](https://fbswiki.org/wiki/index.php/PID_Control),
  segunda edición, de Åström y Murray, con anti-windup por cálculo hacia atrás
  y periodo de muestreo de 0.8 s (`SAMPLING_TIME`).
- Tras cargar el PID, la referencia se fija en 40 °C.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab y de haber
definido el sistema, así:

```julia
using ThermoLab
sys = ThermalSystem();
```

Luego, el controlador PID se programa así:

```julia
set_pid(sys; kp=11.8, ki=2.7, kd=0.0, N=10.0, beta=0.0)

    Referencia fijada en 40.0 °C
    PID actualizado: kp=11.8  ki=2.7  kd=0.0  N=10.0  β=0.0
```
"""
function set_pid(sys::ThermalSystem;
                  kp::Real   = 10.0,
                  ki::Real   = 2.32,
                  kd::Real   = 0.0,
                  N::Real    = 10.0,
                  beta::Real = 0.0)
    payload = Dict(
        "kp"   => float2hex(kp),
        "ki"   => float2hex(ki),
        "kd"   => float2hex(kd),
        "N"    => float2hex(N),
        "beta" => float2hex(beta),
    )
    connect!(sys)
    send_command!(sys, "set_pid", payload)  
    set_reference(sys, DEFAULT_REF);    
    println("PID actualizado: kp=$kp  ki=$ki  kd=$kd  N=$N  β=$beta")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  set_controller
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_controller(sys::ThermalSystem, controller)

Programa un controlador general en la plataforma ThermoLab. El controlador se
define como una función de transferencia continua en la variable de Laplace
`s`, la cual se discretiza automáticamente.

Esta función permite programar un controlador estándar de 1 grado de
libertad (1-GDL), como el mostrado en la siguiente figura:

![Controlador de 1 grado de libertad](../../assets/controller_1gdl.png)

A su vez, esta misma función permite programar un controlador de 2
grados de libertad (2-GDL), como se muestra a continuación:

![Controlador de 2 grados de libertad](../../assets/controller_2gdl.png)

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.
- `controller`: función o matriz de transferencia continua
  (`ControlSystemsBase.TransferFunction`, o cualquier sistema LTI convertible
  con `ss`).
  - Si ``C`` es 1×1 (una entrada y una salida), se interpreta como controlador
    de 1-GDL:

    ``u = C\\,(r - y)``

  - Si ``C=[C_1\\quad C_2]`` es una matriz 1×2 (dos entradas y una salida), se
    interpreta como controlador de 2-GDL:

    ``u = C_1\\,r + C_2\\,y``

    Por ejemplo, el controlador que retorna [`cont2dof`](@ref) tiene la forma
    ``C_1=\\frac{L}{A}`` y ``C_2=-\\frac{M}{A}``.

# Retorna
- `nothing`. Imprime en consola el tipo y el orden del controlador cargado.

# Notas
- El controlador se lleva a una realización mínima en espacio de estados, se
  discretiza con el método bilineal (Tustin) con periodo de muestreo de 0.8 s
  (`SAMPLING_TIME`) y se expresa en forma modal.
- Se implementa un mecanismo anti-windup basado en un observador, cuya
  ganancia se calcula mediante el filtro de Kalman; la potencia aplicada se
  satura al intervalo ``[0, 100]`` %.
- Tras cargar el controlador, la referencia se fija en 40 °C.

# Ejemplos
## Ejemplo 1: controlador de un grado de libertad (1-GDL)
Supongamos que queremos implementar el controlador PI

``C(s)=10 + \\frac{2}{s}``

Para ello usamos el siguiente código:

```julia
using ThermoLab
sys = ThermalSystem();
s = tf("s");
C = 10 + 2/s;
set_controller(sys, C)

    Referencia fijada en 40.0 °C
    Controlador cargado en sistema térmico (1-DOF, orden=1)
```

## Ejemplo 2: controlador de dos grados de libertad (2-GDL)
El controlador de 2-GDL puede diseñarse, por ejemplo, con [`cont2dof`](@ref):

```julia
using ThermoLab
sys = ThermalSystem();
G = tf(sys; numpar=3)
T = itaemodel(3, 0.8)
C = cont2dof(G, T, 2, [-4.0])
set_controller(sys, C)
```
"""
function set_controller(sys::ThermalSystem, controller)
    
    h = SAMPLING_TIME
    ni = size(controller, 2)
    type_control  = ni - 1

    
    sys_c = minreal(ss(controller))

    # ── Discretización bilineal (Tustin) ──────────────────────────────
    sys_int = c2d(sys_c, SAMPLING_TIME, :tustin)
    sys_d, S, E = modal_form(sys_int)
    Ad, Bd, Cd, Dd = ssdata(sys_d)
    order = size(Ad, 1)
    Q = 100000*diagm(ones(size(Ad,1)))
    
    #Lg = lqr(Discrete, Ad',Cd',Q, 1)# xxx
    #Lg = Lg'
    Lg = kalman(sys_d, Q,1) 

    if ni == 1
       Bd =Matrix{Float64}( hcat(Bd, -Bd))
       Dd =Matrix{Float64}( hcat(Dd, -Dd))
    end


    order = size(Ad, 1)
    Ac_fw = Ad - Lg * Cd
    Bc_fw = Bd - Lg * Dd

 
    payload = Dict(
        "order"       => long2hex(order),
        "A"           => matrix2hex(Ac_fw),
        "B"           => matrix2hex(Bc_fw),
        "C"           => matrix2hex(Cd),
        "D"           => matrix2hex(Dd),
        "L"           => matrix2hex(Lg),
        "typeControl" => long2hex(type_control),
    )

    connect!(sys)
    send_command!(sys, "set_gencon", payload)
    set_reference(sys, DEFAULT_REF); 
    println("Controlador cargado en sistema térmico ($ni-DOF, orden=$order)")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  step_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_closed(sys::ThermalSystem; r0=40, r1=50, t0=60, t1=60)

Ejecuta un experimento de respuesta al escalón en lazo cerrado sobre la
plataforma ThermoLab. El controlador que actúa sobre la plataforma ha sido
previamente cargado con [`set_pid`](@ref) o [`set_controller`](@ref).

La referencia inicia en un valor inicial `r0` y pasa a un valor final `r1`. La
duración de cada nivel de referencia está definida por los tiempos `t0` y
`t1`, respectivamente, como muestra la figura siguiente:

![Respuesta al escalón en lazo cerrado de la plataforma ThermoLab](../../assets/step_closed.png)

Los parámetros de la figura se describen a continuación.

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma, con un
  controlador previamente cargado.

# Argumentos de palabra clave
- `r0::Real=40`: valor inicial de la referencia, en °C.
- `r1::Real=50`: valor final de la referencia, en °C.
- `t0::Real=60`: duración en segundos de la referencia en el valor `r0`.
- `t1::Real=60`: duración en segundos de la referencia en el valor `r1`.

# Retorna
- `(t, r, y, u)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  referencia (°C), la temperatura medida (°C) y la potencia aplicada (%).

# Notas
- La respuesta se grafica en tiempo real durante el experimento.
- Al terminar, la referencia se fija en 40 °C.
- Los datos del experimento se guardan en el archivo CSV
  `datafiles/ThermoLab_step_closed_exp.csv` del directorio de trabajo.
- El resultado puede pasarse directamente a [`stepinfo`](@ref).

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego, obtenga la respuesta al escalón en lazo cerrado, así:

```julia
t, r, y, u = step_closed(sys; r0=40, r1=50, t0=60, t1=120);
```
"""
function step_closed(sys::ThermalSystem;
                     r0::Real = 40.0,
                     r1::Real = 50.0,
                     t0::Real = 60.0,
                     t1::Real = 60.0)
    h           = SAMPLING_TIME
    points_low  = round(Int, t0 / h)
    points_high = round(Int, t1 / h)
    total       = points_low + points_high + 1

    payload = Dict(
        "low_val"     => float2hex(r0),
        "high_val"    => float2hex(r1),
        "points_low"  => long2hex(points_low),
        "points_high" => long2hex(points_high),
    )

    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]
    delta = abs(r1 - r0)

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Respuesta al Escalón en lazo cerrado  r0=$(@sprintf("%.1f",r0)) → r1=$(@sprintf("%.1f",r1)) °C" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, (total-1)*h) (0, (total-1)*h)],
        ylims=[(min(r0,r1) - max(5, 0.5*delta), max(r0,r1) + max(5, 0.5*delta)) (-5, 105)],
        background_color_subplot=[:ivory :mintcream],
        legend=[:bottomright :bottomright],
        grid=true, gridalpha=0.25, margin=5Plots.mm)
    redraw!(plt)

    connect!(sys)
    send_command!(sys, "step_closed", payload)

    on_sample = function(msg, np_val)
        t_curr = (np_val) * h
        y_curr = hex2float(string(msg["y"]))
        u_curr = hex2float(string(msg["u"]))
        r_curr = hex2float(string(msg["r"]))
        push!(tv, t_curr); push!(yv, y_curr)
        push!(uv, u_curr); push!(rv, r_curr)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Respuesta al Escalón en lazo cerrado  r0=$(@sprintf("%.1f",r0)) → r1=$(@sprintf("%.1f",r1)) °C" ""],
            ylabel=["Temperatura (°C)" "Potencia (%)"],
            xlabel=["" "Tiempo (s)"],
            xlims=[(0, (total-1)*h) (0, (total-1)*h)],
            ylims=[(min(r0,r1) - max(5, 0.5*delta), max(r0,r1) + max(5, 0.5*delta)) (-5, 105)],
            background_color_subplot=[:ivory :mintcream],
            legend=:best,
            grid=true, gridalpha=0.15, margin=5Plots.mm)
        
        
        #legends y plots
        leg_size = 10
        ref_str = latexstring(@sprintf("r(t) = %.2f \\,\\, ^\\circ  C", r_curr))
        temp_str = latexstring(@sprintf("y(t) = %.2f \\,\\, ^\\circ  C", y_curr))
        power_str = latexstring(@sprintf("u(t) = %.2f  \\,\\, \\%% ", u_curr))
        
        plot!(plt, subplot=1, tv, rv, label= ref_str, color=:teal,  seriestype=:steppost,
              legendfontsize = leg_size, fg_legend = "#d4aa00", linewidth=1.25)
        
        plot!(plt, subplot=1, tv, yv, label =temp_str, color=:deeppink, 
             legendfontsize = leg_size, linewidth=1.5)
        
        plot!(plt, subplot=2, tv, uv, label=power_str,
              legendfontsize = leg_size, fg_legend= "#5fbcd3", color=:royalblue, linewidth=1.5)
        
        
     
        
        
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=5.0)
    catch e
        println("Error: ", e)
    end
    
    set_reference(sys, DEFAULT_REFERENCE);  
    save_experiment([tv, rv, yv, uv], "ThermoLab_step_closed_exp.csv", "t,r,y,u")
   

    
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  stairs_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    stairs_closed(sys::ThermalSystem; stairs=(40, 50, 55), duration=60)

Ejecuta un experimento de seguimiento de una señal de referencia tipo
"escalera" sobre la plataforma ThermoLab. La referencia toma sucesivamente
cada uno de los valores definidos en `stairs`, cada uno con una duración
`duration`, como se muestra en la figura siguiente:

![Señal de referencia tipo escalera para stairs_closed](../../assets/stairs_response.png)

Los parámetros mostrados en la figura se describen a continuación:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma, con un
  controlador previamente cargado.

# Argumentos de palabra clave
- `stairs=(40, 50, 55)`: colección (vector o tupla) con los valores sucesivos
  de la referencia, en °C.
- `duration::Real=60`: duración en segundos de cada escalón. Se redondea
  hacia arriba a un número entero de periodos de muestreo (0.8 s).

# Retorna
- `(t, r, y, u)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  referencia (°C), la temperatura medida (°C) y la potencia aplicada (%).

# Notas
- El controlador que actúa sobre la plataforma ha sido previamente cargado con
  [`set_pid`](@ref) o [`set_controller`](@ref).
- La respuesta se grafica en tiempo real durante el experimento.
- Al terminar, la referencia se fija en 40 °C.
- Los datos del experimento se guardan en el archivo CSV
  `datafiles/ThermoLab_stairs_closed_exp.csv` del directorio de trabajo.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego, ejecute el experimento con una referencia tipo escalera, así:

```julia
t, r, y, u = stairs_closed(sys; stairs=(40, 50, 55, 45), duration=60);
```
"""
function stairs_closed(sys::ThermalSystem;
                       stairs   = (40, 50, 55),
                       duration::Real = 60.0)
    h        = SAMPLING_TIME
    stairs_v = collect(Float64, stairs)
    dur_pts  = ceil(Int, duration / h)
    total    = length(stairs_v) * dur_pts
    mn, mx   = minimum(stairs_v), maximum(stairs_v)
    span     = mx - mn

    payload = Dict(
        "signal"        => join(float2hex(v) for v in stairs_v),
        "duration"      => long2hex(dur_pts),
        "points_stairs" => long2hex(length(stairs_v)),
        "min_val"       => float2hex(mn),
        "max_val"       => float2hex(mx),
    )

    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Escaleras $(length(stairs_v)) niveles, duración $(@sprintf("%.0f",total*h)) s" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, (total-1)*h) (0, (total-1)*h)],
        ylims=[(mn - max(3, 0.1*span), mx + max(3, 0.1*span)) (-5, 105)],
        background_color_subplot=[:ivory :mintcream],
        legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    redraw!(plt)

    connect!(sys)
    send_command!(sys, "stairs_closed", payload)

    on_sample = function(msg, np_val)
        t_curr = np_val * h
        y_curr = hex2float(string(msg["y"]))
        u_curr = hex2float(string(msg["u"]))
        r_curr = hex2float(string(msg["r"]))
        push!(tv, t_curr); push!(yv, y_curr)
        push!(uv, u_curr); push!(rv, r_curr)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Escaleras $(length(stairs_v)) niveles, duración $(@sprintf("%.0f",total*h)) s" ""],
            ylabel=["Temperatura (°C)" "Potencia (%)"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, (total-1)*h) (0, (total-1)*h)],
            ylims=[(mn - max(3, 0.1*span), mx + max(3, 0.1*span)) (-5, 105)],
            background_color_subplot=[:ivory :mintcream],
            legend=:best, grid=true, gridalpha=0.15, margin=5Plots.mm)
        #legends y plots
        leg_size = 11
        ref_str = latexstring(@sprintf("r(t) = %.2f \\,\\, ^\\circ  C", r_curr))
        temp_str = latexstring(@sprintf("y(t) = %.2f \\,\\, ^\\circ  C", y_curr))
        power_str = latexstring(@sprintf("u(t) = %.2f  \\,\\, \\%% ", u_curr))
        
        plot!(plt, subplot=1, tv, rv, label= ref_str, color=:teal,  seriestype=:steppost,
              legendfontsize = leg_size, fg_legend  = RGBA(0.8314, 0.6667, 0.0, 0.3), linewidth=1.25)
        
        plot!(plt, subplot=1, tv, yv, label =temp_str, color=:deeppink, 
             legendfontsize = leg_size, linewidth=1.5)
        
        plot!(plt, subplot=2, tv, uv, label=power_str,
              legendfontsize = leg_size, fg_legend= "#5fbcd3", color=:royalblue, linewidth=1.5)
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=5.0)
    catch e
        println("Error: ", e)
    end

    set_reference(sys, DEFAULT_REFERENCE);  
    save_experiment([tv, rv, yv, uv], "ThermoLab_stairs_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  profile_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    profile_closed(sys::ThermalSystem; timevalues=[0, 45, 90, 135], refvalues=[40, 55, 45, 50])

Ejecuta en lazo cerrado un perfil de referencia arbitrario sobre la
plataforma ThermoLab. El perfil se define por una serie de instantes de tiempo
``t_1, t_2, \\ldots, t_k`` (en `timevalues`) a los cuales corresponden,
respectivamente, los valores de referencia ``r_1, r_2, \\ldots, r_k`` (en
`refvalues`). Entre dos puntos consecutivos el firmware interpola
linealmente la referencia, como se ilustra a continuación:

![Perfil de referencia definido por puntos para profile_closed](../../assets/profile_response.png)

Los parámetros de la función son los siguientes:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma, con un
  controlador previamente cargado.

# Argumentos de palabra clave
- `timevalues=[0, 45, 90, 135]`: colección con los instantes de tiempo, en
  segundos y en orden creciente, que definen el perfil. Se redondean al
  periodo de muestreo (0.8 s). Si el primero no es `0`, la referencia se
  mantiene en `refvalues[1]` desde `t = 0` hasta ese instante.
- `refvalues=[40, 55, 45, 50]`: colección, de la misma longitud que
  `timevalues`, con los valores de referencia (°C) en cada instante.

# Retorna
- `(t, r, y, u)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  referencia interpolada (°C), la temperatura medida (°C) y la potencia
  aplicada (%).

# Notas
- El controlador que actúa sobre la plataforma ha sido previamente cargado con
  [`set_pid`](@ref) o [`set_controller`](@ref).
- La respuesta se grafica en tiempo real durante el experimento.
- Al terminar, la referencia se fija en 40 °C.
- Los datos del experimento se guardan en el archivo CSV
  `datafiles/ThermoLab_profile_closed_exp.csv` del directorio de trabajo.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego, ejecute el experimento con un perfil de referencia, así:

```julia
t, r, y, u = profile_closed(sys;
    timevalues = [0, 120, 240, 360, 480],
    refvalues  = [40, 40, 55, 55, 40]);
```
"""
function profile_closed(sys::ThermalSystem;
                        timevalues = [0.0, 45, 90, 135.0],
                        refvalues  = [40.0, 55.0, 45.0, 50.0])
    h      = SAMPLING_TIME
    tv_cmd = collect(Float64, timevalues)
    rv_cmd = collect(Float64, refvalues)

    int_tv = [round(Int, p / h) for p in tv_cmd]
    if int_tv[1] != 0
        int_tv = vcat([0, int_tv[1] - 1], int_tv)
        rv_cmd = vcat([rv_cmd[1], rv_cmd[1]], rv_cmd)
    end

    mn, mx = minimum(rv_cmd), maximum(rv_cmd)
    total  = int_tv[end] + 1
    span   = mx - mn

    payload = Dict(
        "timevalues" => time2hex(int_tv),
        "refvalues"  => signal2hex(rv_cmd),
        "points"     => long2hex(length(int_tv)),
        "min_val"    => float2hex(mn),
        "max_val"    => float2hex(mx),
    )
    
    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Perfil duración=$(@sprintf("%.0f",tv_cmd[end])) s, $(length(tv_cmd)) puntos" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, (total-1)*h) (0, (total-1)*h)],
        ylims=[(mn - max(3, 0.1*span), mx + max(3, 0.1*span)) (-5, 105)],
        background_color_subplot=[:ivory :mintcream],
        legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    redraw!(plt)

    connect!(sys)
    send_command!(sys, "prof_closed", payload)

    on_sample = function(msg, np_val)
        t_curr = np_val * h
        y_curr = hex2float(string(msg["y"]))
        u_curr = hex2float(string(msg["u"]))
        r_curr = hex2float(string(msg["r"]))
        push!(tv, t_curr); push!(yv, y_curr)
        push!(uv, u_curr); push!(rv, r_curr)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Perfil duración=$(@sprintf("%.0f",tv_cmd[end])) s, $(length(tv_cmd)) puntos" ""],
            ylabel=["Temperatura (°C)" "Potencia (%)"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, (total-1)*h) (0, (total-1)*h)],
            ylims=[(mn - max(3, 0.1*span), mx + max(3, 0.1*span)) (-5, 105)],
            background_color_subplot=[:ivory :mintcream],
            legend=:best, grid=true, gridalpha=0.15, margin=5Plots.mm)
               #legends y plots
        
        leg_size = 10
        ref_str = latexstring(@sprintf("r(t) = %.2f \\,\\, ^\\circ  C", r_curr))
        temp_str = latexstring(@sprintf("y(t) = %.2f \\,\\, ^\\circ  C", y_curr))
        power_str = latexstring(@sprintf("u(t) = %.2f  \\,\\, \\%% ", u_curr))
        
        plot!(plt, subplot=1, tv, rv, label= ref_str, color=:teal,  seriestype=:steppost,
              legendfontsize = leg_size, fg_legend = RGBA(0.8314, 0.6667, 0.0, 0.3), linewidth=1.25)
        
        plot!(plt, subplot=1, tv, yv, label =temp_str, color=:deeppink, 
             legendfontsize = leg_size, linewidth=1.5)
        
        plot!(plt, subplot=2, tv, uv, label=power_str,
              legendfontsize = leg_size, fg_legend= "#5fbcd3", color=:royalblue, linewidth=1.5)
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=5.0)
    catch e
        println("Error: ", e)
    end

    set_reference(sys, DEFAULT_REFERENCE); 
    save_experiment([tv, rv, yv, uv], "ThermoLab_profile_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end




# ═══════════════════════════════════════════════════════════════════════════════
#  stepinfo_real
# ═══════════════════════════════════════════════════════════════════════════════


"""
    StepInfoSR

Métricas de una respuesta experimental al escalón, calculadas por
[`stepinfo`](@ref). Los tiempos se miden desde el instante del escalón.

# Campos
- `y0`, `yf`: valores inicial y final de la temperatura (°C).
- `stepsize`: tamaño del escalón de referencia (°C).
- `peak`, `peaktime`: valor máximo de la temperatura (°C) y su instante (s).
- `overshoot`: sobrepico, en % del escalón, respecto a `yf`.
- `lowerpeak`, `undershoot`: valor mínimo de la temperatura (°C) y subpico,
  en % del escalón, respecto a `y0`.
- `settlingtime`: tiempo de estabilización (s).
- `risetime`: tiempo de subida (s).
- `settling_th`, `risetime_th`: umbrales usados para los dos tiempos anteriores.
- `u_max`: máximo de ``|u(t)|`` tras el escalón (%).
"""
struct StepInfoSR
    y0::Float64
    yf::Float64
    stepsize::Float64
    peak::Float64
    peaktime::Float64        # relativo al instante del escalón
    overshoot::Float64
    lowerpeak::Float64
    undershoot::Float64
    settlingtime::Float64    # relativo al instante del escalón
    risetime::Float64
    settling_th::Float64
    risetime_th::Tuple{Float64, Float64}
    u_max::Float64          #  maximo de  la señal de control 
end






"""
    stepinfo(res, T=nothing; settling_th=0.02, risetime_th=(0.1, 0.9))

Calcula las métricas de una respuesta experimental al escalón en lazo cerrado
y grafica la respuesta medida con las métricas anotadas, junto con,
opcionalmente, la respuesta simulada con un modelo de lazo cerrado nominal `T`.

# Argumentos
- `res`: tupla `(t, r, y, u)` con los vectores de tiempo, referencia,
  temperatura medida y potencia aplicada, tal como los retorna
  [`step_closed`](@ref). El instante del escalón se detecta como el mayor
  cambio entre muestras consecutivas de `r`.
- `T=nothing`: función de transferencia de lazo cerrado (de la referencia a la
  temperatura). Si se indica, su respuesta al escalón se superpone a la
  respuesta medida para compararlas.

# Argumentos de palabra clave
- `settling_th::Real=0.02`: semiancho de la banda de estabilización, como
  fracción del tamaño del escalón (por defecto, 2 %).
- `risetime_th::Tuple{Real,Real}=(0.1, 0.9)`: niveles, como fracción del
  tamaño del escalón, entre los cuales se mide el tiempo de subida (por
  defecto, de 10 % a 90 %).

# Retorna
- `info::StepInfoSR`: estructura con las métricas calculadas (ver
  [`StepInfoSR`](@ref)). Al mostrarla se imprime un resumen en consola.

# Notas
- Todos los tiempos se miden desde el instante del escalón.
- El valor inicial `y0` es el promedio de hasta 10 muestras previas al
  escalón y el valor final `yf` el promedio de las últimas 10 muestras.
- Si la respuesta no cruza alguno de los niveles de `risetime_th`, el tiempo
  de subida es `NaN`.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego obtenga la respuesta al escalón y calcule sus métricas:

```julia
result = step_closed(sys; r0=40, r1=50, t0=60, t1=120);
info = stepinfo(result)
```
"""
function stepinfo(res, T = nothing;  settling_th = 0.02, risetime_th = (0.1, 0.9))

    t1, r1, y1, u1 = res[1], res[2], res[3], res[4]

    # ── Detectar instante del escalón en r ────────────────────────────
    step_idx = argmax(abs.(diff(r1))) + 1
    t_step   = t1[step_idx]

    t = t1[step_idx:end] .- t_step
    y = y1[step_idx:end]
    r = r1[step_idx:end]
    u = u1[step_idx:end]

    u_max = maximum(abs.(u))

    # ── Valores inicial y final ───────────────────────────────────────
    #  y0: promedio de hasta 10 puntos ANTES del escalón
    #  yf: promedio de los últimos 10 puntos
    n_pre = min(step_idx - 1, 10)
    y0    = n_pre > 0 ? mean(y1[step_idx - n_pre : step_idx - 1]) : y[1]
    n_end = min(10, length(y))
    yf    = mean(y[end - n_end + 1 : end])

    # ── Tamaño del escalón (desde r) ──────────────────────────────────
    stepsize = r1[end] - r1[1]

    # ── Subpico (undershoot) — solo post-escalón ──────────────────────
    lowerpeak, _ = findmin(y)
    undershoot   = max(100.0 * (y0 - lowerpeak) / stepsize, 0.0)

    # ── Tiempo de estabilización ──────────────────────────────────────
    #  Se recorre la respuesta desde el final: el primer punto fuera de la
    #  banda marca el tiempo de estabilización.
    band    = settling_th * abs(stepsize)
    idx_rev = findfirst(abs.(reverse(y) .- yf) .> band)
    t_rev   = reverse(t)
    settlingtime = idx_rev === nothing ? NaN : t_rev[idx_rev] + SAMPLING_TIME

    # ── Sobrepico (overshoot) — solo post-escalón ─────────────────────
    peak, pidx = findmax(y)
    peaktime   = t[pidx]
    overshoot  = 100.0 * (peak - yf) / stepsize

    # ── Tiempo de subida — interpolación lineal post-escalón ──────────
    lv10 = y0 + risetime_th[1] * stepsize
    lv90 = y0 + risetime_th[2] * stepsize
    i10  = findfirst(y .> lv10)
    i90  = findfirst(y .> lv90)

    if i10 === nothing || i90 === nothing
        risetime = NaN
        t10, t90 = NaN, NaN
    else
        t10      = _interp_cross(t, y, lv10, i10)
        t90      = _interp_cross(t, y, lv90, i90)
        risetime = t90 - t10
    end

    info = StepInfoSR(y0, yf, stepsize, peak, peaktime, overshoot,
                      lowerpeak, undershoot, settlingtime, risetime,
                      settling_th, risetime_th,  u_max)

    _plot_stepinfo(t, r, y, t10, t90, info, T)
    return info
end

# ── Interpolación lineal para cruce preciso ───────────────────────────
function _interp_cross(t, y, level, idx)
    idx == 1 && return t[1]
    α = (level - y[idx-1]) / (y[idx] - y[idx-1])
    return t[idx-1] + α * (t[idx] - t[idx-1])
end

# ── Gráfica con anotaciones ──────────────────────────────────────────
function _plot_stepinfo(t, r, y, t10, t90, si, T)

    leg_size = 10
    p = plot(t, r;
        label = "r(t)", legendfontsize = leg_size, lw = 1.5, color = :green,
        alpha=0.7, background_color=:mintcream, seriestype=:steppost, margin=5Plots.mm)
    plot!([0, 0], [r[end]-si.stepsize, r[end]];
        label = "", lw = 1.5, color = :green,
        alpha=0.7, background_color=:mintcream)

    if  T !== nothing
        tsim = 0:SAMPLING_TIME/10:t[end]
        res = step(T*si.stepsize, tsim)
        plot!(p, res.t, res.y' .+ (r[end]-si.stepsize), label="y(t)  (simulada)",
        legendfontsize = leg_size, color = "#00aad4", alpha=0.75)
    end
    plot!(t, y;
        label  = "y(t) medida (yf = $(@sprintf("%.2f",si.yf)) °C)",
        lw     = 2, color = "#ff2a7f", alpha = 1,
        xlabel = "Tiempo (s)", ylabel = "Temperatura (°C)",
        title  = "Respuesta experimental al Escalón",
        legend = :right, size = (900, 500),
        )

    # Banda de estabilización
    hline!(p, [si.yf + si.settling_th * si.stepsize,
               si.yf - si.settling_th * si.stepsize];
        ls = :dot, color = :orange, alpha = 0.5, label ="" )

    # Pico (sobrepico) — posición absoluta
    scatter!(p, [si.peaktime], [si.peak];
        ms = 5, color = :red, markershape = :circle,
        label = "Pico = $(@sprintf("%.2f",si.peak)) (SP = $(@sprintf("%.2f",si.overshoot)) %)")

    # Tiempo de estabilización — posición absoluta
    if !isnan(si.settlingtime)
        vline!(p, [si.settlingtime];
            ls = :dashdot, color = :orange, lw = 1.25,
            label = "Ts = $(@sprintf("%.1f",si.settlingtime)) s")
    end

    # Niveles 10% y 90% para tiempo de subida
    if !isnan(si.risetime)
        lv10 = si.y0 + si.risetime_th[1] * si.stepsize
        lv90 = si.y0 + si.risetime_th[2] * si.stepsize
        vline!(p, [t10, t90];
            ls = :dashdot, color = :purple, alpha = 0.4, lw=1.25, label = "")
        annotate!(p, t10 + 0.4*si.risetime , lv10, text("10%", 7, :purple))
        annotate!(p, t90 - 0.4*si.risetime, lv90, text("90%", 7, :purple))
    end

    # Leyenda informativa
    plot!(p, Float64[], Float64[];
        label = "Tr = $(@sprintf("%.1f",si.risetime)) s", lw = 1,
        ls = :dashdot, color = :purple, alpha = 0.6)

    redraw!(p)
end

# ── Mostrar resumen en consola ────────────────────────────────────────
function Base.show(io::IO, si::StepInfoSR)
    println(io, "Info Resp. Escalón:")
    @printf(io, "  %-18s %8.3f\n",   "Valor inicial:",    si.y0)
    @printf(io, "  %-18s %8.3f\n",   "Valor final:",      si.yf)
    @printf(io, "  %-18s %8.3f\n",   "Cambio escalón:",   si.stepsize)
    @printf(io, "  %-18s %8.3f\n",   "Pico:",             si.peak)
    @printf(io, "  %-18s %8.3f s\n", "Tiempo pico:",      si.peaktime)
    @printf(io, "  %-18s %8.2f %%\n","Sobrepico:",        si.overshoot)
    @printf(io, "  %-18s %8.3f s\n", "T. estabilización:",    si.settlingtime)
    @printf(io, "  %-18s %8.3f s\n", "T. subida:",        si.risetime)
    @printf(io, "  %-18s %8.3f %%\n", "Max. |u(t)|:",        si.u_max)
end