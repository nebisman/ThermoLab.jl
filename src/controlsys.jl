# ═══════════════════════════════════════════════════════════════════════════════
#  controlsys.jl – Funciones de control para el sistema ThermoLab via serial.
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




# ═══════════════════════════════════════════════════════════════════════════════
#  set_reference
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_reference(sys::ThermalSystem, ref_value::Real=40.0)

Fija la referencia de temperatura de la plataforma ThermoLab al valor `ref_value` (en °C).
El controlador activo (ver [`set_pid`](@ref) y [`set_controller`](@ref)) lleva la
temperatura a ese valor.

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.
- `ref_value::Real=40.0`: temperatura de referencia, en °C. No debe superar
  los 90 °C.

# Retorna
- `nothing`. Como efecto secundario, imprime en consola la referencia fijada.

# Lanza
- `ErrorException` si `ref_value` es mayor que 90 °C.

# Ejemplo
```julia
using ThermoLab
sys = ThermalSystem();
set_reference(sys, 50.0)

    "Referencia fijada a 50.00 °C"
```
"""
function set_reference(sys::ThermalSystem, ref_value::Real = DEFAULT_REFERENCE)
    ref_value <= MAX_TEMPERATURE ||
        error("La referencia no debe superar $(MAX_TEMPERATURE) °C")
    connect!(sys)
    send_command!(sys, "set_ref", Dict("reference" => float2hex(ref_value)))
    disconnect!(sys)
    println("Referencia fijada a $(@sprintf("%.2f", ref_value)) °C \n")
    return nothing
end



# ═══════════════════════════════════════════════════════════════════════════════
#  set_pid
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_pid(sys::ThermalSystem; kp=16.796, ki=2.0, kd=16.441, N=27.38, beta=0.1)

Esta función configura los cinco parámetros `kp`, `ki`, `kd`, `N` y `beta` de
un controlador PID de 2-GDL (dos grados de libertad), como el mostrado en la siguiente figura:

![Diagrama de bloques del controlador PID de dos grados de libertad](../../assets/pid2gdl.png)

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma ThermoLab.

# Argumentos de palabra clave
- `kp::Real=16.796`: ganancia proporcional del controlador PID.
- `ki::Real=2.0`: ganancia integral del controlador PID.
- `kd::Real=16.441`: ganancia derivativa del controlador PID.
- `N::Real=27.38`: coeficiente del filtro del derivador.
- `beta::Real=0.1`: ponderación del punto de ajuste (*setpoint weighting*)
  del controlador PID.

# Retorna
- `nothing`, una vez que los parámetros del PID se han actualizado
  correctamente. Imprime en consola los parámetros configurados.

# Notas
- Cuando `beta=1.0`, el PID resultante corresponde al PID estándar de un grado
  de libertad (1-DOF) descrito en la mayoría de los libros de texto.
- Esta implementación se basa en el algoritmo formulado en el libro [*Feedback
  Systems: An Introduction for Scientists and Engineers*](https://fbswiki.org/wiki/index.php/PID_Control),
  segunda edición, de Åström y Murray. El algoritmo usa un periodo de muestreo
  del controlador de 0.8 segundos, apropiado para la plataforma ThermoLab, e
  incluye un mecanismo *anti-windup* para la acción integral.
- La potencia aplicada se satura entre 0 y 100 %.
- Al cargar el controlador, la referencia vuelve al valor por defecto de 40 °C.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab y de haber definido
el sistema, así:

```julia
using ThermoLab
sys = ThermalSystem();
```

Luego, el controlador PID se programa así:

```julia
set_pid(sys; kp=11.8, ki=2.7, kd=0, N=10.0, beta=0.0)

    "PID actualizado: kp=11.8  ki=2.7  kd=0  N=10.0  β=0.0"
```
"""
function set_pid(sys::ThermalSystem;
                  kp::Real = 16.796, ki::Real = 2.0, kd::Real = 16.441,
                  N::Real = 27.38, beta::Real = 0.1)

    payload = Dict(
        "kp"   => float2hex(kp),
        "ki"   => float2hex(ki),
        "kd"   => float2hex(kd),
        "N"    => float2hex(N),
        "beta" => float2hex(beta),
    )
    connect!(sys)
    send_command!(sys, "set_pid", payload)
    set_reference(sys, DEFAULT_REFERENCE)
    println("PID actualizado: kp=$kp  ki=$ki  kd=$kd  N=$N  β=$beta")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  set_controller  (controlador general en espacio de estados)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_controller(sys::ThermalSystem, controller)

Programa un controlador en la plataforma ThermoLab. El controlador se define como una función de transferencia en el dominio de
Laplace `s`, la cual es automáticamente discretizada.

Esta función permite programar un controlador estándar de 1 grado de
libertad (1-GDL), como el mostrado en la siguiente figura:

![Controlador de 1 grado de libertad](../../assets/controller_1gdl.png)

A su vez, esta misma función permite programar un controlador de 2
grados de libertad, como se muestra a continuación:

![Controlador de 2 grados de libertad](../../assets/controller_2gdl.png)

# Argumentos
- `sys::ThermalSystem`:  objeto que representa la plataforma ThermoLab.
- `controller`: función de transferencia continua (`ControlSystems.TransferFunction`)
  en el dominio de Laplace `s`.
  - Si ``C`` es 1x1 (una entrada y una salida), se interpreta como controlador de 1 grado de
    libertad:

    ``u = C·(r - y)``

  - Si  ``C=[C_1\\quad C_2]`` es una matriz 1×2 (dos entradas y una salida), se interpreta como controlador de 2 grados de
    libertad:

    ``u = C_1·r + C_2·y``

    Con el controlador obtenido por [`cont2dof`](@ref), ``C_1=\\frac{L}{A}`` es la
    primera columna y ``C_2=-\\frac{M}{A}`` es la segunda.

# Retorna
- `nothing`. Imprime un mensaje de confirmación en consola.

# Notas
- Esta función admite tanto controladores de 1-GDL como de 2-GDL, de orden
  máximo 10.
- El controlador se convierte a espacio de estados para su implementación
  digital. Se discretiza usando el método bilineal (Tustin) con un periodo
  de muestreo de 0.8 segundos.
- Se implementa un mecanismo anti-windup basado en un observador, cuya
  ganancia se calcula mediante el filtro de Kalman.
- La potencia aplicada se satura entre 0 y 100 %.

# Ejemplos
## Ejemplo 1: controlador de  un grado de libertad (1-GDL)
Supongamos que queremos implementar un controlador PI para la temperatura de la plataforma
**ThermoLab**, definido por la siguiente función de transferencia:

``C(s)=\\frac{11.8\\,s + 2.7}{s}``

Para ello usamos el siguiente código:

```julia
using ThermoLab
sys = ThermalSystem();
s = tf("s");
C = (11.8s + 2.7)/s;
set_controller(sys, C)

    "Controlador cargado en ThermoLab"
```

## Ejemplo 2: controlador de dos grados de libertad (2-GDL)
Supongamos que queremos implementar un controlador de 2-GDL diseñado con [`cont2dof`](@ref)
para que la temperatura siga una respuesta de lazo cerrado de segundo orden `T`:

```julia
using ThermoLab
sys = ThermalSystem();
G = tf(sys)
s = tf("s");
ωn = 0.1; ζ = 0.8
T = ωn^2/(s^2 + 2ζ*ωn*s + ωn^2)
C = cont2dof(G, T, 1, [-0.3])
set_controller(sys, C);

    "Controlador cargado en ThermoLab"
```
"""
function set_controller(sys::ThermalSystem, controller)
    # Determinar si es 1 DOF o 2 DOF
    ni = size(controller, 2)  # número de entradas
    ni in (1, 2) || error("El controlador debe ser de 1 o 2 GDL (1×1 o 1×2)")
    type_control = ni - 1     # 0: controlador de 1-GDL, 1: de 2-GDL

    # # ── Construir sistema continuo con 2 entradas [r, y] ────────────

    sys_c = minreal(ss(controller))

    # ── Discretización bilineal (Tustin) ──────────────────────────────
    sys_int = c2d(sys_c, SAMPLING_TIME, :tustin)
    sys_d, S, E = modal_form(sys_int)
    Ad, Bd, Cd, Dd = ssdata(sys_d)
    order = size(Ad, 1)
    order <= 10 || error("El orden del controlador no debe superar 10")
    Q = 100000*diagm(ones(size(Ad,1)))


    Lg = kalman(sys_d, Q,1)

    if ni == 1
       Bd =Matrix{Float64}( hcat(Bd, -Bd))
       Dd =Matrix{Float64}( hcat(Dd, -Dd))
    end


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
    sleep(0.1)
    set_reference(sys, DEFAULT_REFERENCE)
    println("Controlador cargado en ThermoLab ($ni-GDL, orden=$order)")
    return nothing
end



# ═══════════════════════════════════════════════════════════════════════════════
#  Gráfica de lazo cerrado (compartida por step, stairs y profile)
# ═══════════════════════════════════════════════════════════════════════════════

"""Figura base de dos paneles (temperatura y potencia) para lazo cerrado."""
function _closed_loop_plot(title_str, xmax, ylim)
    return plot(layout=(2, 1), size=(900, 500),
        title=[title_str ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["" "Tiempo (s)"],
        xlims=[(0, xmax) (0, xmax)],
        ylims=[ylim (-5, 105)],
        background_color_subplot=[:ivory :mintcream],
        legend=[:topleft :topleft],
        grid=true, gridalpha=0.15, margin=5Plots.mm)
end

"""Agrega a la figura las curvas r(t), y(t) y u(t) con sus valores actuales."""
function _closed_loop_curves!(plt, tv, rv, yv, uv)
    leg_size = 10
    ref_str   = latexstring(@sprintf("r(t) = %.2f \\,\\, ^\\circ C", rv[end]))
    temp_str  = latexstring(@sprintf("y(t) = %.2f \\,\\, ^\\circ C", yv[end]))
    power_str = latexstring(@sprintf("u(t) = %.2f \\,\\, \\%%", uv[end]))
    plot!(plt, subplot=1, tv, rv, label=ref_str, color=:teal,
          seriestype=:steppost, legendfontsize=leg_size, fg_legend="#d4aa00",
          linewidth=1.25)
    plot!(plt, subplot=1, tv, yv, label=temp_str, color=:deeppink,
          legendfontsize=leg_size, linewidth=1.5)
    plot!(plt, subplot=2, tv, uv, label=power_str, color=:royalblue,
          legendfontsize=leg_size, fg_legend="#5fbcd3", linewidth=1.5)
    return plt
end

"""Recibe las muestras de un experimento en lazo cerrado y las grafica."""
function _run_closed_loop!(sys, total, title_str, xmax, ylim)
    h = SAMPLING_TIME
    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]

    on_sample = function(msg, np_val)
        push!(tv, np_val * h)
        push!(rv, hex2float(string(msg["r"])))
        push!(yv, hex2float(string(msg["y"])))
        push!(uv, hex2float(string(msg["u"])))

        # Reconstruir gráfica desde cero para evitar trazas duplicadas
        plt = _closed_loop_plot(title_str, xmax, ylim)
        _closed_loop_curves!(plt, tv, rv, yv, uv)
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample)
    catch e
        println("Error: ", e)
    end
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  step_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_closed(sys::ThermalSystem; r0=40, r1=50, t0=60, t1=60)

Ejecuta un experimento de respuesta al escalón en lazo cerrado sobre la
plataforma ThermoLab. El controlador que actúa sobre la plataforma ha sido previamente cargado con
[`set_pid`](@ref) o [`set_controller`](@ref).

La referencia de temperatura inicia en un valor inicial `r0` y
pasa a un valor final `r1`. La duración de cada nivel de referencia está
definida por los tiempos `t0` y `t1`, respectivamente, como muestra la figura siguiente.

![Respuesta al escalón en lazo cerrado de la plataforma ThermoLab](../../assets/step_closed.png)

Los parámetros de la figura se describen a continuación.

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma ThermoLab, con un controlador previamente
cargado.

# Argumentos de palabra clave
- `r0::Real=40`: valor inicial de la referencia, en °C.
- `r1::Real=50`: valor final de la referencia tras el escalón, en °C.
- `t0::Real=60`: duración en segundos de la referencia en el valor `r0`.
- `t1::Real=60`: duración en segundos de la referencia en el valor `r1`.

# Retorna
- `(t, r, y, u)`: tupla de vectores `Vector{Float64}` con el tiempo [s], la
  referencia [°C], la temperatura medida [°C] y la potencia aplicada [%], respectivamente.

# Notas
- Para obtener una respuesta al escalón limpia, la temperatura debe estar
  estabilizada en `r0` antes del escalón; elija `t0` suficientemente largo.
- La función genera una gráfica para visualizar la respuesta al escalón en
  tiempo real, mostrando la temperatura y la potencia durante el
  experimento.
- Al terminar, la referencia vuelve al valor por defecto de 40 °C.
- Los datos del experimento se guardan en un archivo CSV en
  `datafiles/ThermoLab_step_closed_exp.csv` presente en el directorio de trabajo.

# Ejemplos
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem()
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego, obtenga el experimento de respuesta al escalón en lazo cerrado, así:

```julia
t, r, y, u = step_closed(sys; r0=40, r1=50, t0=60, t1=120);
```
"""
function step_closed(sys::ThermalSystem;
                     r0::Real = 40.0, r1::Real = 50.0,
                     t0::Real = 60.0, t1::Real = 60.0)
    max(r0, r1) <= MAX_TEMPERATURE ||
        error("Las referencias no deben superar $(MAX_TEMPERATURE) °C")
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

    # Gráfica inicial
    delta = abs(r1 - r0)
    title_str = "Escalón en lazo cerrado  r0=$(@sprintf("%.1f",r0)) → r1=$(@sprintf("%.1f",r1)) °C"
    xmax = (total - 1) * h
    ylim = (min(r0, r1) - max(5, 0.5delta), max(r0, r1) + max(5, 0.5delta))
    redraw!(_closed_loop_plot(title_str, xmax, ylim))

    # Comunicación
    connect!(sys)
    send_command!(sys, "step_closed", payload)
    tv, rv, yv, uv = _run_closed_loop!(sys, total, title_str, xmax, ylim)
    set_reference(sys, DEFAULT_REFERENCE)

    save_experiment([tv, rv, yv, uv], "ThermoLab_step_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  stairs_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    stairs_closed(sys; stairs=[40, 50, 60], duration=120)

Ejecuta  un experimento de seguimiento de una señal de referencia tipo "escalera" sobre la
plataforma ThermoLab. La referencia de temperatura toma sucesivamente cada uno de los
valores de los escalones definidos por el usuario (en `stairs`), con una duración establecida en la variable `duration`,
como se muestra en la figura siguiente:


![Señal de referencia tipo escalera para stairs_closed](../../assets/stairs_response.png)

Los parámetros mostrados en la figura se describen a continuación:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `stairs`: colección con los escalones sucesivos de temperatura de referencia
  deseada, en °C (máximo 50 escalones).
- `duration::Real=120`: duración en segundos de cada escalón de la escalera.

# Retorna
- `(t, r, y, u)`: tupla de vectores con el tiempo (s), la
  referencia (°C), la temperatura medida (°C) y la potencia aplicada (%).

# Notas
- El controlador que actúa sobre la plataforma en este experimento ha sido previamente cargado con
[`set_pid`](@ref) o [`set_controller`](@ref).
- Se genera una gráfica para visualizar la respuesta a la señal de
  referencia en tiempo real, mostrando la temperatura y la potencia
  durante el experimento.
- Los datos del experimento se guardan en un archivo CSV en
  `datafiles/ThermoLab_stairs_closed_exp.csv` que está presente en el
  directorio de trabajo.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego, ejecute el experimento de obtención de la respuesta en lazo cerrado a una señal de referencia tipo escalera, así:

```julia
stairs_closed(sys; stairs=[40, 50, 60, 45], duration=120)
```
"""
function stairs_closed(sys::ThermalSystem;
                       stairs = [40, 50, 60],
                       duration::Real = 120.0)
    h        = SAMPLING_TIME
    stairs_v = collect(Float64, stairs)
    length(stairs_v) <= 50 || error("stairs admite a lo sumo 50 escalones")
    dur_pts  = ceil(Int, duration / h)
    total    = length(stairs_v) * dur_pts
    mn, mx   = minimum(stairs_v), maximum(stairs_v)
    mx <= MAX_TEMPERATURE ||
        error("Las referencias no deben superar $(MAX_TEMPERATURE) °C")

    payload = Dict(
        "signal"        => signal2hex(stairs_v),
        "duration"      => long2hex(dur_pts),
        "points_stairs" => long2hex(length(stairs_v)),
        "min_val"       => float2hex(mn),
        "max_val"       => float2hex(mx),
    )

    span = mx - mn
    title_str = "Escaleras con $(length(stairs_v)) niveles, duración $(@sprintf("%.0f",total*h)) s"
    xmax = (total - 1) * h
    ylim = (mn - max(3, 0.1span), mx + max(3, 0.1span))
    redraw!(_closed_loop_plot(title_str, xmax, ylim))

    connect!(sys)
    send_command!(sys, "stairs_closed", payload)
    tv, rv, yv, uv = _run_closed_loop!(sys, total, title_str, xmax, ylim)
    set_reference(sys, DEFAULT_REFERENCE)

    save_experiment([tv, rv, yv, uv], "ThermoLab_stairs_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  profile_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    profile_closed(sys; timevalues=[0, 120, 240, 360], refvalues=[40, 55, 55, 40])

Ejecuta en lazo cerrado un perfil de referencia arbitrario sobre la
plataforma ThermoLab. La señal de referencia se define como una serie de
instantes de tiempo `t1`, `t2`,...`ti`, a los cuales corresponden, respectivamente,
los valores de temperatura de referencia `r1`, `r2`,...`ri`. Entre dos instantes
consecutivos la referencia se interpola linealmente, como se ilustra a continuación:


![Perfil de referencia definido por puntos para profile_closed](../../assets/profile_response.png)

Los parámetros de la función son los siguientes:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `timevalues`: colección con los instantes de tiempo (con valores crecientes
  en segundos) que definen el perfil. El primer punto de `timevalues` debe ser `0`.
- `refvalues`: colección, de la misma longitud que `timevalues`, con los valores
  de temperatura de referencia (°C) en cada instante.

# Retorna
- `(t, r, y, u)`: tupla de vectores con el tiempo (s), la
  referencia interpolada (°C), la temperatura medida (°C) y la potencia aplicada (%).

# Notas
- El controlador que actúa sobre la plataforma en este experimento ha sido previamente cargado con
[`set_pid`](@ref) o [`set_controller`](@ref).
- Se genera una gráfica para visualizar la respuesta al perfil de referencia en tiempo
  real, mostrando la temperatura y la potencia durante el experimento.
- Los datos del experimento se guardan en un archivo CSV en
  `datafiles/ThermoLab_profile_closed_exp.csv` que está presente en el directorio de trabajo.


# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego, ejecute el experimento de obtención de la respuesta en lazo cerrado a un perfil de referencia, así:

```julia
profile_closed(sys; timevalues=[0, 120, 240, 360, 480], refvalues=[40, 40, 55, 55, 40])
```
"""
function profile_closed(sys::ThermalSystem;
                        timevalues = [0, 120, 240, 360],
                        refvalues  = [40, 55, 55, 40])
    h      = SAMPLING_TIME
    tv_cmd = collect(Float64, timevalues)
    rv_cmd = collect(Float64, refvalues)
    length(tv_cmd) == length(rv_cmd) ||
        error("timevalues y refvalues deben tener la misma longitud")
    maximum(rv_cmd) <= MAX_TEMPERATURE ||
        error("Las referencias no deben superar $(MAX_TEMPERATURE) °C")

    int_tv = [round(Int, p / h) for p in tv_cmd]
    if int_tv[1] != 0
        int_tv = vcat([0, int_tv[1] - 1], int_tv)
        rv_cmd = vcat([rv_cmd[1], rv_cmd[1]], rv_cmd)
    end
    length(int_tv) <= 50 || error("El perfil admite a lo sumo 50 puntos")

    mn, mx = minimum(rv_cmd), maximum(rv_cmd)
    total  = int_tv[end] + 1

    payload = Dict(
        "timevalues" => time2hex(int_tv),
        "refvalues"  => signal2hex(rv_cmd),
        "points"     => long2hex(length(int_tv)),
        "min_val"    => float2hex(mn),
        "max_val"    => float2hex(mx),
    )

    span = mx - mn
    title_str = "Perfil duración=$(@sprintf("%.0f",tv_cmd[end])) s, $(length(tv_cmd)) puntos"
    xmax = (total - 1) * h
    ylim = (mn - max(3, 0.1span), mx + max(3, 0.1span))
    redraw!(_closed_loop_plot(title_str, xmax, ylim))

    connect!(sys)
    send_command!(sys, "prof_closed", payload)
    tv, rv, yv, uv = _run_closed_loop!(sys, total, title_str, xmax, ylim)
    set_reference(sys, DEFAULT_REFERENCE)

    save_experiment([tv, rv, yv, uv], "ThermoLab_profile_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  stepinfo_real
# ═══════════════════════════════════════════════════════════════════════════════


"""
Estructura con los resultados de stepinfo para datos experimentales reales.
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

Calcula métricas de la respuesta al escalón a partir de datos experimentales
en lazo cerrado, y grafica la respuesta medida (con las métricas anotadas)
junto con, opcionalmente, la respuesta simulada con un modelo de lazo cerrado nominal `T`.


# Argumentos
- `res`: tupla `(t, r, y, u)` con los vectores de tiempo, referencia, temperatura
  medida y potencia aplicada, tal y como los retorna [`step_closed`](@ref).

# Argumentos de palabra clave
- `T=nothing`: función de transferencia del modelo de lazo cerrado; cuando se suministra, su
  respuesta al escalón se superpone en la gráfica experimental para comparar.
- `settling_th::Real=0.02`: umbral, como fracción del tamaño del escalón,
  de la banda usada para el cálculo del tiempo de estabilización (por defecto, 2 %).
- `risetime_th::Tuple{Real,Real}=(0.1, 0.9)`: niveles, como fracción del
  tamaño del escalón, usados para el tiempo de subida (por defecto, entre
  10 % y 90 %).

# Retorna
- `info::StepInfoSR`: estructura con las métricas calculadas: valores
  inicial y final (`y0`, `yf`), tamaño del escalón (`stepsize`), pico y
  tiempo de pico (`peak`, `peaktime`), sobrepico en % (`overshoot`),
  subpico y subpico en % (`lowerpeak`, `undershoot`), tiempo de
  estabilización (`settlingtime`), tiempo de subida (`risetime`), los
  umbrales usados (`settling_th`, `risetime_th`) y el máximo de la potencia
  aplicada (`u_max`).

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber cargado un controlador, así:

```julia
using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
```

Luego obtenga la respuesta al escalón y use stepinfo

```julia
result = step_closed(sys; r0=40, r1=50, t0=60, t1=120);
info = stepinfo(result)

    Info Resp. Escalón:
      Valor inicial:       40.013
      Valor final:         49.981
      Cambio escalón:      10.000
      Pico:                50.312
      Tiempo pico:          6.400 s
      Sobrepico:             3.31 %
      T. estabilización:   28.000 s
      T. subida:            3.188 s
      Max. |u(t)|:         94.795 %
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
