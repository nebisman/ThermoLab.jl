# ═══════════════════════════════════════════════════════════════════════════════
#  controlsys.jl – Funciones de control para el sistema UNThermal.
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using ControlSystems
using LinearAlgebra
using Plots
using Printf
using Statistics
using RobustAndOptimalControl
using LaTeXStrings




DEFAULT_REF = 40

# ═══════════════════════════════════════════════════════════════════════════════
#  set_reference!
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_reference(sys, ref)

Fija la referencia de temperatura [°C] en el sistema.

# Ejemplo
```julia
set_reference!(sys, 50.0)
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
    set_pid(sys; kp, ki, kd, N, beta)

Configura el controlador PID con filtro derivativo y peso en la referencia.
El algoritmo sigue Åström & Murray (Feedback Systems, 2nd ed.) con
tiempo de muestreo h = 0.8 s.

# Parámetros
- `kp`  : ganancia proporcional (default 16.796)
- `ki`  : ganancia integral (default 2.0)
- `kd`  : ganancia derivativa (default 16.441)
- `N`   : coeficiente de filtro del derivativo (default 27.38)
- `beta`: peso de setpoint, β=1 → PID 1-DOF estándar (default 0.1)

# Ejemplo
```julia
set_pid!(sys; kp=10.0, ki=1.5, kd=8.0, N=20.0, beta=0.5)
```
"""
function set_pid(sys::ThermalSystem;
                  kp::Real   = 16.796,
                  ki::Real   = 2.0,
                  kd::Real   = 16.441,
                  N::Real    = 27.38,
                  beta::Real = 0.1)
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
    set_controller(sys, controller)

Carga un controlador general C(s) en el ESP32.
Soporta controladores 1-DOF (SISO) y 2-DOF (1×2 matriz de transferencia).

El controlador se discretiza con transformada bilineal a h = SAMPLING_TIME.
Se implementa anti-windup por observador (ganancia L calculada con LQR).

# Ejemplo 1-DOF
```julia
s = tf("s")
C = 10.0 + 2.0/s
set_controller!(sys, C)
```

# Ejemplo 2-DOF
```julia
C = tf([[2, 1], [-6, -5]], [[1, 2, 1], [1, 2, 1]])
set_controller!(sys, C)
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
    sleep(0.1)
    set_reference(sys, DEFAULT_REF); 
    println("Controlador cargado en sistema térmico ($ni-DOF, orden=$order)")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  step_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_closed(sys; r0, r1, t0, t1)

Ejecuta un escalón en lazo cerrado: referencia pasa de `r0` a `r1` [°C].
- `t0`: duración en el valor inicial [s] (default 60)
- `t1`: duración en el valor final [s] (default 60)

Grafica en tiempo real y retorna `(t, r, y, u)`.

# Ejemplo
```julia
t, r, y, u = step_closed(sys; r0=40, r1=55, t0=60, t1=120)
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
    display(plt)

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
            legend=[:topleft :topleft],
            grid=true, gridalpha=0.15, margin=5Plots.mm)
        
        
        #legends y plots
        leg_size = 11
        ref_str = latexstring(@sprintf("r(t) = %.2f \\,\\, ^\\circ  C", r_curr))
        temp_str = latexstring(@sprintf("y(t) = %.2f \\,\\, ^\\circ  C", y_curr))
        power_str = latexstring(@sprintf("u(t) = %.2f  \\,\\, \\%% ", u_curr))
        
        plot!(plt, subplot=1, tv, rv, label= ref_str, color=:teal,  seriestype=:steppost,
              legendfontsize = leg_size, fg_legend = "#d4aa00", linewidth=1.25)
        
        plot!(plt, subplot=1, tv, yv, label =temp_str, color=:deeppink, 
             legendfontsize = leg_size, linewidth=1.5)
        
        plot!(plt, subplot=2, tv, uv, label=power_str,
              legendfontsize = leg_size, fg_legend= "#5fbcd3", color=:royalblue, linewidth=1.5)
        
        #annotate!(plt, t90 - 0.4*si.risetime, lv90, text("90%", 7, :purple))
     
        #plot!(t[i10:i90], y[i10:i90], color = "#800033", label ="")
        
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=5.0)
    catch e
        println("Error: ", e)

    end

    _save_exp([tv, rv, yv, uv], "Thermal_step_closed_exp.csv", "t,r,y,u")

    set_reference(sys, DEFAULT_REF);  

    
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  stairs_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    stairs_closed(sys; stairs, duration)

Ejecuta una señal en escalera en lazo cerrado.
- `stairs`  : vector/tupla de temperaturas de referencia [°C]
- `duration`: duración de cada escalón [s] (default 60)

Grafica en tiempo real y retorna `(t, r, y, u)`.

# Ejemplo
```julia
t, r, y, u = stairs_closed(sys; stairs=(40, 50, 55, 45), duration=60)
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
    display(plt)

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
            legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, rv, label="r(t)", color=:darkgreen, linewidth=1.1)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:deeppink,  linewidth=1.25)
        plot!(plt, subplot=2, tv, uv, label="u(t)", color=:royalblue)
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=max(120.0, 3*total*h))
    catch e
        println("Error: ", e)
    finally
        disconnect!(sys)
    end

    _save_exp([tv, rv, yv, uv], "Thermal_stairs_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  profile_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    profile_closed(sys; timevalues, refvalues)

Ejecuta un perfil de referencia interpolado en lazo cerrado.
- `timevalues`: vector de tiempos [s]
- `refvalues` : vector de temperaturas de referencia [°C]

Grafica en tiempo real y retorna `(t, r, y, u)`.

# Ejemplo
```julia
t, r, y, u = profile_closed(sys;
    timevalues = (0, 120, 240, 360, 480),
    refvalues  = (40, 40, 55, 55, 40))
```
"""
function profile_closed(sys::ThermalSystem;
                        timevalues = [0.0, 120.0, 240.0, 360.0],
                        refvalues  = [40.0, 55.0, 55.0, 40.0])
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
        "timevalues" => join(long2hex(t) for t in int_tv),
        "refvalues"  => join(float2hex(v) for v in rv_cmd),
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
    display(plt)

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
            legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, rv, label="r(t)", color=:darkgreen, linewidth=1.25)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:deeppink,  linewidth=1.0)
        plot!(plt, subplot=2, tv, uv, label="u(t)", color=:royalblue, linewidth=1.0)
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=max(120.0, 3*tv_cmd[end]))
    catch e
        println("Error: ", e)
    finally
        disconnect!(sys)
    end

    _save_exp([tv, rv, yv, uv], "Thermal_profile_closed_exp.csv", "t,r,y,u")
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
    stepinfo_real(t, u, y; settling_th=0.02, risetime_th=(0.1, 0.9))

Calcula métricas de respuesta al escalón a partir de datos experimentales reales.
Detecta automáticamente el instante del escalón y mide tiempos desde allí.

# Argumentos
- `t`: vector de tiempos equiespaciados
- `u`: vector de entrada (escalón)
- `y`: vector de salida medida
- `settling_th`: umbral para tiempo de estabilización (default 0.02 = 2%)
- `risetime_th`: umbrales para tiempo de subida (default 10% y 90%)
"""
function stepinfo_exp(res; T = nothing,  settling_th = 0.02, risetime_th = (0.1, 0.9))
    
   # ── Detectar instante del escalón en u ────────────────────────────
   t1, r1, y1, u1 = res  

   step_idx = argmax(abs.(diff(r1))) + 1
    t_step = t1[step_idx]
    
    t = t1[step_idx: end].-t_step
    y = y1[step_idx: end]
    r = r1[step_idx: end]
    u = u1[step_idx: end]
    
    u_max = maximum(abs.(u))
    #n  = length(y)
    Ts = SAMPLING_TIME
    fs = 1.0 / Ts
    
    #Valores iniciales y finales   
    y0    = y[1]    
    yf    = y[end]

    # ── Dirección y magnitud (desde u) ────────────────────────────────
    
    rf = r1[end]
    r0 = r1[1]
    stepsize  = abs(rf- r0)




    # ── Subpico (undershoot) — solo post-escalón ──────────────────────
    lowerpeak, _ =  findmin(y) 
    undershoot   = max(100.0 * (y0 - lowerpeak) / stepsize, 0.0)

    # ── Tiempo de estabilización ──────────────────────────────────────
    #  Butterworth causal orden 4, fc=1 Hz sobre y_post reflejada.
    #  El filtro ve primero el estado estacionario; el primer índice
    #  donde la salida se sale de la banda marca el settling time.
    

    y_rev =  reverse(y) 
    band           = settling_th * stepsize
    
    idx_rev        = findfirst(abs.(y_rev .- yf) .> band)
    t_rev    = reverse(t)
     

    # ── Sobrepico (overshoot) — solo post-escalón ─────────────────────
    peak, pidx = findmax(y) 
    peaktime   = t[pidx] 
    overshoot  = 100.0 * (peak - yf) / stepsize

    if idx_rev === nothing
        settlingtime = NaN
    else
        settlingtime = t_rev[idx_rev] + SAMPLING_TIME
    end
 

    # ── Tiempo de subida — interpolación suave post-escalón ───────────

    
   

    lv10 = y0 + risetime_th[1] * stepsize 
    lv90 = y0 + risetime_th[2] * stepsize     
    i10  = findfirst(y .> lv10)
    i90  = findfirst(y .> lv90)

    if i10 === nothing || i90 === nothing
        risetime = NaN
        t10 = NaN
        t90 = NaN   
    else
        t10      = _interp_cross(t, y, lv10, i10)
        t90      = _interp_cross(t, y, lv90, i90)
        risetime = t90- t10
    end

    info = StepInfoSR(y0, yf, stepsize, peak, peaktime, overshoot,
                      lowerpeak, undershoot, settlingtime, risetime,
                      settling_th, risetime_th,  u_max)

    _plot_stepinfo(t, r, y, t10, t90, i10, i90,  info, T)
    return info
end

# ── Interpolación lineal para cruce preciso ───────────────────────────
function _interp_cross(t, y, level, idx)
    idx == 1 && return t[1]
    α = (level - y[idx-1]) / (y[idx] - y[idx-1])
    return t[idx-1] + α * (t[idx] - t[idx-1])
end

# ── Gráfica con anotaciones ──────────────────────────────────────────
function _plot_stepinfo(t, r, y, t10, t90, i10, i90,  si, T)    
    

    p =    plot( t, r;
        label = "r(t)", lw = 1.5, color = :green, alpha=0.7, seriestype=:steppost, background_color=:mintcream)

    if  T !== nothing
        ref0 = si.stepsize
        tsim = 0: 1/20: t[end]
        res = step(T*ref0, tsim)
        plot!(p, res.t, res.y'.+ (r[1]-si.stepsize),  color = "#00aad4", alpha=0.5)
    end      
    plot!(t, y;
        label  = "y(t) medida (yf = $(@sprintf("%.2f",si.yf)))",
        lw     = 2, color = :navy, alpha = 1,
        xlabel = "Tiempo (s)", ylabel = "Amplitud",
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
        label = "Pico = $(@sprintf("%.2f",si.peak)) (SP = $(@sprintf("%.2f",si.overshoot)))")

    # Tiempo de estabilización — posición absoluta
    vline!(p, [si.settlingtime];
        ls = :dashdot, color = :orange, lw = 1.25,
        label = "Ts = $(@sprintf("%.2f",si.settlingtime)) s")

    # Niveles 10% y 90% para tiempo de subida
    if !isnan(si.risetime)
        lv10 = si.y0 + si.risetime_th[1] * si.stepsize
        lv90 = si.y0 + si.risetime_th[2] * si.stepsize
        vline!(p, [t10, t90];
            ls = :dashdot, color = :purple, alpha = 0.4, lw=1.25, label = "")
        annotate!(p, t10 + 0.4*si.risetime , lv10, text("10%", 7, :purple))
        annotate!(p, t90 - 0.4*si.risetime, lv90, text("90%", 7, :purple))
        #plot!(t[i10:i90], y[i10:i90], color = "#800033", label ="")
    end

    # Leyenda informativa
    plot!(p, Float64[], Float64[];
        label = "Tr = $(@sprintf("%.2f",si.risetime)) s", lw = 1,
        ls = :dashdot, color = :purple, alpha = 0.6)
        
             

    display(p)
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
    @printf(io, "  %-18s %8.3f V\n", "Max. |u(t)|:",        si.u_max)

end



