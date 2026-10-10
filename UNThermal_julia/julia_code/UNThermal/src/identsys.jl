# ═══════════════════════════════════════════════════════════════════════════════
#  identsys.jl – Identificación del sistema UNThermal.
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using ControlSystems
using Plots
using Printf
using Statistics
using DSP
using ControlSystemIdentification
using LinearAlgebra
using LaTeXStrings

const PRBS_LENGTH   = 1023
# ═══════════════════════════════════════════════════════════════════════════════
#  get_static_model
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_static_model(sys; step=5.0, usefile=false)

Obtiene la curva estática del sistema térmico: temperatura [°C] vs potencia [%].
Recorre setpoints de 40 °C a 85 °C en pasos de `step`.
Grafica en tiempo real y retorna `(power_values, temperatures)`.

# Ejemplo
```julia
uee, yee = get_static_model(sys; step=5.0)
```
"""
function get_static_model(sys::ThermalSystem;
                          step::Real    = 5.0,
                          usefile::Bool = false)
    if usefile
        uss, yss = read_csv_file(PATH_DATA * "Thermal_static_gain_response.csv")
    else

        uss      = Float64[]
        yss      = Float64[]
        y_targets = collect(40.0:step:90.0) 
    
        for yi in y_targets        
            uee_curr, yee_curr = _step_static(sys, yi , uss, yss; t1=40)
            push!(uss, uee_curr)
            push!(yss, yee_curr)       
            
        end    
    end


    
     # ── Regresión lineal: yss = K*uss + b ──────────────────────────────────
    A      = hcat(uss, ones(length(uss)))   # matriz de diseño [u 1]
    coeffs = A \ yss                        # mínimos cuadrados
    K, b   = coeffs[1], coeffs[2]

    u_fit  = range(minimum(uss), maximum(uss), length=100)
    y_fit  = K .* u_fit .+ b

    R²     = 1 - sum((yss .- (K .* uss .+ b)).^2) /
                     sum((yss .- mean(yss)).^2)

    println("Modelo lineal:  y = $(round(K,digits=4))·u + $(round(b,digits=4))")
    println("R² = $(round(R², digits=5))")
    # ───────────────────────────────────────────────────────────────────────



    # Datos experimentales
    datos = scatter(uss, yss,
        label="Datos experimentales",
        color="#d40055", marker=:circle, markersize=4, linewidth=1.5)

    # Modelo lineal
    mod_str = @sprintf("%.2f uee + %0.2f  (R² = %0.2f)", K, b, R²)
    plot!(datos, collect(u_fit), y_fit,
        label = mod_str,
        color="#0055d4", linewidth=2, linestyle=:dash)

    plt = plot(datos,  size=(600,600), 
        title="Curva estática – UNThermal",
        xlabel="Potencia (% de 2.475 W)",
        ylabel="Temperatura estacionaria (°C)",
        xlims=(0, 60), ylims=(30, 100),
        background_color_subplot=:ivory,
        legend=:topleft, grid=true, gridalpha=0.2,
        xticks=0:10:100, yticks=20:10:100,
        margin=5Plots.mm)
    display(plt)
    exp_data = hcat(uss, yss)
    open(PATH_DATA * "Thermal_static_gain_response.csv", "w") do io
        println(io, "u,y")
        for i in axes(exp_data, 1)
            @printf(io, "%.8f,%.8f\n", exp_data[i, 1], exp_data[i, 2])
        end
    end
   
    println("Curva estática completada con $(length(uss)) puntos")   
    sleep(0.1)
    set_reference(sys, DEFAULT_REF);   
    disconnect!(sys)
    sleep(0.25)   
    return uss, yss

end

# Auxiliar: escalón cerrado para obtener un punto estacionario
function _step_static(sys::ThermalSystem, r_ref::Real, uee, yee; t1::Real = 60.0)
    
    h       = SAMPLING_TIME
    pts     = round(Int, t1 / h) + 1
    payload = Dict(
        "low_val"     => float2hex(r_ref),
        "high_val"    => float2hex(r_ref),        
        "points_high" => long2hex(pts),
        "points_low"  => long2hex(0),
    )
    
    yv = Float64[]
    uv = Float64[]
    tv = Float64[]

    leg_size = 8


    connect!(sys)
    send_command!(sys, "step_closed", payload)
    on_sample = function(msg, np_val)
        t_curr = (np_val) * h
        y_curr = hex2float(string(msg["y"]))
        u_curr = hex2float(string(msg["u"]))   
     
        push!(tv, t_curr); push!(yv, y_curr); push!(uv, u_curr);         
        


            y_str = @sprintf("y(t) = %.2f °C", y_curr)
            u_str = @sprintf("u(t) = %.2f °C", u_curr)
            

            if isempty(yee)
                ee_str=""
            else
                ee_str = @sprintf("uee = %.2f %%    yee=%0.2f °C", uee[end], yee[end])
            end
            
            
            p_ee = scatter(uee, yee, color="#d40055", marker=:circle,label = ee_str,
            legendfontsize = leg_size, fg_legend = "#d4aa00", markersize=4, linewidth=1.5)
                 

            p_ins = plot(tv, yv, label=y_str, color="#00aad4",
             legendfontsize = leg_size, fg_legend= "#5fbcd3", linewidth=1.5)

           
            plot!(p_ins, tv, uv, label=u_str, color="#44aa00", linewidth=1.5)
          
            
            plt = plot(p_ee, p_ins, layout=(1, 2), size=(900, 500),
            title=["Curva estática – UNThermal" "Experimento actual"],
            xlabel=["Potencia (%)" "Tiempo (s)"],
            ylabel=["Temperatura estacionaria (°C)" "Temp. (°C) y potencia (%) actual"], 
            xlims=[(0, 60) (0,t1)],
            ylims=[(20, 100) (0, 100)],
            background_color_subplot=[:ivory :mintcream],
            legend=[:bottomright :topleft], 
            grid=true, gridalpha=0.2,
            xticks=0:10:100, yticks=0:10:100,
            margin=5Plots.mm)
            redraw!(plt)

       
    end
    try
        receive_samples!(sys, pts, on_sample; timeout=5.0)
    catch e
        println("Error en punto estático $(r_ref) °C: $e")
   
    end

    n_ee =length(yv) - 10
    uee_curr = mean(uv[n_ee:end])
    yee_curr = mean(yv[n_ee:end])
    return uee_curr, yee_curr
end


# ═══════════════════════════════════════════════════════════════════════════════
#  step_open
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_open(sys; yop, amplitude, t1, stab_time, uee_time, usefile)

Realiza un experimento de escalón en lazo abierto.
El firmware primero estabiliza en `yop` °C usando el PI presintonizado,
luego aplica un escalón de amplitud `amplitude` [%] en la potencia.

# Parámetros
- `yop`       : punto de operaGplr, Gn = plr(d,na,nb,nc, initial_order=20) ción [°C] (default 50)
- `amplitude` : amplitud del escalón en potencia [%] (default 10)
- `t1`        : duración del experimento tras el escalón [s] (default 360)
- `stab_time` : tiempo de estabilización en yop [s] (default 60)
- `uee_time`  : tiempo de retención antes del escalón [s] (default 10)

Retorna `(t, u, y)` con t=0 en el instante del escalón.

# Ejemplo
```julia
t, u, y = step_open(sys; yop=50, amplitude=10, t1=360)
```
"""
function step_open(sys::ThermalSystem;
                   yop::Real       = 50.0,
                   amplitude::Real = 10.0,
                   t1::Real        = 360.0,
                   stab_time::Real = 50.0,
                   uee_time::Real  = 10.0,
                   usefile::Bool   = false)
    if usefile
        return read_csv_file3(PATH_DATA * "Thermal_step_open_exp.csv")
    end

    h        = SAMPLING_TIME
    stab_pts = round(Int, stab_time / h)
    uee_pts  = round(Int, uee_time / h)
    step_pts = round(Int, t1 / h)
    total    = stab_pts + uee_pts + step_pts
    t_total  = stab_time + uee_time + t1


    m = 1.2341015052212259
    b = 24.750915901094388
    uf_est = (yop - b) / m
    percent = 0.2

    ymax = yop + m* amplitude + 5
    ymin = yop - 5
    umax = uf_est + (1 + percent) * amplitude
    umin =  minimum([0, uf_est - (1 + percent) * amplitude])


    payload = Dict(
        "amplitude" => float2hex(amplitude),
        "op_point"       => float2hex(yop),       
        "stab_points" => long2hex(stab_pts),
        "uee_points"  => long2hex(uee_pts),
        "points_high" => long2hex(step_pts),
    )

    tv_stab, uv_stab, yv_stab = Float64[], Float64[], Float64[]
    tv_step, uv_step, yv_step = Float64[], Float64[], Float64[]
    tstep = h * (stab_pts + uee_pts + 1)

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Escalón en lazo abierto - yop=$(@sprintf("%.1f",yop)) °C" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["" "Tiempo (s)"],
        xlims=[(0, t_total) (0, t_total)],
        ylims=[(ymin, ymax) (umin, umax)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)    
    send_command!(sys, "step_open", payload)
     
    on_sample = function(msg, np_val)

        t_curr = np_val * h        
        y_curr = hex2float(string(msg["y"]))
        u_curr = hex2float(string(msg["u"]))

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Escalón en lazo abierto - yop=$(@sprintf("%.1f",yop)) °C" ""],
            ylabel=["Temperatura (°C)" "Potencia (%)"],
            xlabel=["" "Tiempo (s)"],
            xlims=[(0, t_total) (0, t_total)],
            ylims=[(ymin, ymax) (0, 100)],
            background_color_subplot=[:ivory :mintcream],
            legend=[:topleft :bottomright], grid=true, gridalpha=0.15, margin=5Plots.mm)

        leg_size = 9
        temp_str = @sprintf("y(t) = %.2f °C", y_curr)
        power_str =  @sprintf("u(t) = %.2f %% (de 2.475 W)", u_curr)

        if np_val <= stab_pts         
       
            push!(tv_stab, t_curr); push!(yv_stab, y_curr); push!(uv_stab, u_curr)
            plot!(plt, subplot=1, tv_stab, yv_stab, label=temp_str, color="#ff6600", alpha=0.5,
            legendfontsize = leg_size, fg_legend = "#d4aa00", linewidth=1.5)
            plot!(plt, subplot=2, tv_stab, uv_stab, label=power_str, color="#2ca02c",
            legendfontsize = leg_size, fg_legend= "#5fbcd3", alpha=0.5, linewidth=1.5)



        else            
            ylims!(plt, subplot=2, (uv_stab[end-2] -4, uv_stab[end-2] + amplitude + 4))
            if np_val == stab_pts  + 1
                push!(tv_stab, t_curr); push!(yv_stab, y_curr); push!(uv_stab, u_curr)
            end
            push!(tv_step, t_curr); push!(yv_step, y_curr); push!(uv_step, u_curr)
            plot!(plt, subplot=1, tv_step, yv_step, label=temp_str, color="#d40055", linewidth=1.5,
            legendfontsize = leg_size, fg_legend = "#d4aa00")
            plot!(plt, subplot=2, tv_step, uv_step, label=power_str            
            , color="#2ca02c",  seriestype=:steppre,

            legendfontsize = leg_size, fg_legend= "#5fbcd3",  linewidth=1.5)
            plot!(plt, subplot=1, tv_stab, yv_stab, label="", color="#ff6600", alpha=0.5, linewidth=1.5)            
            plot!(plt, subplot=2, tv_stab, uv_stab, label="", color="#2ca02c", alpha=0.5, linewidth=1.5)


        end       
        
        
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=5.0)
    catch e
        println("Error: ", e)
        sleep(0.1)
        set_reference(sys, DEFAULT_REF);   
        disconnect!(sys)
        sleep(0.25)
        return nothing, nothing, nothing
    end


    t_rep = tv_step .- tstep    
    sleep(0.1)
    set_reference(sys, DEFAULT_REF);   
    disconnect!(sys)
    sleep(0.25)
    _save_exp([t_rep, uv_step, yv_step], "Thermal_step_open_exp.csv", "t,u,y")
    
    return tv_step, uv_step, yv_step

end

# ═══════════════════════════════════════════════════════════════════════════════
#  prbs_open
# ═══════════════════════════════════════════════════════════════════════════════

"""
    prbs_open(sys; yop, amplitude, stab_time, uee_time, divider, usefile)

Identificación en lazo abierto con señal PRBS.
El firmware primero estabiliza en `yop` °C, luego aplica la señal PRBS.

# Parámetros
- `yop`       : punto de operación [°C] (default 50)
- `amplitude` : amplitud pico de la PRBS en potencia [%] (default 4)
- `stab_time` : tiempo de estabilización [s] (default 60)
- `uee_time`  : tiempo de retención antes de la PRBS [s] (default 10)
- `divider`   : divisor de la PRBS — mayor = más lento (default 35)

Retorna `(t, u, y)` con t=0 en el inicio de la PRBS.

# Ejemplo
```julia
t, u, y = prbs_open(sys; yop=50, amplitude=4, divider=35)
```
"""
function prbs_open(sys::ThermalSystem;
                   yop::Real       = 50.0,
                   amplitude::Real = 5.0,
                   stab_time::Real = 60.0,
                   uee_time::Real  = 10.0,
                   divider::Int    = 25,
                   usefile::Bool   = false)
    if usefile
        return read_csv_file3(PATH_DATA * "Thermal_prbs_open_exp.csv")
    end

    h           = SAMPLING_TIME  
    stab_pts    = ceil(Int, stab_time / h)
    uee_pts     = ceil(Int, uee_time / h)
    points = divider * 63 + stab_pts + uee_pts 
    total_time = points * h


    m = 1.2341015052212259
    b = 24.750915901094388
    uf_est = (yop - b) / m
    percent = 0.2
    ymax = yop + m*amplitude + 2
    ymin = yop - m*amplitude - 2
    umax = uf_est + (1 + percent) * amplitude
    umin =  minimum([0, uf_est - (1 + percent) * amplitude - 5.0])

    payload = Dict(
        "peak_amp" => float2hex(amplitude),
        "op_point"       => float2hex(yop),  
        "stab_points" => long2hex(stab_pts),
        "uee_points"  => long2hex(uee_pts),
        "divider"   => long2hex(divider),
    )

    tv_prbs, uv_prbs, yv_prbs = Float64[], Float64[], Float64[]
    tv_stab, uv_stab, yv_stab = Float64[], Float64[], Float64[]

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["PRBS  yop=$(@sprintf("%.1f",yop)) °C con duración=$(@sprintf("%.0f",total_time)) s" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["" "Tiempo (s)"],
        xlims=[(0, total_time) (0, total_time)],
        ylims=[(ymin, ymax) (umin, umax)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "prbs_open", payload)

     on_sample = function(msg, np_val)

        t_curr = np_val * h        
        y_curr = hex2float(string(msg["y"]))
        u_curr = hex2float(string(msg["u"]))

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["PRBS  yop=$(@sprintf("%.1f",yop)) °C, duración=$(@sprintf("%.0f",total_time)) s" ""],
            ylabel=["Temperatura (°C)" "Potencia (%)"],
            xlabel=["" "Tiempo (s)"],
            xlims=[(0, total_time) (0, total_time)],
            ylims=[(ymin, ymax) (umin, umax)],
            background_color_subplot=[:ivory :mintcream],
            legend=[:topleft :bottomright], grid=true, gridalpha=0.15, margin=5Plots.mm)

        leg_size = 9
        temp_str = @sprintf("y(t) = %.2f °C", y_curr)
        power_str =  @sprintf("u(t) = %.2f %% (de 2.475 W)", u_curr)

        if np_val <= stab_pts + uee_pts       
       
            push!(tv_stab, t_curr); push!(yv_stab, y_curr); push!(uv_stab, u_curr)
            plot!(plt, subplot=1, tv_stab, yv_stab, label=temp_str, color="#ff6600", alpha=0.5,
            legendfontsize = leg_size, fg_legend = "#d4aa00", linewidth=1.5)
            plot!(plt, subplot=2, tv_stab, uv_stab, label=power_str, color="#2ca02c",
            legendfontsize = leg_size, fg_legend= "#5fbcd3", alpha=0.5, linewidth=1.5)



        else            
            ylims!(plt, subplot=2, (uv_stab[end-2] - amplitude - 2, uv_stab[end-2] + amplitude + 2))
            if np_val == stab_pts + uee_pts + 1
                push!(tv_stab, t_curr); push!(yv_stab, y_curr); push!(uv_stab, u_curr)
            end
            push!(tv_prbs, t_curr); push!(yv_prbs, y_curr); push!(uv_prbs, u_curr)
            plot!(plt, subplot=1, tv_prbs, yv_prbs, label=temp_str, color="#d40055", linewidth=1.5,
            legendfontsize = leg_size, fg_legend = "#d4aa00")
            plot!(plt, subplot=2, tv_prbs, uv_prbs, label=power_str, color="#2ca02c",  seriestype=:steppre,
            legendfontsize = leg_size, fg_legend= "#5fbcd3", linewidth=1.5)

            plot!(plt, subplot=1, tv_stab, yv_stab, label="", color="#ff6600", alpha=0.5, linewidth=1.5)            
            plot!(plt, subplot=2, tv_stab, uv_stab, label="", color="#2ca02c", alpha=0.5, linewidth=1.5)


        end         
        redraw!(plt)
    end

    try
        receive_samples!(sys, points, on_sample; timeout=5.0)
    catch e
        println("Error: ", e)
        sleep(0.1)
        set_reference(sys, DEFAULT_REF);   
        disconnect!(sys)
        sleep(0.25)
        return nothing, nothing, nothing
    end
        sleep(0.1)
        set_reference(sys, DEFAULT_REF);   
        disconnect!(sys)
        sleep(0.25)
    _save_exp([tv_prbs, uv_prbs, yv_prbs], "Thermal_prbs_open_exp.csv", "t,u,y")
    return tv_prbs, uv_prbs, yv_prbs
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_fomodel_step
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_fomodel_step(sys; yop, amplitude, t1, usefile)

Estima un modelo FOTD (primer orden más retardo) a partir de un escalón abierto.

    G(s) = α / (τ·s + 1) · exp(−L·s)

Usa el método de cuatro puntos con mínimos cuadrados.
Retorna `(G, L)` donde `G = tf([α], [τ, 1])`.

# Ejemplo
```julia
G, L = get_fomodel_step(sys; yop=50, amplitude=10)
println("G(s) = \$G   retardo L = \$L s")
```
"""
function get_fomodel_step(sys::ThermalSystem;
                          yop::Real       = 50.0,
                          amplitude::Real = 10.0,
                          t1::Real        = 600.0,
                          usefile::Bool   = false)
    if !usefile
         step_open(sys; yop=yop, amplitude=amplitude, t1=t1)
    end

    t, u, y = read_csv_file3(PATH_DATA * "Thermal_step_open_exp.csv")

    # Detectar el escalón en u
    ind_step = argmax(abs.(diff(u))) + 1
     

    ya     = mean(y[1:ind_step])
    yb     = mean(y[max(1, end-10):end])
    ua = u[1]
    ub= u[end]  
    delta_u = u[end] - u[1]
    delta_y = yb - ya
    alpha   = delta_y / delta_u

    y_norm = (y[ind_step:end] .- ya) ./ delta_y
    t_rel  = t[ind_step:end] 

    # Método de cuatro puntos (LSQ) para estimar τ y L
    yi_pts = [0.05, 0.1, 0.15, 0.2]
    num_pts = length(yi_pts)
    ti_pts = Float64[]
    for yp in yi_pts
        idx = findfirst(>=(yp), y_norm)
        if isnothing(idx) || idx <= 1
            push!(ti_pts, t_rel[1])
        else
            frac = (yp - y_norm[idx-1]) / (y_norm[idx] - y_norm[idx-1])
            push!(ti_pts, t_rel[idx-1] + frac*(t_rel[idx] - t_rel[idx-1]))
        end
    end

    A_lsq = hcat(ones(num_pts), [-log(1 - p) for p in yi_pts])
    x     = A_lsq \ ti_pts
    L_val = max(0.0, x[1])
    tau   = x[2]

    ymodel = [alpha * delta_u * (1 - exp(-max(0, ti - L_val) / tau)) + ya
              for ti in t_rel]
    
    b = alpha / tau
    a = 1 / tau

    leg_size = 12
    model_str = latexstring(@sprintf("G(s) = \\frac{%.4f}{s + %.3f} \\,\\,\\, e^{-%.2f\\,s}", b, a, L_val))


    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelo FOTD estimado – UNThermal" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        background_color_subplot=[:ivory :mintcream],                    
        xlims=[(t[1], t[end]) (t[1], t[end])],               
        ylims=[(ya - 3, yb + 3) (ua-5, ub+5)],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    leg_size = 9
    plot!(plt, subplot=1, t_rel, ymodel,
          label=model_str, legendfontsize = leg_size, 
          fg_legend = "#d4aa00",color=:deeppink, linewidth=1.5)
    plot!(plt, subplot=1, [t[1], t[ind_step]], [ya, ya],
          label="", color=:deeppink, linewidth=2)
    plot!(plt, subplot=1, t, y,
          label="Datos", color=:teal, linewidth=2, linestyle=:dot)
    plot!(plt, subplot=2, t, u, legendfontsize = leg_size, 
          label="Entrada", color=:green, fg_legend= "#5fbcd3",linewidth=1.0)
    display(plt)

    for path in (PATH_DATA, PATH_DEFAULT)
        open(path * "Thermal_fomodel_step.csv", "w") do io
            println(io, "b,a,L")
            @printf(io, "%.8f,%.8f,%.8f\n", b, a, L_val)
        end
    end

    s = tf("s")
    G = b/ (s + a)
    return G, L_val
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_models_prbs
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_models_prbs(sys; yop, amplitude, divider, usefile)

Identifica modelos FO y FOTD del sistema térmico usando una señal PRBS.

    G₁(s) = α / (τ·s + 1)               (primer orden)
    G₂(s) = α₂/(τ₁·s+1) · exp(−τ₂·s)   (primer orden con retardo)

Retorna `(G1, G2, tau2)`.

# Ejemplo
```julia
G1, G2, L = get_models_prbs(sys; yop=50, amplitude=4)
println("FO  : \$G1")
println("FOTD: \$G2  L = \$L s")
```
"""
function get_model_prbs(sys::ThermalSystem;
                         yop::Real       = 50.0,
                         amplitude::Real = 4.0,
                         divider::Int    = 25,
                         usefile::Bool   = false)
    if !usefile
        prbs_open(sys; yop=yop, amplitude=amplitude, divider=divider, 
                stab_time=120, uee_time=10 )
    end
    t, u, y = read_csv_file3(PATH_DATA * "Thermal_prbs_open_exp.csv")
    h = SAMPLING_TIME

    ymean = mean(y)
    ym    = y .- ymean
    um    = u .- mean(u)
    t = t .- t[1]  # ajustar t=0 al inicio de la PRBS

    # Identificación ARX de primer orden con ControlSystemIdentification.jl
    na, nb = 1,1
    data   = iddata(ym, um, h)

    Gd = arx(data, na, nb, estimator=wtls_estimator(y,na,nb))   # FO sin retardo


    G  = d2c(tf(Gd))
    
    L = h * 1.0   # retardo = 1 muestra

    # Simulaciones
    ysim1 = lsim(G, um', t).y[1, :]  
    fit1  = _fit_score(ym, ysim1)

    b1 = numvec(G)[1][1];  a1 = denvec(G)[1][2]


    modelstr1 = latexstring(@sprintf("G(s) = \\frac{%.4f}{s + %.3f} e^{-0.8s} \\quad (FIT=%.1f\\,\\%%)", b1, a1, fit1))

   
    t_total = (t[1], t[end])
    ylim = (minimum(y)-6, maximum(y)+1)
    ulim = (minimum(u)-2, maximum(u)+2)
    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelo de primer orden por PRBS con yop=$(@sprintf("%.1f",yop)) °C" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[t_total t_total],
        ylims=[ylim ulim],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomleft, grid=true, gridalpha=0.15, margin=5Plots.mm)
    plot!(plt, subplot=1, t, y,
          label="Datos", color=:teal, linewidth=1.5, linestyle=:dot)
    plot!(plt, subplot=1, t, ysim1 .+ ymean,
          label=modelstr1, color=:deeppink, linewidth=1.5, fg_legend = "#d4aa00", alpha_legend=0.5)
    plot!(plt, subplot=2, t, u,
          label="PRBS u(t)", color=:green, linewidth=0.8, seriestype=:steppre)
    display(plt)


    open(PATH_DATA * "Thermal_fo_model_pbrs.csv", "w") do io
        println(io, "b,a,L")
        @printf(io, "%.8f,%.8f,%.8f\n", b1, a1, L)
    end


    return G, L
end

"""Métrica FIT: 100 % = ajuste perfecto."""
function _fit_score(y_data::Vector, y_sim::Vector)
    num = norm(y_data .- y_sim)
    den = norm(y_data .- mean(y_data))
    den < eps() && return 100.0
    return max(0.0, (1 - num / den) * 100)
end
