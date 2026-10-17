# ═══════════════════════════════════════════════════════════════════════════════
#  identsys.jl – Funciones de identificación para el sistema ThermoLab.
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using Plots
using ControlSystemsBase
using LinearAlgebra
using Statistics
using Printf
using ControlSystemIdentification
using LaTeXStrings


# ═══════════════════════════════════════════════════════════════════════════════
#  Utilidades comunes de los experimentos en lazo abierto
# ═══════════════════════════════════════════════════════════════════════════════

"""
Verifica el punto de operación `yop` y retorna la potencia estimada para
alcanzarlo, `uf_est`, junto con la ganancia estática `K` (°C/%). Con
`symmetric=true` (PRBS) verifica también que `uf_est - amplitude` no sea negativa.
"""
function _operating_point(sys::ThermalSystem, yop::Real, amplitude::Real; symmetric::Bool = false)
    K, _ = read_model(_datafile("ThermoLab_static_pars.csv"))
    uf_est = power_from_temp(sys, yop)
    if uf_est + amplitude > 100 || (symmetric && uf_est - amplitude < 0)
        error("Con yop=$(yop) °C la potencia de operación estimada es " *
              "$(@sprintf("%.1f", uf_est)) %; la amplitud $(amplitude) % la " *
              "sacaría del intervalo [0, 100] %.")
    end
    return uf_est, K
end

# ═══════════════════════════════════════════════════════════════════════════════
#  step_open  (respuesta escalón en lazo abierto)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_open(sys::ThermalSystem; yop=50, amplitude=10, t1=600, stab_time=60, uee_time=10)

Configura y ejecuta un experimento para observar la respuesta de la
temperatura de la plataforma **ThermoLab** ante un escalón de potencia aplicado en
lazo abierto (es decir, sin ningún controlador activo durante el escalón).

El experimento tiene tres fases, como se muestra en la figura siguiente:

![Respuesta al escalón en lazo abierto de la plataforma ThermoLab](../../assets/step_open.png)

1. **Estabilización** (duración `stab_time`): un controlador PI interno del
   firmware lleva la temperatura al punto de operación `yop`.
2. **Retención** (duración `uee_time`): se abre el lazo y se mantiene
   constante la potencia ``u_{ee}`` que sostiene la temperatura en `yop`.
3. **Escalón** (duración `t1`): la potencia pasa a ``u_{ee}`` + `amplitude`.

Los parámetros mostrados en la figura se describen a continuación.

# Argumentos
- `sys::ThermalSystem`: objeto de la plataforma.

# Argumentos de palabra clave
- `yop::Real=50`: temperatura del punto de operación, en °C.
- `amplitude::Real=10`: amplitud del escalón de potencia, en %.
- `t1::Real=600`: duración en segundos del escalón.
- `stab_time::Real=60`: duración en segundos de la fase de estabilización.
- `uee_time::Real=10`: duración en segundos de la fase de retención.

# Retorna
- `(t, u, y)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  potencia aplicada (%) y la temperatura medida (°C), desde el inicio de la
  fase de retención. El tiempo `t = 0` corresponde al instante del escalón.

# Notas
- Se usa típicamente para identificar el modelo dinámico de la plataforma
  **ThermoLab** (ver [`get_model_step`](@ref)). La respuesta se grafica en tiempo real.
- La potencia de operación se estima con el modelo estático (ver
  [`get_static_model`](@ref) y [`power_from_temp`](@ref)) solo para verificar que el
  escalón no exceda el intervalo [0, 100] % y para escalar la gráfica.
- Los datos del experimento se guardan en un archivo CSV en
  `datafiles/ThermoLab_step_open_exp.csv`.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab y de haber
definido el sistema, así:

```julia
using ThermoLab
sys = ThermalSystem();
```

Luego, obtenga la respuesta al escalón en lazo abierto así:

```julia
t, u, y = step_open(sys; yop=50, amplitude=10, t1=600);
```
"""
function step_open(sys::ThermalSystem;
                   yop::Real       = 50.0,
                   amplitude::Real = 10.0,
                   t1::Real        = 360.0,
                   stab_time::Real = 50.0,
                   uee_time::Real  = 10.0)



  

    h        = SAMPLING_TIME
    stab_pts = round(Int, stab_time / h)
    uee_pts  = round(Int, uee_time / h)
    step_pts = round(Int, t1 / h) + 2
    total    = stab_pts + uee_pts + step_pts
    t_total  = total * h



    uf_est, K = _operating_point(sys, yop, amplitude)
    percent = 0.2

    ymax = yop + K * amplitude + 5
    ymin = yop - 5
    umax = uf_est + (1 + percent) * amplitude
    umin =  minimum([0, uf_est - (1 + percent) * amplitude])


    payload = Dict(
        "amplitude"   => float2hex(amplitude),
        "op_point"    => float2hex(yop),
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
    redraw!(plt)

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
            plot!(plt, subplot=1, tv_stab, yv_stab, label=temp_str, color="#ff6600", alpha=0.8,
            legendfontsize = leg_size, fg_legend = "#d4aa00", linewidth=1.5, ls = :dot)
            plot!(plt, subplot=2, tv_stab, uv_stab, label=power_str, color="#2ca02c",
            legendfontsize = leg_size, fg_legend= "#5fbcd3", alpha=0.8, linewidth=1.5,  ls = :dot)



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
            plot!(plt, subplot=1, tv_stab, yv_stab, label="", color="#ff6600",
             ls = :dot, alpha=0.8, linewidth=1.5)            
            plot!(plt, subplot=2, tv_stab, uv_stab, label="", color="#2ca02c",
             ls = :dot, alpha=0.8, linewidth=1.5)


        end
        
        redraw!(plt)
    end

    try
        receive_samples!(sys, total, on_sample; timeout=5.0)
    catch e
        println("Error: ", e)
    end
    
    set_reference(sys, DEFAULT_REFERENCE)  # Volver a referencia cero al finalizar
    
    t_rep = tv_step .- tstep    
    # Volver a referencia de 40° al finalizar
    set_reference(sys, DEFAULT_REFERENCE);   
    save_experiment([t_rep, uv_step, yv_step], "ThermoLab_step_open_exp.csv", "t,u,y")
    
    return tv_step, uv_step, yv_step

end



# ═══════════════════════════════════════════════════════════════════════════════
#  prbs_open  (identificación con señal PRBS en lazo abierto)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    prbs_open(sys::ThermalSystem; yop=50, amplitude=4, stab_time=60, uee_time=10, divider=25)

Configura y ejecuta un experimento de identificación en lazo abierto,
excitando la plataforma **ThermoLab** con una señal binaria pseudoaleatoria (PRBS)
de potencia alrededor de un punto de operación, como se ilustra en la figura:

![Señal PRBS aplicada en lazo abierto a la plataforma ThermoLab](../../assets/prbs_open.png)

Igual que en [`step_open`](@ref), el experimento tiene tres fases:
**estabilización** en la temperatura `yop` (durante `stab_time`) con un PI interno
del firmware, **retención** de la potencia de operación ``u_{ee}`` (durante `uee_time`),
y finalmente la **señal PRBS**, que alterna entre ``u_{ee}`` - `amplitude` y
``u_{ee}`` + `amplitude`.

Los parámetros de esta función son:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `yop::Real=50`: temperatura del punto de operación, en °C.
- `amplitude::Real=4`: amplitud pico de la señal PRBS, en % de potencia.
- `stab_time::Real=60`: duración en segundos de la fase de estabilización.
- `uee_time::Real=10`: duración en segundos de la fase de retención.
- `divider::Int=25`: número de periodos de muestreo durante los cuales se
  mantiene cada bit de la secuencia PRBS (a mayor valor en `divider`, se aplica una señal más
  "lenta" y de mayor duración total).

  Note que el tamaño de la secuencia PRBS es fijo
   (tiene 63 valores ya grabados en el firmware de la plataforma) y cada uno de esos
   valores se repite un número `divider` de veces. Con `divider=25` la señal PRBS
   dura ``63 \\times 25 \\times 0.8 = 1260`` s.

# Retorna
- `(t, u, y)`: tupla de vectores `Vector{Float64}` con el tiempo (s), la
  potencia aplicada (%) y la temperatura medida (°C) durante la señal PRBS. El
  tiempo `t = 0` corresponde al inicio de la PRBS.

# Notas

- Esta función se usa para identificar el modelo dinámico de la plataforma
  por medio de  [`get_model_prbs`](@ref). El usuario también puede
  disponer de los datos para usar otros algoritmos de identificación de sistemas como los
  del paquete [ControlSystemIdentification.jl](https://github.com/baggepinnen/ControlSystemIdentification.jl).
- Los datos del experimento se guardan en un archivo CSV en
  `datafiles/ThermoLab_prbs_open_exp.csv`.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab y de haber
definido el sistema, así:

```julia
using ThermoLab
sys = ThermalSystem();
```

Luego, obtenga la respuesta a la señal PRBS en lazo abierto así:

```julia
t, u, y = prbs_open(sys; yop=50, amplitude=4, divider=25);
```
"""
function prbs_open(sys::ThermalSystem;
                   yop::Real       = 50.0,
                   amplitude::Real = 5.0,
                   stab_time::Real = 90.0,
                   uee_time::Real  = 10.0,
                   divider::Int    = 25,
                   usefile::Bool   = false)


    h           = SAMPLING_TIME  
    stab_pts    = ceil(Int, stab_time / h)
    uee_pts     = ceil(Int, uee_time / h)
    points = divider * PRBS_LENGTH + stab_pts + uee_pts 
    total_time = points * h


    uf_est, K = _operating_point(sys, yop, amplitude)
    percent = 0.2
    ymax = yop + K * amplitude + 2
    ymin = yop - K * amplitude - 2
    umax = uf_est + (1 + percent) * amplitude
    umin =  minimum([0, uf_est - (1 + percent) * amplitude - 5.0])


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
    redraw!(plt)

    payload = Dict(
        "peak_amp" => float2hex(amplitude),
        "op_point"       => float2hex(yop),  
        "stab_points" => long2hex(stab_pts),
        "uee_points"  => long2hex(uee_pts),
        "divider"   => long2hex(divider),
    )


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
    end
    
    set_reference(sys, DEFAULT_REFERENCE)  # Volver a referencia por defecto al finalizat
    save_experiment([tv_prbs, uv_prbs, yv_prbs], "ThermoLab_prbs_open_exp.csv", "t,u,y")
    return tv_prbs, uv_prbs, yv_prbs
end




# ═══════════════════════════════════════════════════════════════════════════════
#  _step_static  (escalón en lazo cerrado para la curva estática)
# ═══════════════════════════════════════════════════════════════════════════════

"""Escalón en lazo cerrado hasta `r_ref`. Retorna la potencia y temperatura estacionarias."""
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
            hline!(p_ins, [r_ref], label="Referencia", color=:gray, ls=:dash, linewidth=1)
            
            plt = plot( p_ins, p_ee, layout=(1, 2), size=(900, 500),
            title=["Curva estática – ThermoLab" "Experimento actual"],
            xlabel=[ "Tiempo (s)" "Potencia (%)"],
            ylabel=["Temp. (°C) y potencia (%) actual" "Temperatura estacionaria (°C)" ], 
            xlims=[ (0,t1) (0, 60)] ,
            ylims=[(0, 100) (20, 100)],
            background_color_subplot=[ :mintcream :ivory],
            legend=[:bottomright :bottomright ], 
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
#  get_static_model  (curva estática del sistema térmico)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_static_model(sys::ThermalSystem; step=5.0, duration=60.0, usefile=false)

Realiza una secuencia de experimentos de respuesta al escalón en lazo cerrado para
obtener la curva de ganancia estática (curva potencia vs. temperatura
estacionaria) de la plataforma ThermoLab.

Esta función fija sucesivamente la referencia de temperatura en los valores
40, 40 + `step`, 40 + 2·`step`, … hasta 90 °C. Para cada nivel, una vez que la
temperatura alcanza su valor estacionario, se registran el promedio de las
últimas 10 muestras de la temperatura y de la potencia aplicada, conforme se
muestra en la figura siguiente:

![Curva estática potencia-temperatura de ThermoLab](../../assets/get_static_model.png)

A continuación se describen los parámetros de la función:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `step::Real=5.0`: separación, en °C, entre referencias sucesivas.
- `duration::Real=60.0`: duración, en segundos, de cada escalón de referencia.
- `usefile::Bool=false`: si es `true`, no se realiza un experimento nuevo, sino que
  se ajusta el modelo con los datos guardados en
  `datafiles/ThermoLab_static_gain_response.csv`.

# Retorna
- `(uee, yee)`: tupla de vectores `Vector{Float64}` con las potencias
  estacionarias (%) y las temperaturas estacionarias medidas (°C) en cada nivel.

# Notas
- El controlador que actúa durante los experimentos es el que esté cargado en
  la plataforma (por defecto, el PID que trae el firmware). Debe ser capaz de
  llevar la temperatura a la referencia en el tiempo `duration`.
- Al finalizar la secuencia se muestra un modelo lineal que relaciona la
  temperatura en estado estacionario con la potencia aplicada:

``y_{ee} = K\\,u_{ee} + b``

  Este modelo se ajusta por mínimos cuadrados; ``b`` corresponde aproximadamente a
  la temperatura ambiente.

- Los resultados se guardan en `datafiles/ThermoLab_static_gain_response.csv`
  y `datafiles/ThermoLab_static_pars.csv` (usados luego por
  [`temp_from_power`](@ref) y [`power_from_temp`](@ref)).

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab y de haber
definido el sistema, así:

```julia
using ThermoLab
sys = ThermalSystem();
```

Luego, obtenga el modelo de ganancia estática así:

```julia
uee, yee = get_static_model(sys);
```
"""
function get_static_model(sys::ThermalSystem;
                          step::Real    = 5.0, t1::Real = 40.0,
                          usefile::Bool = false)
    if usefile
        uss, yss = read_csv_file(PATH_DATA * "Thermal_static_gain_response.csv")
    else


        plt = plot(layout=(1, 2), size=(900, 500),
            title=["Curva estática – ThermoLab" "Experimento actual"],
            xlabel=[ "Tiempo (s)" "Potencia (%)"],
            ylabel=["Temp. (°C) y potencia (%) actual" "Temperatura estacionaria (°C)" ], 
            xlims=[ (0,t1) (0, 60)] ,
            ylims=[(0, 100) (20, 100)],
            background_color_subplot=[ :mintcream :ivory],
            legend=[:bottomright :bottomright ], 
            grid=true, gridalpha=0.2,
            xticks=0:10:100, yticks=0:10:100,
            margin=5Plots.mm)
            redraw!(plt)

        uss      = Float64[]
        yss      = Float64[]
        y_targets = collect(40.0:step:MAX_TEMPERATURE) 
    
        for yi in y_targets        
            uee_curr, yee_curr = _step_static(sys, yi , uss, yss; t1=t1)
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
        margin=3Plots.mm)
    redraw!(plt)
    save_experiment([uss, yss], "ThermoLab_static_gain_response.csv", "u,y")
    save_experiment([[K], [b]], "ThermoLab_static_pars.csv", "K,b")   
    println("Curva estática completada con $(length(uss)) puntos")   
    set_reference(sys, DEFAULT_REFERENCE);   
    return uss, yss
end




# ═══════════════════════════════════════════════════════════════════════════════
#  get_model_step  (modelo de primer orden con retardo desde step)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_model_step(sys::ThermalSystem; yop=50, amplitude=10, t1=600, usefile=false)

Estima, mediante la respuesta al escalón,  un modelo dinámico de primer orden con retardo
 **FOTD** (del inglés, *First Order plus Time Delay*), el cual relaciona la temperatura
de la plataforma ThermoLab con la potencia aplicada. El modelo FOTD está representado por la siguiente función de transferencia:

``G(s) = \\dfrac{\\alpha}{\\tau s + 1} e^{-Ls}``

donde ``\\alpha`` es la ganancia del sistema, ``\\tau`` es la constante de
tiempo y ``L`` es el retardo.


Para estimar este modelo se ejecuta [`step_open`](@ref): la temperatura se
estabiliza en el punto de operación `yop` y luego se aplica un escalón de potencia de
amplitud `amplitude`, como se muestra en la siguiente figura:

![Modelo de primer orden con retardo desde la respuesta al escalón](../../assets/get_model_step.png)

Los parámetros de esta función son los siguientes:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `yop::Real=50`: temperatura del punto de operación (en °C) alrededor del cual
  se obtiene el modelo FOTD.
- `amplitude::Real=10`: amplitud del escalón de potencia, en %.
- `t1::Real=600`: duración en segundos del escalón. Debe ser suficiente para que la
  temperatura llegue al estado estacionario (varias constantes de tiempo).
- `usefile::Bool=false`: si es `true`, se usan los datos del último experimento,
  guardados en `datafiles/ThermoLab_step_open_exp.csv` en lugar de ejecutar uno
  nuevo.

# Retorna
- `G::ControlSystemsBase.TransferFunction`: modelo continuo de primer orden
  `b/(s+a)`, equivalente a `α/(τs+1)` con
  `b = α/τ` y `a = 1/τ`. Retorna así por compatibilidad con las otras funciones.
- `L::Float64`: retardo estimado, en segundos.

# Notas

- El retardo `L` y la constante de tiempo `τ` se ajustan por mínimos
  cuadrados a partir de los tiempos de cruce de la respuesta en
  los niveles 5 %, 10 %, 15 % y 20 % del cambio total de temperatura. La ganancia estática
  `α = Δy/Δu` se calcula directamente de los valores estacionarios antes y
  después del escalón.
- Se muestra una gráfica comparando la salida experimental con el modelo
  simulado, y se guardan los parámetros en `datafiles/ThermoLab_fo_model_2p.csv`.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab y de haber
definido el sistema, así:

```julia
using ThermoLab
sys = ThermalSystem();
```

Es recomendable, al usar por primera vez la plataforma, obtener el modelo estático
para que haya una buena calibración del punto de operación, así:

```julia
uee, yee = get_static_model(sys);
```

Luego, obtenga el modelo FOTD a partir de la respuesta al escalón así:

```julia
G, L = get_model_step(sys; yop=50, amplitude=10);
```
"""
function get_model_step(sys::ThermalSystem;
                        yop::Real = 50.0, amplitude::Real = 10.0,
                        t1::Real = 600.0, usefile::Bool = false)
    if !usefile
        t, u, y = step_open(sys; yop=yop, amplitude=amplitude, t1=t1)
    else
        t, u, y = read_csv_file3(_datafile("ThermoLab_step_open_exp.csv"))
    end

    # Detectar el escalón en u
    ind_step = argmax(abs.(diff(u))) + 1

    ya      = mean(y[1:ind_step-1])
    yb      = mean(y[max(1, end-9):end])
    ua, ub  = u[1], u[end]
    delta_u = ub - ua
    delta_y = yb - ya
    alpha   = delta_y / delta_u

    # Normalizar la respuesta posterior al escalón
    y_norm = (y[ind_step:end] .- ya) ./ delta_y
    t_rel  = t[ind_step:end] .- t[ind_step]

    # LSQ para estimar tau y L:  ti = L + tau * (-ln(1 - yi))
    yi_pts = [0.05, 0.1, 0.15, 0.2]
    ti_pts = Float64[]
    for yp in yi_pts
        idx = findfirst(>=(yp), y_norm)
        if idx === nothing || idx <= 1
            push!(ti_pts, t_rel[1])
        else
            # Interpolación lineal entre idx-1 e idx
            frac = (yp - y_norm[idx-1]) / (y_norm[idx] - y_norm[idx-1])
            push!(ti_pts, t_rel[idx-1] + frac * (t_rel[idx] - t_rel[idx-1]))
        end
    end
    A_lsq = hcat(ones(length(yi_pts)), [-log(1 - p) for p in yi_pts])
    x     = A_lsq \ ti_pts
    L_val = max(0.0, x[1])
    tau   = x[2]

    # Modelo simulado
    tsim   = range(t[1], t[end], length=1000)
    ymodel = [ti < t[ind_step] + L_val ? ya :
              ya + alpha * delta_u * (1 - exp(-(ti - t[ind_step] - L_val) / tau)) for ti in tsim]

    # Gráfica
    model_str = latexstring(@sprintf("G(s) = \\frac{%.3f}{%.2f\\,s+1} e^{-%0.2f\\,s}", alpha, tau, L_val))

    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelo FOTD estimado para ThermoLab" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["" "Tiempo (s)"],
        xlims=[(t[1], t[end]) (t[1], t[end])],
        ylims=[(min(ya, yb) - 2, max(ya, yb) + 2) (min(ua, ub) - 3, max(ua, ub) + 3)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    plot!(plt, subplot=1, t, y, label="Datos", color=:teal,
          linewidth=2, linestyle=:dot)
    plot!(plt, subplot=1, tsim, ymodel, label=model_str, legendfontsize = 10, color=:deeppink,
          linewidth=1.5)
    plot!(plt, subplot=2, t, u, label="Entrada", color=:green, seriestype=:steppre)
    redraw!(plt)

    s = tf("s")
    b = alpha / tau
    a = 1 / tau
    G = b / (s + a)

    save_experiment([[b], [a], [L_val]], "ThermoLab_fo_model_2p.csv", "b,a,L")
    return G, L_val
end

# ═══════════════════════════════════════════════════════════════════════════════
#  get_model_prbs  (modelo de uno o dos polos desde PRBS)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_model_prbs(sys::ThermalSystem; yop=50, amplitude=4, divider=25, numpar=2, usefile=false)

Estima un modelo dinámico lineal que relaciona la temperatura de la plataforma ThermoLab con
la potencia aplicada. Según el valor de `numpar`, el modelo puede ser:

- `numpar=2` (por defecto): modelo de primer orden, con dos parámetros (``a`` y ``b``):

  ``\\qquad G(s) = \\dfrac{b}{s+a}``

- `numpar=3`: modelo de segundo orden con dos polos reales, con tres parámetros
  (``a_1``, ``a_2`` y ``b``):

  ``\\qquad G(s) = \\dfrac{b}{(s+a_1)(s+a_2)}``

Para obtener el modelo se realiza un experimento en lazo abierto con [`prbs_open`](@ref),
aplicando como entrada una onda binaria pseudoaleatoria (PRBS) de potencia
alrededor del punto de operación `yop`, tal como se ilustra en la figura siguiente:

![Identificación con PRBS de la plataforma ThermoLab](../../assets/get_model_prbs.png)

Los parámetros de esta función son los siguientes:

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `yop::Real=50`: temperatura del punto de operación (en °C) alrededor del cual
  se obtiene el modelo lineal.
- `amplitude::Real=4`: amplitud pico de la señal PRBS, en % de potencia.
- `divider::Int=25`: número de periodos de muestreo que dura cada bit de la PRBS
  (ver [`prbs_open`](@ref)).
- `numpar::Int=2`: número de parámetros del modelo a estimar: `2` para el modelo de
  primer orden ``b/(s+a)`` o `3` para el modelo de dos polos ``b/((s+a_1)(s+a_2))``.
- `usefile::Bool=false`: si es `true`, no se realiza un experimento nuevo, sino que se
  usan los datos del último experimento PRBS guardado en
  `datafiles/ThermoLab_prbs_open_exp.csv` para estimar los parámetros del modelo.

# Retorna
- `G::ControlSystemsBase.TransferFunction`: modelo de la temperatura
  identificado a partir de los datos PRBS (de uno o dos polos, según `numpar`).
- `L::Float64`: retardo, fijado en un periodo de muestreo (0.8 s).

# Notas
- Con `numpar=2` se ajusta un modelo ARX discreto de orden (1,1) con retardo de una
  muestra, usando el estimador de
  [ControlSystemIdentification.jl](https://github.com/baggepinnen/ControlSystemIdentification.jl),
  y el modelo discreto resultante se convierte a tiempo continuo con la función `d2c`.
- Con `numpar=3` se realiza una identificación de caja gris con el método del error de
  predicción (`structured_pem` de ControlSystemIdentification.jl), estimando
  directamente ``a_1``, ``a_2`` y ``b`` del modelo continuo.
- Al finalizar, se muestra una gráfica que compara la salida experimental con la
  simulada por el modelo (indicando el porcentaje de ajuste, o *FIT*), y se guardan los
  parámetros estimados. Luego se pueden recuperar con [`tf`](@ref), [`ss`](@ref) o
  [`get_last_model`](@ref) usando el mismo valor de `numpar`.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab y de haber
definido el sistema, así:

```julia
using ThermoLab
sys = ThermalSystem();
```

Es recomendable, al usar por primera vez la plataforma, obtener el modelo estático
para que haya una buena calibración del punto de operación, así:

```julia
uee, yee = get_static_model(sys);
```

Luego, estime los parámetros del modelo de primer orden a partir de un experimento
PRBS, así:

```julia
G, L = get_model_prbs(sys; yop=50);
```

o, si se desea el modelo de dos polos a partir de los mismos datos:

```julia
G2, L = get_model_prbs(sys; numpar=3, usefile=true);
```
"""
function get_model_prbs(sys::ThermalSystem;
                        yop::Real = 50.0, amplitude::Real = 5.0,
                        numpar::Int = 2, divider::Int = 25, usefile::Bool = false)
    
    numpar in (2, 3) || error("numpar debe ser 2 o 3")

    if !usefile
        t, u, y = prbs_open(sys; yop=yop, amplitude=amplitude, divider=divider,
                            stab_time=240, uee_time=10)
    else
        t, u, y = read_csv_file3(_datafile("ThermoLab_prbs_open_exp.csv"))
    end

    s = tf("s")

    function model2p(p)
        a1, a2, b = p
        sis = b/((s+a1)*(s+a2)) # modelo continuo
        return ss(sis, balance=false)
    end


    t = t .- t[1]   # t=0 al inicio de la PRBS
    
    ymean_val = mean(y)

    # removing means
    um = u .- mean(u)
    ym = y .- ymean_val

    data = iddata(ym, um, SAMPLING_TIME)
    u_matrix = reshape(um, 1, :)
    L = SAMPLING_TIME  # Retardo de una muestra

    if numpar == 2
        na = 1
        nb = 1
        Gh = arx(data, na, nb, inputdelay=1, estimator = wtls_estimator(data.y, na, nb))
        G1 = d2c(Gh)
        res = lsim(G1, u_matrix, t)
        ysim = vec(res.y)
        r1 = modelfit(data.y', ysim)
        b = numvec(G1)[1][1]
        a = denvec(G1)[1][2]
        modelstr1 = latexstring(@sprintf("G(s) = \\frac{%.5f}{s + %.5f} \\quad (FIT=%.1f\\,\\%%)", b, a, r1[1]))

        save_experiment([[b], [a], [L]], "ThermoLab_fo_model_2p.csv", "b,a,L")

    else  # numpar == 3
        p0 = [0.025, 0.05, 1.3]  # valores iniciales para [a1, a2, b]
        optim_model = structured_pem(data, 2; p0=p0, constructor=model2p)
        a1, a2, b = optim_model.res.minimizer.p
        a1, a2 = minmax(a1, a2)
        G1 = b/((s+a1)*(s+a2))
        res = lsim(G1, u_matrix, t)
        ysim = vec(res.y)
        r1 = modelfit(data.y', ysim)
        modelstr1 = latexstring(@sprintf("G(s) = \\frac{%.5f}{(s + %.4f)(s + %.4f)} \\quad (FIT=%.1f\\,\\%%)", b, a1, a2, r1[1]))
        save_experiment([[b], [a1], [a2], [L]], "ThermoLab_fo_model_3p.csv", "b,a1,a2,L")
    end

    # ── Gráfica comparativa ──────────────────────────────────────────
    xlims_v = (t[1], t[end])
    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelo PRBS  yop=$(@sprintf("%.1f",ymean_val)) °C" ""],
        ylabel=["Temperatura (°C)" "Potencia (%)"],
        xlabel=["" "Tiempo (s)"],
        xlims=[xlims_v xlims_v],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomleft, grid=true, gridalpha=0.15, margin=5Plots.mm)
    plot!(plt, subplot=1, t, ym .+ ymean_val,
          label="Datos", color=:teal, linewidth=1.5, linestyle=:dot)
    plot!(plt, subplot=1, t, ysim .+ ymean_val,
          label=modelstr1, color=:deeppink, linewidth=1.5)
    plot!(plt, subplot=2, t, u,
          label="PRBS", color=:green, seriestype=:steppre)
    redraw!(plt)
    return G1, L
end
