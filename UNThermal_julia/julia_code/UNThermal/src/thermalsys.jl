# ═══════════════════════════════════════════════════════════════════════════════
#  thermalsys.jl – Comunicación serial con el sistema térmico (ESP32).
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using LibSerialPort
using JSON3
using DelimitedFiles
using Printf
using Statistics



# ── Constantes del protocolo ─────────────────────────────────────────────────
const SAMPLING_TIME = 0.8        # Tiempo de muestreo del sistema térmico [s]
       # Longitud del PRBS (máximo 2^n - 1, n ≤ 16)

# ── Rutas por defecto ────────────────────────────────────────────────────────
const PATH_DATA    = joinpath(@__DIR__, "..", "datafiles") * "/"




# ═══════════════════════════════════════════════════════════════════════════════
#  Funciones de conversión hex  (idénticas al protocolo del firmware)
#  IEEE-754 float32 big-endian, coincide con struct.pack('>f') de Python.
# ═══════════════════════════════════════════════════════════════════════════════

"""Convierte Float32 a string hexadecimal de 8 caracteres (IEEE-754)."""
function float2hex(value::Real)::String
    u = reinterpret(UInt32, Float32(value))
    return string(u; base=16, pad=8)
end

"""Convierte string hexadecimal de 8 caracteres a Float64 (IEEE-754)."""
function hex2float(h::AbstractString)::Float64
    u = parse(UInt32, h; base=16)
    return Float64(reinterpret(Float32, u))
end

"""Convierte UInt32 a string hexadecimal de 8 caracteres."""
function long2hex(value::Integer)::String
    return string(UInt32(value); base=16, pad=8)
end

"""Convierte string hexadecimal de 8 caracteres a Int."""
function hex2long(h::AbstractString)::Int
    return Int(parse(UInt32, h; base=16))
end

"""Convierte un string hex (posiblemente con comas) a vector de Float64."""
function hexframe_to_array(hexframe::AbstractString)::Vector{Float64}
    return [hex2float(strip(p)) for p in split(hexframe, ",")]
end

"""Convierte una matriz (2D array) a string hex fila por fila."""
function matrix2hex(mat)::String
    if ndims(mat) == 1
        return join(float2hex(e) for e in mat)
    else
        return join(float2hex(mat[i, j]) for i in axes(mat, 1) for j in axes(mat, 2))
    end
end

# ═══════════════════════════════════════════════════════════════════════════════
#  Lectura de archivos CSV
# ═══════════════════════════════════════════════════════════════════════════════

"""Lee archivo CSV de 2 columnas (u, y). Retorna (u, y)."""
function read_csv_file(filepath::String = PATH_DATA * "Thermal_static_gain_response.csv")
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2])
end

"""Lee archivo CSV de 3 columnas (t, u, y). Retorna (t, u, y)."""
function read_csv_file3(filepath::String = PATH_DATA * "Thermal_step_open_exp.csv")
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2]), Float64.(data[:, 3])
end


"""Lee archivo CSV de 4 columnas (t, u, y). Retorna (t, u, y)."""
function read_csv_file4(filepath::String = PATH_DATA * "Thermal_step_open_exp.csv")
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2]), Float64.(data[:, 3]), Float64.(data[:, 4])
end

# ═══════════════════════════════════════════════════════════════════════════════
#  Estructura principal: ThermalSystem
# ═══════════════════════════════════════════════════════════════════════════════

"""
    ThermalSystem(; port, bauds)

Interfaz serial con el sistema UNThermal (ESP32).
Equivalente a la clase ThermalSystemIoT de Python.

# Ejemplo
```julia
sys = ThermalSystem(port="/dev/ttyUSB0")
```
"""
mutable struct ThermalSystem
    port         :: String
    bauds        :: Int
    sp           :: Union{SerialPort, Nothing}
    topics       :: Dict{String, String}
    queue        :: Channel{Dict{String, Any}}
    reader_task  :: Union{Task, Nothing}
    stop_reader  :: Threads.Atomic{Bool}

    function ThermalSystem(;
            port::String = "/dev/ttyUSB0",
            bauds::Int   = 460800)

        mkpath(PATH_DATA)

        topics = Dict(
            "set_pid"       => "/set_pid",
            "set_ref"       => "/set_ref",
            "step_closed"   => "/step_closed",
            "stairs_closed" => "/stairs_closed",
            "prbs_open"     => "/prbs_open",
            "step_open"     => "/step_open",
            "set_gencon"    => "/set_gencon",
            "prof_closed"   => "/prof_closed",
        )
        new(port, bauds, nothing, topics,
            Channel{Dict{String, Any}}(1024), nothing, Threads.Atomic{Bool}(false))
    end
end


# ── Conexión / desconexión ───────────────────────────────────────────────────

"""Abre el puerto serial e inicia la tarea lectora en segundo plano."""
function connect!(sys::ThermalSystem)
    if sys.sp === nothing
        sys.sp = LibSerialPort.open(sys.port, sys.bauds)
        try sp_flush(sys.sp, SP_BUF_INPUT) catch end
    end
    _start_reader!(sys)
    return nothing
end

"""Detiene la tarea lectora y cierra el puerto serial."""
function disconnect!(sys::ThermalSystem)
    _stop_reader!(sys)
    if sys.sp !== nothing
        close(sys.sp)
        sys.sp = nothing
    end
    return nothing
end


# ── Hilo lector en segundo plano ─────────────────────────────────────────────

function _start_reader!(sys::ThermalSystem)
    if sys.reader_task !== nothing && !istaskdone(sys.reader_task)
        return
    end
    sys.stop_reader[] = false
    while isready(sys.queue)
        take!(sys.queue)
    end
    sys.reader_task = Threads.@spawn _reader_loop(sys)
    return nothing
end

function _stop_reader!(sys::ThermalSystem)
    sys.stop_reader[] = true
    if sys.reader_task !== nothing
        t0 = time()
        while !istaskdone(sys.reader_task) && (time() - t0) < 5.0
            sleep(0.05)
        end
        sys.reader_task = nothing
    end
    return nothing
end

function _reader_loop(sys::ThermalSystem)
    buf = IOBuffer()
    while !sys.stop_reader[]
        try
            if sys.sp === nothing; break; end
            nb = bytesavailable(sys.sp)
            if nb == 0
                sleep(0.01)
                continue
            end
            bytes = read(sys.sp, nb)
            write(buf, bytes)

            raw   = String(take!(buf))
            lines = split(raw, '\n')
         
            
            if !endswith(raw, '\n')
                write(buf, lines[end])
                lines = lines[1:end-1]
                
                
            end

            for line in lines
                line = strip(line)
                isempty(line) && continue

                # Solo líneas JSON (el firmware térmico no emite prefijo "D:")
                startswith(line, "{") || continue

                msg = try
                    JSON3.read(line, Dict{String, Any})
                catch
                    continue
                end

                # El firmware térmico usa "np" (número de punto de muestreo)
                if haskey(msg, "np")
                    try
                        put!(sys.queue, msg)
                    catch
                        break
                    end
                end
            end
        catch e
            e isa InterruptException && break
            sleep(0.01)
        end
    end
end


# ── Envío de comandos ────────────────────────────────────────────────────────

"""
    send_command!(sys, topic_key, payload)

Envía `topic\\npayload_json\\n` al firmware (protocolo serialCommandTask).
"""
function send_command!(sys::ThermalSystem, topic_key::String, payload::Dict)
    topic = sys.topics[topic_key]
    raw = topic * "\n" * JSON3.write(payload) * "\n"
    write(sys.sp, raw)
    return nothing
end


# ── Recepción síncrona de muestras ───────────────────────────────────────────

"""
    receive_samples!(sys, total_points, on_sample; timeout)

Bloquea hasta recibir `total_points` muestras del ESP32.
Invoca `on_sample(msg, np_val)` por cada muestra.

A diferencia del motor DC (buffer de 20 muestras), el firmware térmico
envía **una muestra por tiempo de muestreo** con el campo `np`.
"""
function receive_samples!(sys::ThermalSystem, total_points::Int, on_sample::Function;
                          timeout::Float64 = 120.0)
    sync     = false
    received = 0

    while received < total_points
        msg    = _wait_sample(sys, timeout)
        np_val = hex2long(string(msg["np"]))

        if np_val ==0
            sync = true
        end
        if sync
            on_sample(msg, np_val)
            received += 1
        end
    end
    return nothing
end

function _wait_sample(sys::ThermalSystem, timeout::Float64)
    deadline = time() + timeout
    while time() < deadline
        if isready(sys.queue)
            return take!(sys.queue)
        end
        sleep(0.01)
    end
    throw(ErrorException(
        "No se recibieron datos del ESP32. " *
        "Verifique la conexión y que el firmware esté en ejecución."))
end


# ── Modelo de la planta ──────────────────────────────────────────────────────

"""
    transfer_function(sys)

Retorna la función de transferencia del sistema térmico.
Carga los parámetros del último experimento PRBS guardado en datafiles/.
Si no existe el archivo usa parámetros nominales.
"""
function transfer_function(sys::ThermalSystem) 
    b, a = 0.03150185, 0.02812272 
    return ControlSystems.tf(b, [1.0 ,a])
end


# ── Utilidades internas ──────────────────────────────────────────────────────

"""Guarda vectores de datos como CSV con encabezado."""
function _save_exp(vectors::Vector{Vector{Float64}}, filename::String, header::String)
    mat = reduce(hcat, vectors)    
    open(PATH_DATA * filename, "w") do io
        println(io, header)
        for i in axes(mat, 1)
            println(io, join((@sprintf("%.8f", mat[i, j]) for j in axes(mat, 2)), ","))
        end
    end    
end

"""Fuerza redibujado de la gráfica en tiempo real."""
function redraw!(plt)
    try
        display(plt)
    catch
    end
end
