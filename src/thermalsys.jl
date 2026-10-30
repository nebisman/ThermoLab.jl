# ═══════════════════════════════════════════════════════════════════════════════
#  thermalsys.jl – Comunicación serial con el sistema térmico ThermoLab (ESP32).
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using LibSerialPort
using JSON3
using DelimitedFiles
using Printf
using Statistics
import ControlSystemsBase: tf, ss, numvec, denvec

# ── Constantes del protocolo ─────────────────────────────────────────────────

"""Periodo de muestreo del firmware, en segundos."""
const SAMPLING_TIME = 0.8

"""Longitud (en bits) de la secuencia PRBS generada por el firmware."""
const PRBS_LENGTH   = 63

"""Referencia de temperatura (°C) que el firmware fija por defecto."""
const DEFAULT_REFERENCE = 40.0

"""Temperatura máxima de referencia (°C) admitida por el paquete."""
const MAX_TEMPERATURE   = 95.0

"""Tamaño de fuente por defecto usado en las gráficas."""
const FONT_SIZE     = 12

# ── Rutas por defecto ────────────────────────────────────────────────────────
"""Nombre del directorio de trabajo donde se guardan los archivos CSV."""
const DATA_DIR = "datafiles_tmlab"

"""Nombre del directorio de trabajo donde se copian los ejemplos del paquete."""
const EXAMPLES_DIR = "ejemplos_tmlab"

# PATH_DATA apunta a la carpeta de trabajo del usuario: la que ya contiene
# datafiles_tmlab/ o ejemplos_tmlab/ (buscando desde el directorio actual hacia
# arriba), o el directorio actual si no se encuentra ninguna.
const PATH_DATA = Ref{String}("")

"""
Busca, desde `inicio` y subiendo hasta `niveles` carpetas, la primera que ya
contenga `datafiles_tmlab/` o `ejemplos_tmlab/`. Así un notebook abierto en,
p. ej., `ejemplos_tmlab/notebooks/` reutiliza las carpetas existentes en vez de
crear otras. Retorna `nothing` si no encuentra ninguna.
"""
function _buscar_raiz_datos(inicio::AbstractString = pwd(); niveles::Int = 2)
    dir = abspath(inicio)
    for _ in 0:niveles
        if isdir(joinpath(dir, DATA_DIR)) || isdir(joinpath(dir, EXAMPLES_DIR))
            return dir
        end
        padre = dirname(dir)
        padre == dir && break      # se llegó a la raíz del sistema de archivos
        dir = padre
    end
    return nothing
end

"""Ruta absoluta de un archivo dentro de PATH_DATA/datafiles_tmlab."""
_datafile(name::AbstractString) = joinpath(PATH_DATA[], DATA_DIR, name)

"""
Crea (si no existen) los directorios "datafiles_tmlab" y "ejemplos_tmlab" dentro
de PATH_DATA, y los puebla con los archivos del paquete instalado. Si ya
existen, no hace nada.
"""
function _ensure_dirs()
    pkg_root = joinpath(@__DIR__, "..")

    datafiles_dst = joinpath(PATH_DATA[], DATA_DIR)
    if !isdir(datafiles_dst)
        mkpath(datafiles_dst)
        datafiles_src = joinpath(pkg_root, DATA_DIR)
        for f in filter(f -> endswith(f, ".csv"), readdir(datafiles_src))
            dst = joinpath(datafiles_dst, f)
            cp(joinpath(datafiles_src, f), dst; force=true)
            chmod(dst, 0o644)
        end
    end

    ejemplos_dst = joinpath(PATH_DATA[], EXAMPLES_DIR)
    if !isdir(ejemplos_dst)
        ejemplos_src = joinpath(pkg_root, EXAMPLES_DIR)
        cp(ejemplos_src, ejemplos_dst; force=true)
        # cp() conserva los permisos de origen; si el paquete fue instalado
        # vía Pkg, los archivos en ~/.julia/packages quedan de solo lectura,
        # por lo que hay que asegurar que la copia del usuario sea escribible.
        for (root, _, files) in walkdir(ejemplos_dst)
            for f in files
                chmod(joinpath(root, f), 0o644)
            end
        end
    end
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Funciones de conversión hex  (idénticas al protocolo del firmware)
#  El firmware usa big-endian IEEE-754 float32 y big-endian uint32.
#  Nota: reinterpret(UInt32, Float32(x)) devuelve el bit-pattern IEEE-754,
#  que coincide con la representación big-endian usada por Python struct.pack('>f').
# ═══════════════════════════════════════════════════════════════════════════════

"""
    float2hex(value::Real) -> String

Convierte `value` a `Float32` y retorna su patrón de bits IEEE-754 como un
string hexadecimal de 8 caracteres (formato usado por el firmware).

# Ejemplo
```julia
float2hex(1.0)

    "3f800000"
```
"""
function float2hex(value::Real)::String
    u = reinterpret(UInt32, Float32(value))
    return string(u; base=16, pad=8)
end

"""
    hex2float(h::AbstractString) -> Float64

Interpreta el string hexadecimal de 8 caracteres `h` como el patrón de bits
IEEE-754 de un `Float32` y lo retorna como `Float64`. Es la inversa de
[`float2hex`](@ref).
"""
function hex2float(h::AbstractString)::Float64
    u = parse(UInt32, h; base=16)
    return Float64(reinterpret(Float32, u))
end

"""
    long2hex(value::Integer) -> String

Convierte `value` a `UInt32` y lo retorna como un string hexadecimal de 8
caracteres. Lanza `InexactError` si `value` no cabe en un `UInt32`.
"""
function long2hex(value::Integer)::String
    return string(UInt32(value); base=16, pad=8)
end

"""
    hex2long(h::AbstractString) -> Int

Interpreta el string hexadecimal `h` como un `UInt32` y lo retorna como `Int`.
Es la inversa de [`long2hex`](@ref).
"""
function hex2long(h::AbstractString)::Int
    return Int(parse(UInt32, h; base=16))
end

"""
    signal2hex(signal) -> String

Concatena, sin separadores, la representación [`float2hex`](@ref) de cada
elemento de `signal`.
"""
function signal2hex(signal)::String
    return join(float2hex(v) for v in signal)
end

"""
    time2hex(time_points) -> String

Concatena, sin separadores, la representación [`long2hex`](@ref) de cada
elemento de `time_points` (índices de muestra enteros).
"""
function time2hex(time_points)::String
    return join(long2hex(Int(t)) for t in time_points)
end

"""
    matrix2hex(mat) -> String

Concatena, sin separadores, la representación [`float2hex`](@ref) de los
elementos de `mat`, recorriéndola fila por fila. Si `mat` es un vector, se
recorre en orden.
"""
function matrix2hex(mat)::String
    if ndims(mat) == 1
        return join(float2hex(e) for e in mat)
    else
        return join(float2hex(mat[i, j]) for i in axes(mat, 1) for j in axes(mat, 2))
    end
end

"""
    hexframe_to_array(hexframe::AbstractString) -> Vector{Float64}

Convierte un string de valores hexadecimales separados por comas (cada uno
con el formato de [`float2hex`](@ref)) en un vector de `Float64`.
"""
function hexframe_to_array(hexframe::AbstractString)::Vector{Float64}
    return [hex2float(strip(p)) for p in split(hexframe, ",")]
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Lectura de archivos CSV
# ═══════════════════════════════════════════════════════════════════════════════

"""
    read_csv_file(filepath=datafiles_tmlab/ThermoLab_static_gain_response.csv) -> (u, y)

Lee un archivo CSV con encabezado y retorna sus dos primeras columnas como
vectores `Float64`. Por defecto lee la curva estática guardada por
[`get_static_model`](@ref) (potencia `u` en % y temperatura `y` en °C).
"""
function read_csv_file(filepath::String = _datafile("ThermoLab_static_gain_response.csv"))
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2])
end

"""
    read_csv_file3(filepath::String) -> (t, u, y)

Lee un archivo CSV con encabezado y retorna sus tres primeras columnas como
vectores `Float64`; por ejemplo, los archivos `t,u,y` guardados por
[`step_open`](@ref) y [`prbs_open`](@ref).
"""
function read_csv_file3(filepath::String)
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2]), Float64.(data[:, 3])
end

"""Lee la primera fila de datos (tras el encabezado) de un CSV de parámetros. Retorna un vector."""
function read_model(filepath::String)
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[1, :])
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Estructura principal:  ThermalSystem
# ═══════════════════════════════════════════════════════════════════════════════

"""
    ThermalSystem(; port=find_port(), bauds=460800)

Objeto que representa la interfaz entre Julia y la plataforma
ThermoLab (basada en un microcontrolador ESP32). Todas las funciones de comunicación, control e
identificación del paquete reciben una instancia de `ThermalSystem` como primer
argumento.



# Argumentos de palabra clave
- `port::String=find_port()`: nombre del puerto serial al que está
  conectada la placa ESP32 (por ejemplo: `"/dev/ttyUSB0"` en Linux, o  `"COM3"`
  en Windows). Por defecto se detecta automáticamente con [`find_port`](@ref);
  *solo indíquelo manualmente si hay varias placas conectadas o si la detección falló*.

- `bauds::Int=460800`: velocidad de transmisión (baudios) del puerto serial;
  debe coincidir con la configurada en el firmware. Deje este valor por defecto
  a menos que se haya tenido que modificar la velocidad en el firmware.

# Ejemplos
```julia
using ThermoLab
sys = ThermalSystem();                      # puerto detectado automáticamente
sys = ThermalSystem(port="/dev/ttyUSB0");   # puerto indicado manualmente
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
set_reference(sys, 50)
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
            port::String         = find_port(),
            bauds::Int           = 460800)

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

"""
    find_port()

Detecta automáticamente el puerto serial al cual está conectada la plataforma ThermoLab.

Recorre los puertos USB-serial visibles en sistema y selecciona aquellos compatibles
con la placa ESP32 en que está basada la plataforma ThermoLab.

# Retorna
- `String`: nombre del puerto serial detectado (por ejemplo, `"/dev/ttyUSB0"`
  en Linux o `"COM3"` en Windows).

# Lanza
- `ErrorException` si no se encuentra ningún puerto candidato, o si se
  encuentra más de uno y no es posible determinar cuál corresponde a la
  placa. En ambos casos, debe indicar el puerto manualmente con
  `ThermalSystem(port="...")` o revisar si la placa está conectada.

# Ejemplos
```julia
using ThermoLab
port = find_port()

    "/dev/ttyUSB0"
```

Nótese que `ThermalSystem()` ya llama a `find_port()` internamente si no se
indica `port`, por lo que en la mayoría de casos no es necesario invocarla de forma
explícita:

```julia
using ThermoLab
sys = ThermalSystem();
```
"""
function find_port()
    known_vid_pid = Set{Tuple{UInt16,UInt16}}([
        (0x10C4, 0xEA60),  # Silicon Labs CP2102/CP2104
        (0x1A86, 0x7523),  # WCH CH340/CH341
        (0x1A86, 0x55D4),  # WCH CH9102
        (0x0403, 0x6001),  # FTDI FT232
        (0x303A, 0x1001),  # Espressif USB JTAG/serial debug unit
    ])
    known_keywords = ("cp210", "ch340", "ch9102", "wch", "silicon labs",
                       "espressif", "esp32", "usb-serial", "usb serial", "uart")

    candidates = String[]
    ports = LibSerialPort.sp_list_ports()
    try
        i = 1
        while true
            port = unsafe_load(ports, i)
            port == C_NULL && break
            if LibSerialPort.sp_get_port_transport(port) == LibSerialPort.SP_TRANSPORT_USB
                manufacturer = LibSerialPort.sp_get_port_usb_manufacturer(port)
                product      = LibSerialPort.sp_get_port_usb_product(port)
                description  = LibSerialPort.sp_get_port_description(port)
                vid, pid     = LibSerialPort.sp_get_port_usb_vid_pid(port)

                text = lowercase(join((manufacturer, product, description), " "))
                matched = (UInt16(vid), UInt16(pid)) in known_vid_pid ||
                          any(kw -> occursin(kw, text), known_keywords)
                matched && push!(candidates, LibSerialPort.sp_get_port_name(port))
            end
            i += 1
        end
    finally
        LibSerialPort.sp_free_port_list(ports)
    end

    if isempty(candidates)
        error(
        "No se encontró ningún puerto USB-serial compatible con la plataforma" *
        " ThermoLab. Verifique que la plataforma esté conectada, o " *
        "indique el puerto manualmente con ThermalSystem(port=\"...\").")
    elseif length(candidates) > 1
        error(
        "Se encontraron varios puertos candidatos: " *
        join(candidates, ", ") *
        ". Indique el puerto deseado manualmente con ThermalSystem(port=\"...\").")
    end
    return candidates[1]
end

"""
    connect!(sys::ThermalSystem)

Abre el puerto serial de `sys` (si no está abierto), descarta los bytes
pendientes en el buffer de entrada e inicia la tarea que lee en segundo plano
las muestras enviadas por el firmware.

Normalmente no es necesario llamarla: las funciones de control e
identificación la invocan por sí mismas.
"""
function connect!(sys::ThermalSystem)
    if sys.sp === nothing
        sys.sp = LibSerialPort.open(sys.port, sys.bauds)
        # Limpiar buffer de entrada (ignorar si falla)
        try sp_flush(sys.sp, SP_BUF_INPUT) catch end
    end
    _start_reader!(sys)
    return nothing
end

"""
    disconnect!(sys::ThermalSystem)

Detiene la tarea lectora en segundo plano (esperando hasta 3 s a que
termine) y cierra el puerto serial de `sys`, si está abierto.
"""
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
        return  # ya corriendo
    end
    sys.stop_reader[] = false
    # Vaciar la cola de datos viejos
    while isready(sys.queue)
        take!(sys.queue)
    end
    sys.reader_task = Threads.@spawn _reader_loop(sys)
    return nothing
end

function _stop_reader!(sys::ThermalSystem)
    sys.stop_reader[] = true
    if sys.reader_task !== nothing
        # Esperar hasta 3 segundos
        t0 = time()
        while !istaskdone(sys.reader_task) && (time() - t0) < 3.0
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
            # Leer bytes disponibles del serial
            if sys.sp === nothing
                break
            end
            nb = bytesavailable(sys.sp)
            if nb == 0
                sleep(0.005)
                continue
            end
            bytes = read(sys.sp, nb)
            write(buf, bytes)

            # Procesar líneas completas
            raw = String(take!(buf))
            lines = split(raw, '\n')
            # La última parte puede ser incompleta → devolverla al buffer
            if !endswith(raw, '\n')
                write(buf, lines[end])
                lines = lines[1:end-1]
            end

            for line in lines
                line = strip(line)
                isempty(line) && continue

                # Extraer la parte JSON
                json_str = if startswith(line, "D:")
                    line[3:end]
                elseif startswith(line, "{")
                    line
                else
                    continue  # línea de debug → ignorar
                end

                msg = try
                    JSON3.read(json_str, Dict{String, Any})
                catch
                    continue
                end

                # Solo muestras con campo "np" (número de muestra)
                if haskey(msg, "np")
                    try
                        put!(sys.queue, msg)
                    catch
                        break
                    end
                end
            end
        catch e
            if e isa InterruptException
                break
            end
            # Otros errores → continuar
            sleep(0.01)
        end
    end
end

# ── Envío de comandos ────────────────────────────────────────────────────────

"""
    send_command!(sys::ThermalSystem, topic_key::String, payload::Dict)

Envía un comando al firmware con el protocolo de la tarea `serialCommandTask`:
una línea con el tópico y otra con `payload` serializado como JSON
(`topic\\npayload_json\\n`). El puerto debe estar abierto (ver
[`connect!`](@ref)).

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.
- `topic_key::String`: clave del tópico en `sys.topics`: `"set_pid"`,
  `"set_ref"`, `"set_gencon"`, `"step_closed"`, `"stairs_closed"`,
  `"prof_closed"`, `"step_open"` o `"prbs_open"`.
- `payload::Dict`: parámetros del comando. Los valores reales y enteros se
  envían codificados con [`float2hex`](@ref) y [`long2hex`](@ref).
"""
function send_command!(sys::ThermalSystem, topic_key::String, payload::Dict)
    topic = sys.topics[topic_key]
    raw = topic * "\n" * JSON3.write(payload) * "\n"
    write(sys.sp, raw)
    return nothing
end

# ── Recepción síncrona de muestras ───────────────────────────────────────────

"""
    receive_samples!(sys::ThermalSystem, total_points::Int, on_sample::Function; timeout=5.0)

Bloquea hasta recibir `total_points` muestras del experimento en curso. El
firmware envía **una muestra por periodo de muestreo**, numerada con el campo
`np`, que reinicia en 0 al iniciar cada experimento. Las muestras anteriores a
la de `np == 0` (restos del experimento previo) se descartan; a partir de ella
se invoca `on_sample(msg, np)` por cada muestra, hasta recibir la de
`np == total_points - 1`.

# Argumentos
- `sys::ThermalSystem`: objeto conectado (ver [`connect!`](@ref)).
- `total_points::Int`: número de muestras del experimento.
- `on_sample::Function`: función `on_sample(msg, np)` que recibe el mensaje
  como `Dict{String,Any}` (campos codificados en hexadecimal, p. ej. `"y"`,
  `"u"` y `"r"`; ver [`hex2float`](@ref)) y el número de muestra `np::Int`.

# Argumentos de palabra clave
- `timeout::Float64=5.0`: tiempo máximo, en segundos, de espera entre muestras.

# Lanza
- `ErrorException` si pasan más de `timeout` segundos sin recibir datos.
"""
function receive_samples!(sys::ThermalSystem, total_points::Int, on_sample::Function;
                          timeout::Float64 = 5.0)
    sync = false
    while true
        msg = _wait_sample(sys, timeout)
        np_val = hex2long(string(msg["np"]))
        sync = sync || np_val == 0
        sync || continue
        on_sample(msg, np_val)
        np_val >= total_points - 1 && break
    end
    return nothing
end



function _wait_sample(sys::ThermalSystem, timeout::Float64)
    deadline = time() + timeout
    while time() < deadline
        if isready(sys.queue)
            return take!(sys.queue)
        end
        sleep(0.005)
    end
    throw(ErrorException(
        "No se recibieron datos del ESP32. " *
        "Verifique la conexión y que el firmware envíe las muestras por Serial."))
end

# ── Modelos de la planta ─────────────────────────────────────────────────────

"""
    get_last_model(sys::ThermalSystem; numpar::Int=2)

Retorna la función de transferencia y el retardo de la plataforma ThermoLab, estimados
previamente con [`get_model_step`](@ref) o [`get_model_prbs`](@ref). La salida del
modelo es la temperatura (°C) y la entrada es la potencia aplicada (% de la potencia máxima).

# Argumentos
- `sys::ThermalSystem`: objeto de la plataforma (no necesita estar conectado;
  solo se usa para ubicar el archivo del modelo).

# Argumentos de palabra clave
- `numpar::Int=2`: número de parámetros del modelo a usar.
  + `numpar=2` (por defecto): modelo de un polo, estimado por [`get_model_step`](@ref)
    o por [`get_model_prbs`](@ref) con `numpar=2`:

    ``G(s)=\\dfrac{b}{s+a}``

  + `numpar=3`: modelo de dos polos, estimado por [`get_model_prbs`](@ref) con `numpar=3`:

    ``G(s)=\\dfrac{b}{(s+a_1)(s+a_2)}``

# Retorna
- `G::ControlSystemsBase.TransferFunction`: función de transferencia continua
  del sistema térmico.
- `L::Float64`: retardo estimado, en segundos. Si el modelo se obtuvo con PRBS el retardo
  es de una muestra (0.8 s).

# Ejemplos
```julia
sys = ThermalSystem();
G, L = get_last_model(sys)              # modelo de 1 polo (por defecto)
G2, L = get_last_model(sys; numpar=3)   # modelo de 2 polos
```
"""
function get_last_model(sys::ThermalSystem; numpar::Int = 2)
    if numpar == 2
        b, a, L = read_model(_datafile("ThermoLab_fo_model_2p.csv"))
        num = [b]
        den = [1.0, a]
    elseif numpar == 3
        b, a1, a2, L = read_model(_datafile("ThermoLab_fo_model_3p.csv"))
        num = [b]
        den = [1.0, a1 + a2, a1 * a2]
    else
        error("numpar debe ser 2 o 3")
    end
    G = tf(num, den)
    return G, L
end


"""
    tf(sys::ThermalSystem; numpar::Int=2)

Retorna la función de transferencia nominal (sin retardo) de la plataforma ThermoLab, estimada
previamente con [`get_model_step`](@ref) o [`get_model_prbs`](@ref).

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `numpar::Int=2`: número de parámetros del modelo. Con `numpar=2` (por
  defecto) se usa el modelo de un polo, dado por:

  ``G(s)=\\dfrac{b}{s+a}``

  Con `numpar=3` se usa el modelo de dos polos, estimado con
  [`get_model_prbs`](@ref):

  ``G(s)=\\dfrac{b}{(s+a_1)(s+a_2)}``

# Retorna
- `G::ControlSystemsBase.TransferFunction`: función de transferencia
   de la plataforma ThermoLab (temperatura / potencia), sin incluir retardo.

# Ejemplos
```julia
sys = ThermalSystem();
G = tf(sys)                 # modelo de 1 polo (por defecto)
G2 = tf(sys; numpar=3)      # modelo de 2 polos
```
"""
function tf(sys::ThermalSystem; numpar::Int = 2)
    G, _ = get_last_model(sys; numpar)
    return G
end


"""
    ss(sys::ThermalSystem; numpar::Int=2)

Retorna la realización en espacio de estado del modelo nominal (sin retardo)
de la plataforma ThermoLab, estimada previamente con [`get_model_step`](@ref)
o [`get_model_prbs`](@ref).

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma.

# Argumentos de palabra clave
- `numpar::Int=2`: número de parámetros del modelo. `numpar=2` (por defecto)
  usa el modelo de un polo; `numpar=3`
  usa el modelo de dos polos estimado con [`get_model_prbs`](@ref).

Con `numpar=2`, la realización retornada es:

   ``\\dot x_1 = - a\\,x_1 + b\\,u, \\qquad y =  x_1``

Con `numpar=3` (modelo de dos polos ``a_1``, ``a_2``), la realización retornada es:

   ``\\begin{bmatrix} \\dot x_1 \\\\ \\dot x_2 \\end{bmatrix} = \\begin{bmatrix} - a_1 &  0 \\\\  1 & -a_2 \\end{bmatrix} \\begin{bmatrix}  x_1 \\\\  x_2 \\end{bmatrix} +  \\begin{bmatrix} b \\\\ 0 \\end{bmatrix} u, \\qquad y =  \\begin{bmatrix} 0 & 1 \\end{bmatrix}\\begin{bmatrix}  x_1 \\\\  x_2 \\end{bmatrix}``

# Retorna
- `sys_ss::ControlSystemsBase.StateSpace`: realización continua en espacio de
  estado de la plataforma ThermoLab.

# Nota
- Los parámetros usados en las realizaciones se toman de los valores estimados por medio de las funciones [`get_model_step`](@ref) o [`get_model_prbs`](@ref).

# Ejemplos
```julia
sys = ThermalSystem();
G_ss = ss(sys)                 # modelo de 1 polo (por defecto)
G_ss2 = ss(sys; numpar=3)      # modelo de 2 polos
```
"""
function ss(sys::ThermalSystem; numpar::Int = 2)
    if numpar == 2
        b, a, _ = read_model(_datafile("ThermoLab_fo_model_2p.csv"))
        A = fill(-a, 1, 1)
        B = fill(b, 1, 1)
        C = fill(1.0, 1, 1)
    elseif numpar == 3
        b, a1, a2, _ = read_model(_datafile("ThermoLab_fo_model_3p.csv"))
        A = [-a1 0; 1 -a2]
        B = [b; 0;;]
        C = [0 1]
    else
        error("numpar debe ser 2 o 3")
    end
    D = 0
    return ss(A, B, C, D)
end


"""
    temp_from_power(sys::ThermalSystem, power::Real)

Estima la temperatura de estado estacionario (en °C) que alcanza la plataforma ThermoLab
al aplicarle una potencia constante `power`, expresada en porcentaje.

Para ello usa el modelo estático lineal (obtenido con [`get_static_model`](@ref)):

``\\qquad \\text{\\texttt{temp}} = K\\cdot \\text{\\texttt{power}} + b``,



# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma

- `power::Real`: potencia aplicada, en % de la potencia máxima. Debe cumplir que

    ``0 \\leq`` `power` ``\\leq  100``.

# Retorna
- `Float64`: temperatura estacionaria estimada (en °C) que se produce al aplicar
  la potencia especificada en la variable `power`.

# Lanza
- `ErrorException` si `power` está fuera del intervalo ``[0, 100]``.

# Ejemplos
```julia
using ThermoLab
sys = ThermalSystem();
temp_from_power(sys, 20.0)
```

El valor depende del modelo estático previamente obtenido.
"""
function temp_from_power(sys::ThermalSystem, power::Real)
    0 <= power <= 100 || error("power debe estar en [0, 100] %")
    K, b = read_model(_datafile("ThermoLab_static_pars.csv"))
    return K * power + b
end

"""
    power_from_temp(sys::ThermalSystem, temp::Real)

Estima la potencia  (en % de la potencia máxima) necesaria para que la plataforma
ThermoLab alcance, en estado estacionario, la temperatura `temp` (en °C). Es la función
inversa de [`temp_from_power`](@ref) y usa el mismo modelo estático lineal obtenido con
[`get_static_model`](@ref):

``\\qquad \\text{\\texttt{power}} =\\dfrac{\\text{\\texttt{temp}} - b}{K}``

# Argumentos
- `sys::ThermalSystem`: objeto que representa la plataforma
- `temp::Real`: temperatura de estado estacionario especificada (en °C). Debe cumplir que

   ``b \\leq`` `temp` ``\\leq 95\\,°C`` (`MAX_TEMPERATURE`)

   donde ``b`` es la temperatura estimada sin potencia aplicada (temperatura ambiente).

# Retorna
- `Float64`: potencia estimada (en %) requerida para alcanzar la temperatura
  de estado estacionario definida en la variable `temp`, saturada al
  intervalo ``[0, 100]``.

# Lanza
- `ErrorException` si `temp` está fuera del intervalo indicado.

# Ejemplos
```julia
using ThermoLab
sys = ThermalSystem();
power_from_temp(sys, 50.0)
```

El valor depende del modelo estático guardado en la carpeta `datafiles_tmlab`.
"""
function power_from_temp(sys::ThermalSystem, temp::Real)
    K, b = read_model(_datafile("ThermoLab_static_pars.csv"))
    if !(b <= temp <= MAX_TEMPERATURE)
        error("temp debe estar entre $(round(b, digits=1)) y $(MAX_TEMPERATURE) °C")
    end
    return clamp((temp - b) / K, 0.0, 100.0)
end


# ── Guardar experimentos ─────────────────────────────────────────────────────

"""
Guarda los vectores de `data` como columnas de un CSV `filename` en
PATH_DATA/datafiles_tmlab, con la línea de encabezado `header` y 8 decimales.
"""
function save_experiment(data::Vector, filename::String, header::String)
    mat = hcat(data...)
    open(_datafile(filename), "w") do io
        println(io, header)
        for i in axes(mat, 1)
            println(io, join([@sprintf("%.8f", mat[i, j]) for j in axes(mat, 2)], ","))
        end
    end
    return nothing
end
