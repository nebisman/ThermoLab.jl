# ═══════════════════════════════════════════════════════════════════════════════
#  Utilidades de plotting en tiempo real
# ═══════════════════════════════════════════════════════════════════════════════

#using Plots

"""
    setup_fig(nrows; figsize=(900,500))

Crea y muestra una figura de Plots.jl con `nrows` subplots verticales.
Retorna el objeto plot.
"""
function setup_fig(nrows::Int; figsize=(900, 500))
    # Se usan layouts de Plots.jl
    plt = plot(layout=(nrows, 1), size=figsize, legend=:bottomright,
               background_color=:white, margin=5Plots.mm)
    display(plt)
    return plt
end




# ═══════════════════════════════════════════════════════════════════════════════
#  Visualización en tiempo real unificada: REPL, Jupyter y Pluto.
#
#  Los experimentos arman la figura en su `on_sample` y llaman `redraw!(plt)`;
#  lo único que varía según el entorno es a dónde va a parar la figura:
#
#      REPL     -> display(plt)                    (ventana GR, se redibuja sola)
#      Jupyter  -> clear_output(true) + display    (misma salida de la celda)
#      Pluto    -> guarda la última figura; la celda `screen_pluto()` la sondea
#
#      # celda 1
#      using ThermoLab
#
#      # celda 2 — la pantalla (una sola vez en todo el notebook)
#      screen_pluto()
#
#      # celda 3 — cualquier experimento, sin cambios
#      t, r, y, u = step_closed(sys; r0=40, r1=50, t0=60, t1=120)
#
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using Base64
using HypertextLiteral: @htl
import AbstractPlutoDingetjes

# ── Detección del entorno ────────────────────────────────────────────────────

_ijulia() = isdefined(Main, :IJulia) ? getfield(Main, :IJulia) : nothing

"""
    entorno() -> Symbol

Identifica dónde se está ejecutando el código, para decidir cómo mostrar cada
figura: retorna `:pluto` en un notebook de Pluto, `:jupyter` si IJulia está
cargado, y `:script` en cualquier otro caso (REPL, VS Code o script).
"""
entorno() = isdefined(Main, :PlutoRunner) ? :pluto :
            (_ijulia() !== nothing       ? :jupyter : :script)

# ── Última figura publicada (solo la usa Pluto) ──────────────────────────────
#
#  Se guarda solo la figura más reciente junto con un número de versión. La
#  pantalla pregunta "¿hay algo más nuevo que la versión v?" y la respuesta es
#  inmediata: nunca se deja una tarea del worker bloqueada esperando figuras.

const _LOCK_FIG    = ReentrantLock()
const _FIG         = Ref{Any}(nothing)
const _FIG_VERSION = Ref(0)
const _PNG_CACHE   = Ref((0, ""))      # (versión, data URL) ya renderizada

"Guarda la figura como la más reciente y cede el control para que la pantalla la recoja."
function _publicar_figura(plt)
    lock(_LOCK_FIG) do
        _FIG[] = plt
        _FIG_VERSION[] += 1
    end
    yield()
    return nothing
end

"Devuelve la figura más reciente si es posterior a `desde`; `nothing` si no hay nada nuevo."
function _figura_nueva(desde::Real)
    plt, v = lock(() -> (_FIG[], _FIG_VERSION[]), _LOCK_FIG)
    (plt === nothing || v <= desde) && return nothing
    v_cache, src = _PNG_CACHE[]
    if v_cache != v
        src = _png_data_url(plt)
        _PNG_CACHE[] = (v, src)
    end
    return Dict("version" => v, "src" => src)
end

# ── redraw!: el único punto de contacto con los experimentos ─────────────────

"""
    redraw!(plt)

Muestra la figura `plt` en el destino que corresponda al entorno (ver
[`entorno`](@ref)): con `display` en el REPL o en un script; reemplazando la
salida de la celda en Jupyter; y publicándola para [`screen_pluto`](@ref) en
Pluto. Todos los experimentos la usan para actualizar sus gráficas en tiempo
real. Retorna `nothing`.
"""
function redraw!(plt)
    ent = entorno()
    if ent === :pluto
        _publicar_figura(plt)
    elseif ent === :jupyter
        flush(stdout)                                        # texto pendiente
        ij = _ijulia()
        Base.invokelatest(getfield(ij, :clear_output), true) # borra la salida anterior
        display(plt)
    else
        display(plt)
    end
    return nothing
end

# ── pantalla(): el monitor para Pluto ────────────────────────────────────────

"Serializa una figura como data URL PNG para el atributo src de un <img>."
function _png_data_url(fig)
    io = IOBuffer()
    show(io, MIME"image/png"(), fig)
    return "data:image/png;base64," * base64encode(take!(io))
end

"¿Expone esta versión de AbstractPlutoDingetjes el puente JS <-> Julia?"
_hay_enlace_js() = isdefined(AbstractPlutoDingetjes, :Display) &&
                   isdefined(AbstractPlutoDingetjes.Display, :with_js_link)

"""
    screen_pluto(; alto=nothing)

Widget de monitoreo para Pluto: una celda que muestra cada figura que los
experimentos publican con [`redraw!`](@ref). Créela UNA vez, en su propia celda
(de preferencia antes de las celdas de experimentos), y corra los experimentos
en las celdas que quiera; la pantalla se actualiza sola.

# Argumentos de palabra clave
- `alto=nothing`: altura de la imagen, en píxeles. Con `nothing` la imagen
  conserva su proporción y ocupa el ancho disponible.

# Retorna
- En Pluto, el widget HTML; fuera de Pluto, `nothing` (no hace falta:
  `redraw!` muestra directo en la ventana de gráficos o en la celda de Jupyter).

# Lanza
- `ErrorException` si la versión instalada de AbstractPlutoDingetjes no
  expone `Display.with_js_link`.

# Ejemplo
```julia
# celda 1
using ThermoLab
sys = ThermalSystem()

# celda 2
screen_pluto()

# celda 3
t, r, y, u = step_closed(sys; r0=40, r1=50, t0=60, t1=120)
```
"""
function screen_pluto(; alto = nothing)
    entorno() === :pluto || return nothing   # fuera de Pluto la pantalla es implícita

    _hay_enlace_js() || error("""
        Esta versión de AbstractPlutoDingetjes no expone Display.with_js_link.
        Actualícela en el entorno del paquete:  pkg> up AbstractPlutoDingetjes""")

    # Responde de inmediato: nunca deja una tarea del worker esperando figuras,
    # para no interferir con los sliders ni con la ejecución de otras celdas.
    enlace = AbstractPlutoDingetjes.Display.with_js_link(_figura_nueva)

    estilo = alto === nothing ? "max-width:100%; display:block" :
                                "max-width:100%; height:$(alto)px; display:block"

    return @htl("""
    <div style="font-family: sans-serif">
      <img style=$(estilo)>
      <span style="color:#666; font-size:0.85em">pantalla lista — esperando experimento…</span>
    </div>
    <script>
      const cont  = currentScript.parentElement
      const img   = cont.querySelector("img")
      const lbl   = cont.querySelector("span")
      const pedir = $(enlace)

      let vivo = true
      invalidation.then(() => { vivo = false })

      // Sondeo corto con una sola petición en vuelo: rápido mientras llegan
      // tramas, cada vez más lento cuando no hay nada nuevo.
      const MIN = 60, MAX = 800
      let version = 0, espera = MIN

      const ciclo = async () => {
        if (!vivo) return
        try {
          const r = await pedir(version)
          if (!vivo) return
          if (r != null) {
            version = r.version
            img.src = r.src
            lbl.textContent = "trama " + version
            espera = MIN
          } else {
            espera = Math.min(MAX, espera * 1.5)
          }
        } catch (err) {
          lbl.textContent = "enlace cerrado: " + err
          console.error(err)
          return
        }
        setTimeout(ciclo, espera)
      }
      ciclo()
    </script>
    """)
end