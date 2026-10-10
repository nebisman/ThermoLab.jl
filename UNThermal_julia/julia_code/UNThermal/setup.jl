# ═══════════════════════════════════════════════════════════════════════════════
#  setup.jl – Instala todas las dependencias necesarias para UNThermal.jl
#
#  Ejecutar UNA VEZ desde la carpeta UNThermal/:
#      julia setup.jl
#  O desde el REPL de Julia:
#      include("/ruta/a/UNThermal/setup.jl")
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg

println("═" ^ 60)
println("  Configurando UNThermal.jl")
println("═" ^ 60)

pkg_dir = @__DIR__

println("\n→ Activando proyecto en: $pkg_dir")
Pkg.activate(pkg_dir)


# Agregar dependencias — Pkg.add resuelve los UUIDs automáticamente
deps = [
    "LibSerialPort",    # Comunicación serial con ESP32
    "JSON3",            # Parsing/serialización JSON
    "Interpolations",   # Interpolación para curva estática
    "DelimitedFiles",   # Lectura/escritura de CSV simples
    "ControlSystems",   # Funciones de transferencia, espacio de estados
    "ControlSystemsBase", # funciones de estado
    "Plots",            # Gráficas en tiempo real
    "Statistics",       # mean, std (stdlib)
    "Printf",          # @sprintf, @printf (stdlib)
    "LinearAlgebra",    # Álgebra lineal (stdlib)
    "CSV",              # Lectura avanzada de CSV
    "DataFrames",       # DataFrames para análisis   
    "RobustAndOptimalControl", # Funciones avanzadas de control robusto y óptimo
    "DSP",               #  para procesamiento dwe señales 
    "ControlSystemIdentification",   # para identificacion de sistemas
    "LaTeXStrings"
]

println("\n→ Instalando $(length(deps)) dependencias...")
for pkg in deps
    print("  • $pkg ... ")
    try
        Pkg.add(pkg)
        println("✓")
    catch e
        println("✗  ($e)")
    end
end

println("\n→ Resolviendo versiones...")
Pkg.resolve()

println("\n→ Precompilando (puede tardar varios minutos)...")
Pkg.precompile()

println("\n" * "═" ^ 60)
println("  ✓ Instalación completada.")
println()
println("  Para usar UNThermal en Julia:")
println()
println("    import Pkg")
println("    Pkg.activate(\"$pkg_dir\")")
println("    using UNThermal")
println()
println("  Para instalar globalmente (sin activar cada vez):")
println("    Pkg.develop(path=\"$pkg_dir\")")
println("    using UNThermal")
println("═" ^ 60)
