# ═══════════════════════════════════════════════════════════════════════════════
#  UNThermal.jl – Módulo principal para control e identificación del sistema
#  térmico controlado por ESP32 vía comunicación serial (USB).
#
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

module UNThermal

# ── Dependencias externas ────────────────────────────────────────────────────
using LibSerialPort
using JSON3
using DelimitedFiles
using ControlSystems
using ControlSystemsBase
using LinearAlgebra
using Plots
using Statistics
using Printf
using RobustAndOptimalControl
using ControlSystemIdentification
using DSP
using LaTeXStrings



# ── Código fuente ────────────────────────────────────────────────────────────
include("thermalsys.jl")
include("controlsys.jl")
include("identsys.jl")

# ── API pública ──────────────────────────────────────────────────────────────

# Constantes
export SAMPLING_TIME

# Tipo principal
export ThermalSystem

# Conexión
export connect!, disconnect!

# Conversiones hex (utilidades)
export float2hex, hex2float, long2hex, hex2long
export hexframe_to_array, matrix2hex

# Lectura de archivos
export read_csv_file, read_csv_file3, read_csv_file4

# Comandos de comunicación
export send_command!, receive_samples!

# Modelos de la planta
export transfer_function

# Funciones de control (controlsys)
export set_reference, set_pid, set_controller
export step_closed, stairs_closed, profile_closed, stepinfo_exp

# Funciones de identificación (identsys)
export step_open, prbs_open
export get_static_model, get_fomodel_step, get_model_prbs

end # module UNThermal
