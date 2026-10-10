using Documenter
using ThermoLab

DocMeta.setdocmeta!(ThermoLab, :DocTestSetup, :(using ThermoLab); recursive=true)

makedocs(;
    modules=[ThermoLab],
    authors="LB, HD",
    sitename="ThermoLab.jl",
    checkdocs=:none,
    format=Documenter.HTML(;
        canonical="https://nebisman.github.io/ThermoLab.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Inicio" => "index.md",
        "Instalación" => [

            "Instalación del firmware" => "instalacion/firmware.md",
            "Instalación del paquete" => "instalacion/paquete.md",
            "Problemas con el puerto serial" => [
                "Permisos del puerto serial en linux" => "instalacion/permisos_serial_linux.md",
                "Driver CH340 en Windows" => "instalacion/driver_ch340_windows.md",
            ],
        ],
        "Funciones" => [
            "Utilidades" => [
                "`ThermalSystem`" => "funciones/informacion/ThermalSystem.md",
                "`find_port`" => "funciones/informacion/find_port.md",
                "`temp_from_power`" => "funciones/informacion/temp_from_power.md",
                "`power_from_temp`" => "funciones/informacion/power_from_temp.md",
                "`get_last_model`" => "funciones/informacion/get_last_model.md",
                "`tf`" => "funciones/informacion/tf.md",
                "`ss`" => "funciones/informacion/ss.md",
            ],
            "Identificación" => [
                "`get_static_model`" => "funciones/identificacion/get_static_model.md",
                "`step_open`" => "funciones/identificacion/step_open.md",
                "`prbs_open`" => "funciones/identificacion/prbs_open.md",
                "`get_model_step`" => "funciones/identificacion/get_model_step.md",
                "`get_model_prbs`" => "funciones/identificacion/get_model_prbs.md",
            ],
            "Control" => [
                "`set_reference`" => "funciones/control/set_reference.md",
                "`set_pid`" => "funciones/control/set_pid.md",
                "`set_controller`" => "funciones/control/set_controller.md",
                "`step_closed`" => "funciones/control/step_closed.md",
                "`stepinfo`" => "funciones/control/stepinfo.md",
                "`stairs_closed`" => "funciones/control/stairs_closed.md",
                "`profile_closed`" => "funciones/control/profile_closed.md",
            ],
            "Diseño de controladores" => [
                "`cont1dof`" => "funciones/diseno/cont1dof.md",
                "`cont2dof`" => "funciones/diseno/cont2dof.md",
                "`lqmodel`" => "funciones/diseno/lqmodel.md",
            ],
        ],
    ],
)

deploydocs(;
    repo="github.com/nebisman/ThermoLab.jl",
    devbranch="main",
)
