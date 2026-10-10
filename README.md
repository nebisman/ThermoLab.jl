# ThermoLab

## Laboratorio portátil de control de temperatura.

[![Build Status](https://github.com/nebisman/ThermoLab.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/nebisman/ThermoLab.jl/actions/workflows/CI.yml?query=branch%3Amain)

Este paquete de Julia permite hacer experimentos de control e identificación con la plataforma portátil de experimentación **ThermoLab**. Esta plataforma integra los componentes para controlar la temperatura de un sistema térmico (un calefactor y un sensor de temperatura DS18B20) por medio de un microcontrolador ESP32.

Desde una perspectiva de control y sistemas dinámicos, la plataforma permite representar vívidamente varios conceptos, a saber:

- Representa la respuesta de un sistema de primer orden estable, con una constante de tiempo del orden de decenas de segundos.
- Permite estudiar modelos de mayor orden (dos polos reales) y el efecto del retardo debido al muestreo digital.
- Permite observar la saturación de la señal de control (la potencia aplicada está limitada entre 0 y 100 %) y el efecto de los mecanismos *anti-windup*.

Adicionalmente, tres LEDs RGB muestran con un mapa de color la temperatura, la referencia y la potencia aplicada, lo que hace que el experimento sea muy visual e ilustrativo.

## Instalación

Las instrucciones de instalación del paquete, sus funciones y la instalación del *firmware* de la plataforma se encuentran en la
[documentación](https://nebisman.github.io/ThermoLab.jl).

Uso básico:

```julia
using Pkg
Pkg.add(url="https://github.com/nebisman/ThermoLab.jl")

using ThermoLab
sys = ThermalSystem();
set_pid(sys; kp=11.8, ki=2.7, kd=0, beta=0)
set_reference(sys, 50)
```
