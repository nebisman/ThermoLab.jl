
include("/home/leonardo/datos/share_desktop/proyecto_julia/UNThermal_julia/julia_code/UNThermal/setup.jl")

t, u, y = read_csv_file3("/home/leonardo/datos/share_desktop/proyecto_julia/UNThermal_julia/julia_code/UNThermal/datafiles/Thermal_prbs_open_exp.csv")
using Statistics
using  ControlSystemIdentification

nx  = 2                         # Number of poles in the true system
nu  = 1                         # Number of control inputs
ny  = 1                         # Number of outputs

    ymean = mean(y)
    ym    = y .- ymean
    um    = u .- mean(u)
    t = t .- t[1]  # ajustar t=0 al inicio de la PRBS


dn = iddata(ym, um, 0.8)
sysh,opt = newpem(dn,nx, iterations=400)

f1 = bodeplot([G,sysh.sys], lab=["True system" "estimated system"])
f2 = predplot(sysh,dn)
f3 = simplot(sysh,dn)
plot(f1,f2,f3, layout=(1,3), size=(900,300))

