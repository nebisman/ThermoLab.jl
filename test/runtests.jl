using ThermoLab
using Test

@testset "ThermoLab.jl" begin
    sys = ThermalSystem(port="")

    @testset "Conversiones hex" begin
        @test hex2float(float2hex(3.25)) == 3.25
        @test hex2long(long2hex(1575)) == 1575
        @test hexframe_to_array(float2hex(1.5) * "," * float2hex(-2.0)) == [1.5, -2.0]
        @test length(signal2hex([40.0, 50.0])) == 16
        @test matrix2hex([1.0 2.0; 3.0 4.0]) == signal2hex([1.0, 2.0, 3.0, 4.0])
    end

    @testset "Modelos guardados" begin
        G, L = get_last_model(sys)
        @test L ≈ SAMPLING_TIME
        @test dcgain(G)[1] > 0
        @test size(ss(sys).A) == (1, 1)
        @test size(ss(sys; numpar=3).A) == (2, 2)
        @test dcgain(tf(sys; numpar=3))[1] ≈ dcgain(ss(sys; numpar=3))[1]
        @test temp_from_power(sys, power_from_temp(sys, 50.0)) ≈ 50.0
        @test_throws ErrorException temp_from_power(sys, 120)
        @test_throws ErrorException power_from_temp(sys, MAX_TEMPERATURE + 1)
    end

    @testset "Diseño de controladores" begin
        G = tf(sys)
        s = tf("s")
        ωn, ζ = 0.1, 0.8
        T = ωn^2 / (s^2 + 2ζ * ωn * s + ωn^2)

        C = cont2dof(G, T, 2, [-0.3])
        Tcl = minreal(feedback(G, -C[1, 2]) * C[1, 1])   # u = C₁ r + C₂ y
        @test dcgain(Tcl)[1] ≈ 1.0 atol = 1e-8
        @test_throws ErrorException cont2dof(G, T, 2, Float64[])

        polos = [-0.08 + 0.06im, -0.08 - 0.06im]
        C1 = cont1dof(G, polos)
        @test sort(real.(ThermoLab.ControlSystemsBase.poles(feedback(C1 * G, 1)))) ≈ [-0.08, -0.08]

        Tlq, Gur = lqmodel(G, 10)
        @test dcgain(Tlq)[1] ≈ 1.0
    end
end
