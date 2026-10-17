# ═══════════════════════════════════════════════════════════════════════════════
#  control_design.jl – Diseño algebraico de controladores por asignación de
#  polos.   LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using ControlSystemsBase
using LinearAlgebra
using Polynomials

"""
    cont2dof(G, T, m, polesobs=[])

Calcula, mediante un método lineal algebraico,  un controlador de dos grados de libertad (2-GDL)
que hace que la respuesta de lazo cerrado de la planta `G` coincida con la
función de transferencia deseada `T`.

# Argumentos
- `G`: Función de transferencia de la planta.
- `T`: Función de transferencia de lazo cerrado deseada.
- `m::Int`: Orden del controlador. Debe ser al menos `n-1`, donde `n` es el
  orden de la planta `G`.
- `polesobs::Vector=[]`: polos adicionales del observador implícito en el
  prefiltro.

# Retorna
- `C`: matriz  de transferencia del controlador, definida como: 

 ``C(s) = \\begin{bmatrix} \\dfrac{L(s)}{A(s)} & -\\dfrac{M(s)}{A(s)} \\end{bmatrix}``. 
 
 La señal  de control es ``u = \\frac{L}{A} r − \\frac{M}{A}y``.

# Lanza
- `ErrorException` si `m < n-1` (el orden del controlador es menor que el
  mínimo requerido, `n-1`, con `n` el orden de `G`).
- `ErrorException` si `polesobs` no tiene exactamente la longitud
  requerida por el grado del polinomio ``T(s)/N(s)``; el mensaje indica cuántos
  polos hacen falta.

# Notas

- Si selecciona un orden del controlador tal que `m > n-1`, el algoritmo coloca automaticamente acción integral
en el controlador diseñado.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber obtenido un modelo nominal de la planta usando las
funciones de identificación  [`get_model_step`](@ref) o [`get_model_prbs`](@ref).
Así podrá invocar la función de transferencia del sistema con la función `tf`:

```julia
using ThermoLab
sys = ThermalSystem();
G = tf(sys)
```

Luego, defina la respuesta de lazo cerrado deseada (por ejemplo, una
función de segundo orden) y diseñe el controlador de 2-GDL:

```julia
s = tf("s");
ωn = 0.1; ζ = 0.8
T = ωn^2 / (s^2 + 2ζ*ωn*s + ωn^2)
C = cont2dof(G, T, 2, [-0.3])

# ahora probamos la respuesta del controlador
set_controller(sys, C)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, T)
```
"""
function cont2dof(G, T, m, polesobs=[])
    
    # -- normalizar las funciones de transferencia

    G = tf(G)
    T = tf(T)
    # ─── Funciones auxiliares ───────────────────────────────────────────

    """Multiplicación de polinomios en coeficientes descendentes."""
    function polyconv(a::Vector, b::Vector)
        c = zeros(promote_type(eltype(a), eltype(b)), length(a) + length(b) - 1)
        for i in eachindex(a), j in eachindex(b)
            c[i + j - 1] += a[i] * b[j]
        end
        return c
    end

    """Extrae coeficientes descendentes (grado mayor primero) de un Polinomio."""
    function descvec(p)
        return Float64.(reverse(coeffs(p)))
    end
    
    npol = length(polesobs)
    if npol == 0
        Dhbar = [1]
    else
        Dhbar = descvec(fromroots(polesobs))
    end
    
    D = descvec(denpoly(G)[1, 1])
    N_raw = descvec(numpoly(G)[1, 1])
    n = length(D) - 1  
    
  

    if m < n-1
        error("El orden m del controlador debe ser al menos n-1 = $(n-1)")
    end
    # ─── H = minreal(T / N(s)) ──────────────────────────────────────────

    H = minreal(T * tf([1.0], N_raw))
    nh = descvec(numpoly(H)[1, 1])
    dh = descvec(denpoly(H)[1, 1])
    
    grad_DH = length(dh) - 1   

    gr_Dpbar = n + m - grad_DH 
    
    if npol != gr_Dpbar
        error("Debe asignar $gr_Dpbar polos para el observador")
    end


    
 

    # Rellenar N con ceros a la izquierda (como hace tfdata de MATLAB)
    N = [zeros(length(D) - length(N_raw)); N_raw]



    # ─── Prefiltro L y lado derecho F ───────────────────────────────────

    L = polyconv(nh, Dhbar)
    F = polyconv(dh, Dhbar)

    # ─── Matriz de Sylvester  (n+m+1) × (2m+2) ─────────────────────────

    Sm = zeros(n + m + 1, 2m + 2)
    for k in 1:m+1
        Sm[k:k+n, k]       = D
        Sm[k:k+n, k+m+1]   = N
    end

    # ─── Resolución del sistema lineal ──────────────────────────────────

    local A_vec::Vector{Float64}, M_vec::Vector{Float64}

    if m >= n
        # Intenta diseñar controlador que rechaza perturbaciones
        Ind = setdiff(1:2m+2, m + 1)           # todas las columnas excepto m+1
        Sm_red = Sm[:, Ind]
   
        if rank(Sm_red) >= n + m + 1
            x     = Sm_red \ F
            A_vec = [x[1:m]; 0.0]               # acción integral (cero en s=0)
            M_vec = x[m+1:2m+1]
        else
            x     = Sm \ F
            A_vec = x[1:m+1]
            M_vec = x[m+2:2m+2]
        end

    elseif m == n - 1
        x     = Sm \ F
        A_vec = x[1:m+1]
        M_vec = x[m+2:2m+2]

    else
        error("El orden m del controlador debe ser al menos n-1 = $(n-1)")
    end

    # ─── Controlador de dos parámetros ──────────────────────────────────
     C2P = [tf(L,A_vec) tf(-M_vec,A_vec)]
     return C2P
    
end




"""
    cont1dof(G, poles)

Calcula,  mediante el método lineal algebraico,  un controlador 1-GDL de realimentación
unitaria que ubica los polos de lazo cerrado, según los polos indicados por el vector `poles`.

# Argumentos
- `G`: Función de transferencia de la planta.
- `poles::Vector`: Polos deseados de lazo cerrado del sistema.

# Retorna
- `C`: función de transferencia del controlador.

# Lanza
- `ErrorException` si `poles` no tiene al menos `2n-1` elementos (con `n` el
  orden de `G`), es decir, si el orden  del controlador resulta menor que `n - 1`.

# Notas
- Si selecciona un orden del controlador tal que `m > n-1`, el algoritmo coloca automaticamente acción integral
en el controlador diseñado.



# Ejemplo

Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber obtenido un modelo nominal de la planta usando las
funciones de identificación  [`get_model_step`](@ref) o [`get_model_prbs`](@ref).
Así podrá invocar la función de transferencia del sistema con la función `tf`:

```julia
using ThermoLab
sys = ThermalSystem();
G = tf(sys)
```

Luego, defina los polos deseados de lazo cerrado y diseñe el controlador
(con dos polos para la planta de primer orden se obtiene un PI):

```julia
polos_lc = [-0.08+0.06im, -0.08-0.06im]
C = cont1dof(G, polos_lc)
set_controller(sys, C)


# Aqui vemos la respuesta al escalon en lazo cerrado
T = feedback(C*G, 1)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, T)
```
"""
function cont1dof(G, poles)

    # -- normalizar la función de transferencia

    G = tf(G)

    # ─── Función auxiliar ────────────────────────────────────────────────

    """Extrae coeficientes descendentes (grado mayor primero) de un Polinomio."""
    function descvec(p)
        return Float64.(reverse(coeffs(p)))
    end

    D = descvec(denpoly(G)[1, 1])
    N_raw = descvec(numpoly(G)[1, 1])
    n = length(D) - 1

    # Rellenar N con ceros a la izquierda (como hace tfdata de MATLAB)
    N = [zeros(length(D) - length(N_raw)); N_raw]

    # ─── Orden del controlador y polinomio deseado ─────────────────────

    m = length(poles) - n

    if m < n-1
        error("El número de polos mínimo debe ser  = $(2*n-1)")
    end

    DT = descvec(fromroots(poles))

    # ─── Matriz de Sylvester  (n+m+1) × (2m+2) ─────────────────────────

    Sm = zeros(n + m + 1, 2m + 2)
    for k in 1:m+1
        Sm[k:k+n, k]       = D
        Sm[k:k+n, k+m+1]   = N
    end

    # ─── Resolución del sistema lineal ──────────────────────────────────
     

    local X::Vector{Float64}, Y::Vector{Float64}

    if m >= n
        # Intenta diseñar un controlador que rechaza perturbaciones
        Ind = setdiff(1:2m+2, m + 1)           # todas las columnas excepto m+1
        Sm_red = Sm[:, Ind]

        if rank(Sm_red) >= n + m + 1
            x = Sm_red \ DT
            Y = [x[1:m]; 0.0]                  # acción integral (cero en s=0)
            X = x[m+1:end]
        else
            x = Sm \ DT
            Y = x[1:m+1]
            X = x[m+2:end]
        end

    elseif m == n - 1
        x = Sm \ DT
        Y = x[1:m+1]
        X = x[m+2:2m+2]

    else
        error("El número de polos mínimo debe ser  = $(2*n-1)")
    end

    # ─── Controlador ────────────────────

    C   = tf(X, Y)
    return C
end




"""
    lqmodel(G, q)

Calcula, por factorización espectral, la respuesta de lazo cerrado óptima
LQ  para la planta `G`, con peso `q` sobre la señal de control.

# Argumentos
- `G`: Función de transferencia de la planta.
- `q::Real`: Peso de la señal de control en el índice cuadrático.

# Retorna
- `T`: Función de transferencia de lazo cerrado, esto es, ``T(s)=\\frac{Y(s)}{R(s)}``.
- `Gur`: Función de transferencia de referencia a señal de control (`u = Gur·r`).

# Notas
- Un valor de `q` mayor penaliza más la señal de control, produciendo una
  respuesta de lazo cerrado `T` más lenta pero con menor esfuerzo de
  control; un valor de `q` menor produce una respuesta más rápida a costa
  de un mayor esfuerzo de control.
- `lqmodel` no diseña un controlador por sí sola: retorna un modelo de lazo
  lazo cerrado óptimo `T`, la cual debe pasarse luego a [`cont2dof`](@ref)
  para obtener el controlador que la realiza.

# Ejemplo
Primero, asegúrese de haber importado el paquete ThermoLab, de haber
definido el sistema y de haber obtenido un modelo nominal de la planta,
así:

```julia
using ThermoLab
sys = ThermalSystem();
G = tf(sys)
```

Luego, calcule la respuesta de lazo cerrado óptima LQ y diseñe el
controlador de 2-GDL que la realiza:

```julia
Tlq, Gur = lqmodel(G, 10)
C = cont2dof(G, Tlq, 2, [-0.3, -0.3])

# ahora probamos la respuesta del controlador
set_controller(sys, C)
result = step_closed(sys; r0 = 40, r1 = 50,  t0 = 60, t1 = 120);
stepinfo(result, Tlq)
```
"""
function lqmodel(G, q)

    # -- normalizar la función de transferencia

    G = tf(G)

    # ─── Funciones auxiliares ────────────────────────────────────────────

    """Multiplicación de polinomios en coeficientes descendentes."""
    function polyconv(a::Vector, b::Vector)
        c = zeros(promote_type(eltype(a), eltype(b)), length(a) + length(b) - 1)
        for i in eachindex(a), j in eachindex(b)
            c[i + j - 1] += a[i] * b[j]
        end
        return c
    end

    """Extrae coeficientes descendentes (grado mayor primero) de un Polinomio."""
    function descvec(p)
        return Float64.(reverse(coeffs(p)))
    end

    """Coeficientes descendentes de p(-s) a partir de los de p(s)."""
    function cambia_signo(p)
        signo = ones(length(p))
        signo[end-1:-2:1] .= -1
        return signo .* p
    end

    n = descvec(numpoly(G)[1, 1])
    d = descvec(denpoly(G)[1, 1])

    # ─── Factorización espectral de Q(s) = D(s)D(-s) + q·N(s)N(-s) ──────

    Qd = polyconv(d, cambia_signo(d))
    Qn = q * polyconv(n, cambia_signo(n))
    Qa = [zeros(length(Qd) - length(Qn)); Qn]
    Q  = Qd .+ Qa

    r = roots(Polynomial(reverse(Q)))
    estables = r[real.(r) .<= 0]
    DT = descvec(fromroots(estables))

    # ─── Controlador y funciones de lazo cerrado ─────────────────────────

    T   = tf(n, DT)
    T   = T / dcgain(T)
    Gur = minreal(T / G)

    return T, Gur
end



# Coeficientes normalizados del denominador (ω₀ = 1), de s^n a s^0.
const ITAE_POSICION = Dict(
    1 => [1, 1],
    2 => [1, 1.4, 1],
    3 => [1, 1.75, 2.15, 1],
    4 => [1, 2.1, 3.4, 2.7, 1],
    5 => [1, 2.8, 5.0, 5.5, 3.4, 1],
    6 => [1, 3.25, 6.6, 8.6, 7.45, 3.95, 1],
)

const ITAE_VELOCIDAD = Dict(
    2 => [1, 3.2, 1],
    3 => [1, 1.75, 3.25, 1],
    4 => [1, 2.41, 4.93, 5.14, 1],
    5 => [1, 2.19, 6.5, 6.3, 5.24, 1],
    6 => [1, 6.12, 13.42, 17.16, 14.14, 6.76, 1],
)


"""
    itaemodel(order=3, omega=1.0; type=:pos)

Calcula la respuesta de lazo cerrado de orden `order` que minimiza el índice
``ITAE = \\int_0^\\infty t\\,|e(t)|\\,dt``, con frecuencia ``\\omega_0`` = `omega`.

# Argumentos
- `order::Int=3`: Orden ``n`` de la función de transferencia. Con `type=:pos`
  admite de 1 a 6; con `type=:speed`, de 2 a 6.
- `omega::Real=1.0`: Frecuencia ``\\omega_0`` (rad/s) de la respuesta.

# Argumentos de palabra clave
- `type::Symbol=:pos`: Tipo de respuesta ITAE.
  + `:pos`: cero error de posición (sigue escalones sin error):
    ``T(s) = \\dfrac{\\omega_0^n}{D(s)}``
  + `:speed`: cero error de velocidad (sigue rampas sin error):
    ``T(s) = \\dfrac{a_1\\,\\omega_0^{n-1} s + \\omega_0^n}{D(s)}``

  donde ``D(s) = s^n + a_{n-1}\\,\\omega_0\\, s^{n-1} + \\cdots + a_1\\,\\omega_0^{n-1}\\, s + \\omega_0^n``.

# Retorna
- `T`: Función de transferencia de lazo cerrado candidata, esto es, ``T(s)=\\frac{Y(s)}{R(s)}``.

# Notas
- El parámetro `omega` cambia la velocidad de la respuesta pero no su forma: un
  valor mayor produce una respuesta más rápida a costa de un mayor esfuerzo de
  control.
- `itaemodel` no diseña un controlador por sí sola: retorna un modelo de lazo
  cerrado `T`, el cual debe pasarse luego a [`cont2dof`](@ref) para obtener el
  controlador que lo realiza.

# Ejemplo
Primero, asegúrese de haber importado el paquete DCMotor, de haber
definido el sistema y de haber obtenido un modelo nominal de la planta,
así:

```julia
using DCMotor
sys = MotorSystem();
G_ang = tf(sys, output=:angle, numpar=3)
```

Luego, calcule la respuesta de lazo cerrado ITAE y diseñe el controlador
de 2-GDL que la realiza:

```julia
w = 20
T = itaemodel(3, w; type=:pos)
C = cont2dof(G_ang, T, 2, [-4*w, -4*w])

# ahora probamos la respuesta del controlador
set_controller(sys, C; output=:angle)
result = step_closed(sys; r0 = 0, r1 = 90,  t0 = 0.5, t1 = 2);
stepinfo(result, T)
```
"""
function itaemodel(order::Int = 3, omega::Real = 1.0; type=:pos)
    tabla = type == :pos ? ITAE_POSICION :
            type == :speed ? ITAE_VELOCIDAD :
            error("type debe ser :pos (posición) o :speed (velocidad)")
    haskey(tabla, order) ||
        error("order debe estar entre $(minimum(keys(tabla))) y $(maximum(keys(tabla))) para type=$(repr(type))")
    omega > 0 || error("omega debe ser positiva")

    n = order
    # el coeficiente de s^(n-k) se multiplica por ω₀^k
    den = tabla[n] .* omega .^ (0:n)
    num = type == :pos ? [omega^n] :
                        den[end-1:end]   # mismos dos últimos coeficientes que D(s) -> e₁ = 0
    return tf(num, den)
end