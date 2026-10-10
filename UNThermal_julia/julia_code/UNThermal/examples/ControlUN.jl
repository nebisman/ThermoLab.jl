using ControlSystems
using LinearAlgebra
using Polynomials
using Plots; gr(linewidth=2, grid=:true)
using Printf
Base.show(io::IO, f::Float64) = @printf(io, "%.4f", f)


"""
    dise2p(G, T, m, PolosObs)

Calcula un controlador de dos parámetros para el sistema térmico.

# Argumentos
- `G`        : Función de transferencia de la planta térmica
- `T`        : Función de transferencia de lazo cerrado deseada
- `m::Int`   : Orden del controlador
- `PolosObs` : Vector de polos del observador

# Retorna
- `C` : Función de transferencia 1×2  →  [L(s)/A(s)   -M(s)/A(s)]

La ley de control es:  u = (L/A)·r − (M/A)·y

# Ejemplo
```julia
s  = tf("s")
G  = transfer_function(sys)
T  = 1 / (30*s + 1)
C2 = dise2p(G, T, 1, [-0.1])
set_controller!(sys, C2)
```
"""

"""
    dise2p(G, T, m, PolosObs)

Calcula un controlador de dos parámetros.

# Argumentos
- `G`: Función de transferencia de la planta
- `T`: Función de transferencia de lazo cerrado deseada
- `m::Int`: Orden del controlador
- `PolosObs::Vector`: Polos del observador

# Retorna
- `C`: Función de transferencia 1×2  →  [L(s)/A(s)   -M(s)/A(s)]

La señal de control es:  u = (L/A)·r − (M/A)·y
"""
function dise2p(G, T, m, polosobs)

    # ─── Funciones auxiliares ───────────────────────────────────────────

    """Multiplicación de polinomios en coeficientes descendentes."""
    function polyconv(a::Vector, b::Vector)
        c = zeros(promote_type(eltype(a), eltype(b)), length(a) + length(b) - 1)
        for i in eachindex(a), j in eachindex(b)
            c[i + j - 1] += a[i] * b[j]
        end
        return c
    end

    """Extrae coeficientes descendentes (grado mayor primero) de un Polynomial."""
    function descvec(p)
        return Float64.(reverse(coeffs(p)))
    end
    
    npol = length(polosobs)
    if npol == 0
        Dhbar = [1]
    else
        Dhbar = descvec(fromroots(polosobs))
    end
    
    D = descvec(denpoly(G)[1, 1])
    N_raw = descvec(numpoly(G)[1, 1])
    n = length(D) - 1  
    
    # ─── H = minreal(T / N(s)) ──────────────────────────────────────────

    H = minreal(T * tf([1.0], N_raw))
    nh = descvec(numpoly(H)[1, 1])
    dh = descvec(denpoly(H)[1, 1])
    
    grad_DH = length(dh) - 1   

    gr_Dpbar = n + m - grad_DH 
    
    if npol != gr_Dpbar
        error("DH_bar debe tener $gr_Dpbar polos")
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
        error("El orden del controlador m=$m debe ser al menos n-1 = $(n-1)")
    end

    # ─── Controlador de dos parámetros ──────────────────────────────────
     C2P = [tf(L,A_vec) tf(-M_vec,A_vec)]
     return C2P
    
end




"""
    asigne_polos(P, pol)

Calcula un controlador 1-DOF de realimentación unitaria que asigna los
polos de lazo cerrado al vector `pol`.
Retorna `(K, T, Gur, S, ind_error)`.
"""
function asigne_polos(P, pol)
    P1     = tf(P)
    N, D   = _tfdata(P1)
    n      = length(D) - 1
    l      = length(D) - length(N)
    nMm    = length(pol)
    m      = nMm - n
    DT_p   = fromroots(pol)
    Na     = zeros(length(D))
    Na[l+1:end] = N
    DT     = reverse(DT_p.coeffs)

    Sm = zeros(n + m + 1, 2*(m+1))
    for k in 1:m+1
        Sm[k:k+n, k]     = D
        Sm[k:k+n, k+m+1] = Na
    end

    ind_error = 0
    local Y::Vector{Float64}, X::Vector{Float64}

    if m > n - 1
        Ind  = setdiff(1:2m+2, m+1)
        Smj  = Sm[:, Ind]
        if rank(Smj) >= n + m + 1
            XY = Smj \ DT
            Y  = [XY[1:m]; 0.0]
            X  = XY[m+1:end]
        else
            XY = Sm \ DT
            Y  = XY[1:m+1]
            X  = XY[m+2:end]
        end
    elseif m == n - 1
        XY = Sm \ DT
        Y  = XY[1:m+1]
        X  = XY[m+2:2m+2]
    else
        ind_error = 1
        @warn "asigne_polos: Número de polos insuficiente"
        return nothing, nothing, nothing, nothing, ind_error
    end

    K   = tf(X, Y)
    T   = feedback(K * P)
    Gur = minreal(T / P)
    S   = minreal(1 - T)
    return K, T, Gur, S, ind_error
end


"""
    lq1(P, q)

Diseño LQ de 1-DOF: sintonía óptima por ecuación de Wiener-Hopf.
`q` es el peso relativo salida/entrada. Retorna `(T, Gur)`.
"""
function lq1(P, q)
    n, d  = _tfdata(P)
    Qd    = _convol(d, _cambia_signo(d))
    Qn    = q * _convol(n, _cambia_signo(n))
    Qa    = zeros(length(Qd))
    l     = length(Qd) - length(Qn)
    Qa[l+1:end] = Qn
    Q     = Qd .+ Qa
    Q_poly = Polynomial(reverse(Q))
    r      = roots(Q_poly)
    estables = real.(r) .<= 0
    dT = fromroots(r[estables])
    dT0 = dT(0)
    DT  = reverse(dT.coeffs)
    T   = tf(n, DT) / dcgain(tf(n, DT))
    Gur = minreal(T / P)
    return T, Gur
end


# ─── Utilidades de diseño ─────────────────────────────────────────────────────

"""Grafica respuesta al escalón del sistema `sys` hasta `t_max`."""
function step_plt(sys, t_max)
    rey = step(sys, t_max)
    si  = stepinfo(rey)
    plot(si)
end

"""Evalúa el máximo absoluto de la respuesta al escalón de `sys`."""
function max_resp_step(sys)
    reu   = step(sys)
    v_max = maximum(abs.(reu.y))
    return v_max
end

"""Grafica respuestas al escalón de salida y de control."""
function yu_stp_plt(T, Gur, tmax)
    re_y = step(T, tmax)
    sy   = stepinfo(re_y)
    re_u = step(Gur, tmax)
    su   = stepinfo(re_u)
    plot(plot(sy), plot(su))
end


# ─── Funciones internas ───────────────────────────────────────────────────────

function _tfdata(G)
    G = tf(G)
    n = num(G)[1, 1]
    d = den(G)[1, 1]
    return n, d
end

function _convol(h, u)
    n = length(h); m = length(u); l = n + m - 1
    w = zeros(l)
    for i in 1:l
        for j in max(1, i+1-m):min(i, n)
            w[i] += h[j] * u[i-j+1]
        end
    end
    return w
end

function _cambia_signo(n)
    n1   = copy(n)
    long = length(n1) - 1
    n2   = ones(long + 1)
    for j in long:-2:1
        n2[j] = -1
    end
    return n2 .* n
end
