# ==============================================================================
# GUÍA DE LABORATORIO - CONTROL LQR / ROBOT DE EQUILIBRIO
# Archivo: control_lqr_simulation.jl
# Descripción: Modelo físico, matrices A y B, diseño LQR y llamado al pipeline
# ==============================================================================

using LinearAlgebra
using ControlSystems

# Cargar el motor de automatización (detección de puerto, compilación C y flasheo)
include("lqr_pipeline.jl")

# ==============================================================================
# 1. PARÁMETROS FÍSICOS DEL ROBOT
# ==============================================================================
m           = 0.035                              # Masa de cada rueda [kg]
r           = 0.0672 / 2                         # Radio de la rueda [m]
inercia     = 0.5 * m * r^2                      # Momento de inercia de la rueda [kg·m²]

M           = 1.000 + 2*m                        # Masa del cuerpo del vehículo [kg]
L           = 0.0766 / 2                         # Distancia del centro de masa al eje [m]

J_centroide = (1/12) * M * (0.0766^2 + 0.0575^2) # Momento de inercia respecto al centro de masa [kg·m²]
d           = 0.1612                             # Ancho de vía (distancia entre ruedas) [m]
J_Y_delta   = (1/12) * M * (0.0766^2 + 0.0575^2) # Inercia de giro respecto al eje Y [kg·m²]
g           = 9.8                                # Aceleración gravitacional [m/s²]

# ==============================================================================
# 2. CONSTRUCCIÓN DE MATRICES DEL SISTEMA (ESPACIO DE ESTADOS)
#    Estado x = [posición, vel_lineal, ángulo_pitch, vel_pitch, ángulo_yaw, vel_yaw]
# ==============================================================================
Q_aux = J_centroide * M + (J_centroide + M*L^2) * (2*m + 2*inercia/r^2)

A_23 = -(M^2 * L^2 * g) / Q_aux
A_43 =  M * L * g * (M + 2*m + 2*inercia/r^2) / Q_aux

B_21 = (J_centroide + M*L^2 + M*L*r) / (Q_aux * r)
B_22 = B_21

B_41 = -(M*L/r + M + 2*m + 2*inercia/r^2) / Q_aux
B_42 = B_41

B_61 =  1 / (r * (m*d + inercia*d/r^2 + 2*J_Y_delta/d))
B_62 = -B_61

# Matriz de estado A (6x6)
A = [0.0  1.0   0.0   0.0  0.0  0.0;
     0.0  0.0  A_23   0.0  0.0  0.0;
     0.0  0.0   0.0   1.0  0.0  0.0;
     0.0  0.0  A_43   0.0  0.0  0.0;
     0.0  0.0   0.0   0.0  0.0  1.0;
     0.0  0.0   0.0   0.0  0.0  0.0]

# Matriz de entrada B (6x2)
B = (inercia/r) .* [0.0   0.0;
                    B_21  B_22;
                    0.0   0.0;
                    B_41  B_42;
                    0.0   0.0;
                    B_61  B_62]

C = [0.0 0.0 0.0 0.0 1.0 0.0]
D = zeros(1, 2)

sys = ss(A, B, C, D)

# =====================================
# 3. SINTONIZACIÓN DEL CONTROLADOR LQR 
# =====================================

# Penalización de desviación de estados Q (6x6)
Q_lqr = diagm([
    7700.0,   # [1] Posición X (m)
    0.0,  # [2] Velocidad lineal X (m/s)
    0.0,   # [3] Ángulo Pitch / Inclinación frontal (rad)
    1600.0,  # [4] Velocidad angular Pitch (rad/s)
    500.0,    # [5] Ángulo Yaw / Orientación (rad)
    0.0      # [6] Velocidad angular Yaw (rad/s)
])

# Penalización del esfuerzo de control R (2x2) [Motor Izquierdo, Motor Derecho]
R_lqr = [
    1.0  0.0;
    0.0  1.5
]

  

# Cálculo de la matriz K de ganancias LQR (Matriz de 2x6)
Klqr = lqr(sys, Q_lqr, R_lqr)



# ==============================================================================
# 4. SIMULACIÓN: RESPUESTA A UN ESCALÓN EN EL ÁNGULO DE YAW
#
#    Ley de control con referencia:   u = -K (x - x_ref)
#    con x_ref = [0, 0, 0, 0, ψ_ref, 0].
#
#    La columna 5 de A es nula (el yaw no afecta la dinámica), por lo que x_ref
#    es un punto de equilibrio con u = 0. La ley equivale entonces a
#        u = -K x + Kr ψ_ref,     Kr = K[:, 5]   (ganancia de referencia 2x1)
#    y el lazo cerrado queda:
#        ẋ = (A - B K) x + B Kr ψ_ref
#    En estado estacionario x = x_ref, es decir, error nulo en yaw.
# ==============================================================================
using Plots

ψ_ref = deg2rad(90.0)              # Amplitud del escalón de yaw [rad]
t_sim = 0:0.001:6.0                # Vector de tiempo [s]

Kr_yaw = Klqr[:, 5]                # Ganancia de referencia de yaw (2x1)

# Sistema en lazo cerrado: entrada ψ_ref, salidas = los 6 estados
A_cl   = A - B*Klqr
B_cl   = B*Kr_yaw
sys_cl = ss(A_cl, reshape(B_cl, 6, 1), Matrix(1.0I, 6, 6), zeros(6, 1))

println("Polos de lazo cerrado:")
display(eigvals(A_cl))

# Simulación (el robot parte del reposo en x = 0)
r_sim = fill(ψ_ref, 1, length(t_sim))
res   = lsim(sys_cl, r_sim, t_sim)
X     = res.y                              # Estados (6 x N)
U     = -Klqr*X .+ Kr_yaw*r_sim            # Esfuerzo de control [u_izq; u_der] (2 x N)

# Métricas de la respuesta en yaw
ψ_sim  = X[5, :]
Mp     = 100 * (maximum(ψ_sim) - ψ_ref) / ψ_ref                     # Sobreimpulso [%]
i_10   = findfirst(>=(0.1*ψ_ref), ψ_sim)
i_90   = findfirst(>=(0.9*ψ_ref), ψ_sim)
tr     = isnothing(i_10) || isnothing(i_90) ? NaN : t_sim[i_90] - t_sim[i_10]  # Tiempo de subida 10-90 % [s]
i_fuera = findlast(abs.(ψ_sim .- ψ_ref) .> 0.02*ψ_ref)
ts     = isnothing(i_fuera) ? 0.0 :
         i_fuera == length(t_sim) ? NaN : t_sim[i_fuera + 1]         # Tiempo de establecimiento 2 % [s]

@printf("\nRespuesta a escalón de yaw de %.1f°:\n", rad2deg(ψ_ref))
@printf("  Sobreimpulso            Mp = %.2f %%\n", max(Mp, 0.0))
@printf("  Tiempo de subida (10-90) tr = %.3f s\n", tr)
@printf("  Tiempo de establec. (2%%) ts = %.3f s\n", ts)
@printf("  Yaw final              ψ(t_f) = %.2f°\n", rad2deg(ψ_sim[end]))
@printf("  Esfuerzo máximo        |u|max = %.3f\n", maximum(abs.(U)))

# Gráficas
p1 = plot(t_sim, rad2deg.(ψ_sim), lw=2, label="ψ (yaw)",
          title="Ángulo de yaw", ylabel="[°]")
hline!(p1, [rad2deg(ψ_ref)], ls=:dash, color=:black, label="ψ_ref")

p2 = plot(t_sim, rad2deg.(X[6, :]), lw=2, label="dψ/dt",
          title="Velocidad angular de yaw", ylabel="[°/s]")

p3 = plot(t_sim, rad2deg.(X[3, :]), lw=2, label="θ (pitch)",
          title="Ángulo de pitch", ylabel="[°]")

p4 = plot(t_sim, X[1, :], lw=2, label="x",
          title="Posición lineal", ylabel="[m]")

p5 = plot(t_sim, U[1, :], lw=2, label="u izq.",
          title="Esfuerzo de control", xlabel="t [s]")
plot!(p5, t_sim, U[2, :], lw=2, ls=:dash, label="u der.")

p6 = plot(t_sim, X[2, :], lw=2, label="dx/dt",
          title="Velocidad lineal", xlabel="t [s]", ylabel="[m/s]")

p_yaw = plot(p1, p2, p3, p4, p5, p6, layout=(3, 2), size=(1000, 800),
             plot_title="Respuesta LQR a un escalón de yaw de $(round(rad2deg(ψ_ref), digits=1))°")
display(p_yaw)



# ==============================================================================
# 🚀 5. DESPLIEGUE EN HARDWARE (Alt + Enter)
