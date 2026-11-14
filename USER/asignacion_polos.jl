# # =====================================================
# # GUÍA DE LABORATORIO - CONTROL POR ASIGNACIÓN DE POLOS 
# # =====================================================

# using LinearAlgebra
# using ControlSystems
# using Plots

# # Cargar el motor de automatización (detección de puerto, compilación C y flasheo)

# # ===============================
# # 1. PARÁMETROS FÍSICOS DEL ROBOT
# # ===============================
# m           = 0.035                               # Masa de cada rueda [kg]
# r           = 0.0672 / 2                          # Radio de la rueda [m]
# inercia     = 0.5 * m * r^2                       # Momento de inercia de la rueda [kg·m²]

# M           = 1.000 + 2*m                         # Masa del cuerpo del vehículo [kg]
# L           = 0.0766 / 2                          # Distancia del centro de masa al eje [m]

# J_centroide = (1/12) * M * (0.0766^2 + 0.0575^2) # Momento de inercia respecto al centro de masa [kg·m²]
# d           = 0.1612                              # Ancho de vía (distancia entre ruedas) [m]
# J_Y_delta   = (1/12) * M * (0.0766^2 + 0.0575^2) # Inercia de giro respecto al eje Y [kg·m²]
# g           = 9.8                                 # Aceleración gravitacional [m/s²]

# # ==================================================================================
# # 2. CONSTRUCCIÓN DE MATRICES DEL SISTEMA (ESPACIO DE ESTADOS)
# #    Estado x = [posición, vel_lineal, ángulo_pitch, vel_pitch, ángulo_yaw, vel_yaw]
# # ==================================================================================
# Q_aux = J_centroide * M + (J_centroide + M*L^2) * (2*m + 2*inercia/r^2)

# A_23 = -(M^2 * L * g) / Q_aux
# A_43 =  M * L * g * (M + 2*m + 2*inercia/r^2) / Q_aux

# B_21 = (J_centroide + M*L^2 + M*L*r) / (Q_aux * r)
# B_22 = B_21

# B_41 = -(M*L/r + M + 2*m + 2*inercia/r^2) / Q_aux
# B_42 = B_41

# B_61 =  1 / (r * (m*d + inercia*d/r^2 + 2*J_Y_delta/d))
# B_62 = -B_61

# # Matriz de estado A (6x6)
# A = [0.0  1.0   0.0   0.0  0.0  0.0;
#      0.0  0.0  A_23   0.0  0.0  0.0;
#      0.0  0.0   0.0   1.0  0.0  0.0;
#      0.0  0.0  A_43   0.0  0.0  0.0;
#      0.0  0.0   0.0   0.0  0.0  1.0;
#      0.0  0.0   0.0   0.0  0.0  0.0]

# # Matriz de entrada B (6x2)
# B = (inercia/r) .* [0.0   0.0;
#                     B_21  B_22;
#                     0.0   0.0;
#                     B_41  B_42;
#                     0.0   0.0;
#                     B_61  B_62]

# C = Matrix{Float64}(I, 6, 6)
# D = zeros(6, 2)

# sys = ss(A, B, C, D)

# # =====================================
# # SINTONIZACIÓN POR ASIGNACIÓN DE POLOS
# # =====================================

# # 1. Modo de Posición (X, dX)
# lambda_1   = -12+15.6im
# lambda_2   = -12-15.6im

# # 2. Modo de Inclinación (Pitch θ, dθ)
# lambda_3 = -0.55
# lambda_4 = -22

# # 3. Modo de Orientación (Yaw ψ, dψ)
# lambda_5   = -5
# lambda_6   = -6



# # --- Asignación por Submodos Simétricos ---

# polos_lc = [lambda_1, lambda_2, lambda_3, lambda_4, lambda_5, lambda_6]

# K = place(A, B, polos_lc)



# println("Ganancias calculadas por asignación de polos:")
# println("K1 (Pos X)     = ", round(K[1,1], digits=4))
# println("K2 (Vel X)     = ", round(K[1,2], digits=4))
# println("K3 (Pitch)     = ", round(K[1,3], digits=4))
# println("K4 (Vel Pitch) = ", round(K[1,4], digits=4))
# println("K5 (Yaw)       = ", round(K[1,5], digits=4))
# println("K6 (Vel Yaw)   = ", round(K[1,6], digits=4))

# # ==============================================================================
# # 🚀 4. DESPLIEGUE EN HARDWARE (Alt + Enter)
# # ==============================================================================
# include("lqr_pipeline.jl")

# set_ss_controller(K)

# =====================================================
# GUÍA DE LABORATORIO - CONTROL POR ASIGNACIÓN DE POLOS 
# =====================================================

using LinearAlgebra
using ControlSystems
using Plots

# Cargar el motor de automatización (detección de puerto, compilación C y flasheo)
include("lqr_pipeline.jl")

# ===============================
# 1. PARÁMETROS FÍSICOS DEL ROBOT
# ===============================
m           = 0.035                               # Masa de cada rueda [kg]
r           = 0.0672 / 2                          # Radio de la rueda [m]
inercia     = 0.5 * m * r^2                       # Momento de inercia de la rueda [kg·m²]

M           = 1.000 + 2*m                         # Masa del cuerpo del vehículo [kg]
L           = 0.0766 / 2                          # Distancia del centro de masa al eje [m]

J_centroide = (1/12) * M * (0.0766^2 + 0.0575^2) # Momento de inercia respecto al centro de masa [kg·m²]
d           = 0.1612                              # Ancho de vía (distancia entre ruedas) [m]
J_Y_delta   = (1/12) * M * (0.0766^2 + 0.0575^2) # Inercia de giro respecto al eje Y [kg·m²]
g           = 9.8                                 # Aceleración gravitacional [m/s²]

# ==================================================================================
# 2. CONSTRUCCIÓN DE MATRICES DEL SISTEMA (ESPACIO DE ESTADOS)
#    Estado x = [posición, vel_lineal, ángulo_pitch, vel_pitch, ángulo_yaw, vel_yaw]
# ==================================================================================
Q_aux = J_centroide * M + (J_centroide + M*L^2) * (2*m + 2*inercia/r^2)

A_23 = -(M^2 * L * g) / Q_aux
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

C = Matrix{Float64}(I, 6, 6)
D = zeros(6, 2)

sys = ss(A, B, C, D)

# =====================================
# SINTONIZACIÓN POR ASIGNACIÓN DE POLOS
# =====================================

# 1. Modo de Posición (X, dX)
lambda_1   = -50
lambda_2   = -10

# 2. Modo de Inclinación (Pitch θ, dθ)
lambda_3 = -3.5 + 3.5im
lambda_4 = -3.5 - 3.5im

# 3. Modo de Orientación (Yaw ψ, dψ)
lambda_5   = -0.5+0.5im
lambda_6   = -0.5-0.5im

polos_lc = [lambda_1 lambda_2 lambda_3 lambda_4 lambda_5 lambda_6]

K = place2(A,B, polos_lc)

# # --- Asignación por Submodos Simétricos ---
# A_long = A[1:4, 1:4]
# B_long = B[1:4, 1:1] + B[1:4, 2:2] # Ambas ruedas avanzan juntas
# polos_long = [lambda_1, lambda_2, lambda_3, lambda_4]

# K_long = place(A_long, B_long, polos_long)

# A_yaw = A[5:6, 5:6]
# B_yaw = B[5:6, 1:1] - B[5:6, 2:2] # Giro diferencial
# polos_yaw = [lambda_5, lambda_6]

# K_yaw = place(A_yaw, B_yaw, polos_yaw)

# # --- Reconstrucción de la Matriz K (2x6) ---
# K_matrix = [
#     K_long K_yaw ;
#     K_long -K_yaw
# ]



println("Ganancias calculadas por asignación de polos:")
println("K1 (Pos X)     = ", round(K[1,1], digits=4))
println("K2 (Vel X)     = ", round(K[1,2], digits=4))
println("K3 (Pitch)     = ", round(K[1,3], digits=4))
println("K4 (Vel Pitch) = ", round(K[1,4], digits=4))
println("K5 (Yaw)       = ", round(K[1,5], digits=4))
println("K6 (Vel Yaw)   = ", round(K[1,6], digits=4))

# ==============================================================================
# 🚀 4. DESPLIEGUE EN HARDWARE (Alt + Enter)
# ==============================================================================


set_ss_controller(K; mode=0, ref_pos=0.0, ref_yaw=0.0)  