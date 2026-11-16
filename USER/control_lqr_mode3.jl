# ==============================================================================
# GUÍA DE LABORATORIO - CONTROL LQR / ROBOT DE EQUILIBRIO
# Archivo: control_lqr_mode3.jl
# Descripción: Modelo físico, matrices A y B, diseño LQR y llamado al pipeline (modo 3: velocidad lineal y velocidad angular de yaw con acción integral)
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

# Modo 3: seguimiento de la velocidad lineal y de la velocidad angular de yaw,
# las dos con acción integral.
# Estado x = [vel_lineal, ángulo_pitch, vel_pitch, vel_yaw] más dos estados
# integrales: z_v = ∫(ref_vel - vel_lineal) dt y z_w = ∫(ref_wd - vel_yaw) dt.
# No se realimentan la posición ni el ángulo de yaw: el robot avanza a ref_vel
# mientras gira a ref_wd, así que recorre un círculo de radio ref_vel/ref_wd.

# Matriz de estado A3 (4x4)
A3 = [0.0  A_23  0.0  0.0;
      0.0   0.0  1.0  0.0;
      0.0  A_43  0.0  0.0;
      0.0   0.0  0.0  0.0]

# Matriz de entrada B3 (4x2)
B3 = (inercia/r) .* [B_21  B_22;
                     0.0   0.0;
                     B_41  B_42;
                     B_61  B_62]

C3 = [1.0  0.0  0.0  0.0;   # salida 1: velocidad lineal
      0.0  0.0  0.0  1.0]   # salida 2: velocidad angular de yaw
D3 = zeros(2, 2)

sys3 = ss(A3, B3, C3, D3)

# Sistema aumentado con los dos integradores (6x6): dz/dt = [ref_vel; ref_wd] - C3 x
Ag3 = [A3   zeros(4, 2);
       -C3  zeros(2, 2)]
Bg3 = [B3;
       zeros(2, 2)]

# =====================================
# 3. SINTONIZACIÓN DEL CONTROLADOR LQR
# =====================================

# Penalización de desviación de estados Q (6x6)
# Los pesos de avance son los del modo 1 y los de giro los del modo 2. Como el
# avance y el giro están desacoplados en el modelo, las ganancias de avance salen
# iguales a las del modo 1 y las de giro iguales a las del modo 2.
Q3 = diagm([
    5000.0,   # [2] Velocidad lineal X (m/s)
       0.0,   # [3] Ángulo Pitch / Inclinación frontal (rad)
    1600.0,   # [4] Velocidad angular Pitch (rad/s)
    5000.0,   # [6] Velocidad angular Yaw (rad/s)
    2000.0,   # [7] Integral del error de velocidad lineal
    2000.0,   # [8] Integral del error de velocidad angular de yaw
])

# Penalización del esfuerzo de control R (2x2) [Motor Izquierdo, Motor Derecho]
R3 = [
    1.0  0.0;
    0.0  1.0
]

# Cálculo de la matriz K de ganancias LQR (Matriz de 2x6)
K3 = lqr(Continuous, Ag3, Bg3, Q3, R3)


# ==============================================================================
# 🚀 4. DESPLIEGUE EN HARDWARE (Alt + Enter)
# ==============================================================================
include("lqr_pipeline.jl")

set_ss_controller(K3; mode=3, ref_vel=0.22, ref_wd=deg2rad(45))
