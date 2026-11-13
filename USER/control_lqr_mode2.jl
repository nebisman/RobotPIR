# ==============================================================================
# GUÍA DE LABORATORIO - CONTROL LQR / ROBOT DE EQUILIBRIO
# Archivo: diseno_controlador.jl
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
Aw = [0.0  1.0   0.0   0.0   0.0;
     0.0  0.0  A_23   0.0   0.0;
     0.0  0.0   0.0   1.0   0.0;
     0.0  0.0  A_43   0.0   0.0;     
     0.0  0.0   0.0   0.0   0.0]

# Matriz de entrada B (6x2)
Bw = (inercia/r) .* [0.0   0.0;
                    B_21  B_22;
                    0.0   0.0;
                    B_41  B_42;                    
                    B_61  B_62]

Cw = [0.0 0.0 0.0 0.0 1.0]
Dw = zeros(1, 2)

sysw = ss(Aw, Bw, Cw, Dw)

# =====================================
# 3. SINTONIZACIÓN DEL CONTROLADOR LQR 
# =====================================

# # Penalización de desviación de estados Q (6x6)


Qw = diagm([
     500.0,   # [1] Posición X (m)
     00.0,  # [2] Velocidad lineal X (m/s)
     100.0,   # [3] Ángulo Pitch / Inclinación frontal (rad)
     1000.0,  # [4] Velocidad angular Pitch (rad/s)
     200.0,    # [5] velocidad w_delta
     50.0      # [6] integral velocidad w_delta
 ])


# Penalización del esfuerzo de control R (2x2) [Motor Izquierdo, Motor Derecho]
Rw = [
    1.0  0.0;
    0.0  1.0
]

Agw = [Aw  zeros(5, 1); -Cw 0  ]
Bgw = [Bw; 0 0]


# Cálculo de la matriz K de ganancias LQR (Matriz de 2x6)
Kw = lqr(Continuous, Agw, Bgw, Qw, Rw)
  


include("lqr_pipeline.jl")

set_ss_controller(Kw; ref_wd = 10.0,ref_pos=0.0, mode=2)