// Ganancias LQR calculadas para los 6 estados del robot
/* =================================================================== */
/* ARCHIVO GENERADO AUTOMÁTICAMENTE DESDE JULIA                        */
/* NO EDITAR MANUALMENTE - Ejecutar diseno_controlador.jl              */
/* =================================================================== */
#ifndef __LQR_GAIN_H
#define __LQR_GAIN_H

#define MODE_LQR 0  // modo de control

// Ganancias LQR calculadas para los 6 estados del robot
#define K11_LQR    -158.1138830085  // Posición X
#define K12_LQR    -134.2385295940  // Velocidad lineal V
#define K13_LQR    -412.5396138241  // Ángulo Pitch
#define K14_LQR    -27.0036035178  // Vel. Angular Pitch
#define K15_LQR    31.6227766017  // Ángulo Yaw
#define K16_LQR    23.1004745385  // Vel. Angular Yaw
#define K21_LQR    -158.1138830085  // Posición X
#define K22_LQR    -134.2385295940  // Velocidad X
#define K23_LQR    -412.5396138241  // Ángulo Pitch
#define K24_LQR    -27.0036035178  // Vel. Angular Pitch
#define K25_LQR    -31.6227766017  // Ángulo Yaw
#define K26_LQR    -23.1004745385  // Vel. Angular Yaw
#define Ki11_LQR     0.0000000000  // Constante integral de pos x
#define Ki12_LQR     0.0000000000  // Constante integral de Yaw
#define Ki21_LQR     0.0000000000  // Constante integral de pos x
#define Ki22_LQR     0.0000000000  // Constante integral de Yaw
#define REFPOS_LQR   0.0000000000  // referencia pos x 
#define REFYAW_LQR   0.0000000000  // referencia angulo Yaw 
#define REFVEL_LQR   0.0000000000  // referencia velocidad lineal 
#define REFWD_LQR     0.0000000000  // referencia velocidad angular Yaw 
#endif /* __LQR_GAIN_H */
