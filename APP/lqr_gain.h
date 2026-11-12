// Ganancias LQR calculadas para los 7 estados del robot
/* =================================================================== */
/* ARCHIVO GENERADO AUTOMÁTICAMENTE DESDE JULIA                        */
/* NO EDITAR MANUALMENTE - Ejecutar diseno_controlador.jl              */
/* =================================================================== */
#ifndef __LQR_GAIN_H
#define __LQR_GAIN_H

// Ganancias LQR calculadas para los 6 estados del robot
#define K11_LQR    0.0000000000  // Posición X
#define K12_LQR    -49.3676338982  // Velocidad X
#define K13_LQR    -412.0434521588  // Ángulo Pitch
#define K14_LQR    -39.2239519992  // Vel. Angular Pitch
#define K15_LQR    15.0458723710  // Ángulo Yaw
#define K16_LQR    10.2481008380  // Vel. Angular Yaw
#define K21_LQR    0.0000000000  // Posición X
#define K22_LQR    -34.1659204691  // Velocidad X
#define K23_LQR    -338.0014172330  // Ángulo Pitch
#define K24_LQR    -29.6435703559  // Vel. Angular Pitch
#define K25_LQR    -15.0960909901  // Ángulo Yaw
#define K26_LQR    -10.4053314128  // Vel. Angular Yaw
#define Ki11_LQR     18.1432433746  // Constante integral v
#define Ki12_LQR     0.0000000000  // Constante integral w_delta
#define Ki21_LQR     12.5189225348  // Constante integral v
#define Ki22_LQR     0.0000000000  // Constante integral w_delta
#define REFPOS_LQR     0.0000000000  // referencia velocidad  
#define REFPSI_LQR     0.0000000000  // referencia wdelta 
#define REFVEL_LQR   0.0500000000  // referencia pos x 
#define REFW_LQR     0.0000000000  // referencia angulo Yaw 
#endif /* __LQR_GAIN_H */
