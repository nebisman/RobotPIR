// Ganancias LQR calculadas para los 7 estados del robot
/* =================================================================== */
/* ARCHIVO GENERADO AUTOMÁTICAMENTE DESDE JULIA                        */
/* NO EDITAR MANUALMENTE - Ejecutar diseno_controlador.jl              */
/* =================================================================== */
#ifndef __LQR_GAIN_H
#define __LQR_GAIN_H

// Ganancias LQR calculadas para los 6 estados del robot
#define K11_LQR    0.0000000000  // Posición X
#define K12_LQR    -46.5248954723  // Velocidad X
#define K13_LQR    -398.2820069736  // Ángulo Pitch
#define K14_LQR    -37.6734146663  // Vel. Angular Pitch
#define K15_LQR    51.3292955954  // Ángulo Yaw
#define K16_LQR    33.2221547265  // Vel. Angular Yaw
#define K21_LQR    0.0000000000  // Posición X
#define K22_LQR    -36.8320461397  // Velocidad X
#define K23_LQR    -350.9034384129  // Ángulo Pitch
#define K24_LQR    -31.0537170356  // Vel. Angular Pitch
#define K25_LQR    -44.9681346350  // Ángulo Yaw
#define K26_LQR    -29.3252813654  // Vel. Angular Yaw
#define Ki11_LQR     17.0905206744  // Constante integral v
#define Ki12_LQR     0.0000000000  // Constante integral w_delta
#define Ki21_LQR     17.0905206744  // Constante integral v
#define Ki22_LQR     0.0000000000  // Constante integral w_delta
#define REFPOS_LQR     0.0000000000  // referencia velocidad  
#define REFPSI_LQR     0.0000000000  // referencia wdelta 
#define REFVEL_LQR   0.0500000000  // referencia pos x 
#define REFW_LQR     0.0000000000  // referencia angulo Yaw 
#endif /* __LQR_GAIN_H */
