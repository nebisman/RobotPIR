/* ======================================================================= */
/* lqr_gain_mode0.h - MODO 0: realimentacion de estados                    */
/* ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)        */
/* NO EDITAR MANUALMENTE - ejecutar control_lqr_mode0.jl                   */
/*                                                                         */
/* u = -K (x - x_ref),  x = [x, v, pitch, pitch_rate, yaw, yaw_rate]       */
/* Sufijo _0: ganancias del modo 0 (conviven con las _1 del modo 1)        */
/* Las referencias estan en lqr_mode.h                                     */
/* ======================================================================= */
#ifndef __LQR_GAIN_MODE0_H
#define __LQR_GAIN_MODE0_H

// Rueda izquierda (fila 1 de K)
#define K11_0_LQR       -50.0000000000  // Posicion x
#define K12_0_LQR       -68.0982757390  // Velocidad lineal v
#define K13_0_LQR      -374.0820844238  // Angulo pitch
#define K14_0_LQR       -23.1449839151  // Vel. angular pitch
#define K15_0_LQR        31.6227766017  // Angulo yaw
#define K16_0_LQR        23.1004745385  // Vel. angular yaw

// Rueda derecha (fila 2 de K)
#define K21_0_LQR       -50.0000000000  // Posicion x
#define K22_0_LQR       -68.0982757390  // Velocidad lineal v
#define K23_0_LQR      -374.0820844238  // Angulo pitch
#define K24_0_LQR       -23.1449839151  // Vel. angular pitch
#define K25_0_LQR       -31.6227766017  // Angulo yaw
#define K26_0_LQR       -23.1004745385  // Vel. angular yaw

#endif /* __LQR_GAIN_MODE0_H */
