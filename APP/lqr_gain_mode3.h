/* ======================================================================= */
/* lqr_gain_mode3.h - MODO 3: velocidad lineal y de yaw con integral       */
/* ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)        */
/* NO EDITAR MANUALMENTE - ejecutar control_lqr_mode3.jl                   */
/*                                                                         */
/* u = -(K (x - x_ref) + Kiv zv + Kiw zw)                                  */
/* x = [v, pitch, pitch_rate, yaw_rate]                                    */
/* zv = integral(ref_vel - v),  zw = integral(ref_wd - yaw_rate)           */
/* Los indices Kij siguen el estado completo del modo 0 (j=2: v, ..., 6);  */
/* no hay K11/K15: el modo 3 no realimenta posicion ni angulo de yaw.      */
/* Sufijo _3: ganancias del modo 3 (conviven con las de los otros modos)   */
/* Las referencias estan en lqr_mode.h                                     */
/* ======================================================================= */
#ifndef __LQR_GAIN_MODE3_H
#define __LQR_GAIN_MODE3_H

// Rueda izquierda (fila 1 de K)
#define K12_3_LQR       -73.8882840028  // Velocidad lineal v
#define K13_3_LQR      -401.1340881441  // Angulo pitch
#define K14_3_LQR       -36.9988380116  // Vel. angular pitch
#define K16_3_LQR        50.3351956776  // Vel. angular yaw
#define Kiv1_3_LQR       31.6227766017  // Integral del error de velocidad
#define Kiw1_3_LQR      -31.6227766017  // Integral del error de vel. yaw

// Rueda derecha (fila 2 de K)
#define K22_3_LQR       -73.8882840028  // Velocidad lineal v
#define K23_3_LQR      -401.1340881441  // Angulo pitch
#define K24_3_LQR       -36.9988380116  // Vel. angular pitch
#define K26_3_LQR       -50.3351956776  // Vel. angular yaw
#define Kiv2_3_LQR       31.6227766017  // Integral del error de velocidad
#define Kiw2_3_LQR       31.6227766017  // Integral del error de vel. yaw

#endif /* __LQR_GAIN_MODE3_H */
