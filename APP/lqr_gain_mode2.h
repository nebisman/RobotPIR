/* ======================================================================= */
/* lqr_gain_mode2.h - MODO 2: velocidad de yaw con accion integral         */
/* ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)        */
/* NO EDITAR MANUALMENTE - ejecutar control_lqr_mode2.jl                   */
/*                                                                         */
/* u = -(K (x - x_ref) + Ki z),  x = [x, v, pitch, pitch_rate, yaw_rate]   */
/* z = integral(ref_wd - yaw_rate)                                         */
/* Los indices Kij siguen el estado completo del modo 0 (j=6: yaw_rate);   */
/* no hay K15/K25 porque el modo 2 no realimenta el angulo de yaw.         */
/* Sufijo _2: ganancias del modo 2 (conviven con las de los otros modos)   */
/* Las referencias estan en lqr_mode.h                                     */
/* ======================================================================= */
#ifndef __LQR_GAIN_MODE2_H
#define __LQR_GAIN_MODE2_H

// Rueda izquierda (fila 1 de K)
#define K11_2_LQR       -50.0000000000  // Posicion x
#define K12_2_LQR       -72.0405936173  // Velocidad lineal v
#define K13_2_LQR      -376.7011753263  // Angulo pitch
#define K14_2_LQR       -23.4091597069  // Vel. angular pitch
#define K16_2_LQR        50.3351956776  // Vel. angular yaw
#define Ki1_2_LQR       -31.6227766017  // Integral del error de vel. yaw

// Rueda derecha (fila 2 de K)
#define K21_2_LQR       -50.0000000000  // Posicion x
#define K22_2_LQR       -72.0405936173  // Velocidad lineal v
#define K23_2_LQR      -376.7011753264  // Angulo pitch
#define K24_2_LQR       -23.4091597069  // Vel. angular pitch
#define K26_2_LQR       -50.3351956776  // Vel. angular yaw
#define Ki2_2_LQR        31.6227766017  // Integral del error de vel. yaw

#endif /* __LQR_GAIN_MODE2_H */
