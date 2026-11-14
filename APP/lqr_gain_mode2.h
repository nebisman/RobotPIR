/* ======================================================================= */
/* lqr_gain_mode2.h - MODO 2: velocidad angular de yaw (sin integral)      */
/* ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)        */
/* NO EDITAR MANUALMENTE - ejecutar control_lqr_mode2.jl                   */
/*                                                                         */
/* u = -K (x - x_ref),  x = [x, v, pitch, pitch_rate, yaw_rate]            */
/* Los indices Kij siguen el estado completo del modo 0 (j=6: yaw_rate);   */
/* no hay K15/K25 porque el modo 2 no realimenta el angulo de yaw.         */
/* Sufijo _2: ganancias del modo 2 (conviven con las de los otros modos)   */
/* Las referencias estan en lqr_mode.h                                     */
/* ======================================================================= */
#ifndef __LQR_GAIN_MODE2_H
#define __LQR_GAIN_MODE2_H

// Rueda izquierda (fila 1 de K)
#define K11_2_LQR       -70.7106781187  // Posicion x
#define K12_2_LQR       -84.3865906007  // Velocidad lineal v
#define K13_2_LQR      -383.6103760608  // Angulo pitch
#define K14_2_LQR       -24.1049665138  // Vel. angular pitch
#define K16_2_LQR       158.1138830084  // Vel. angular yaw

// Rueda derecha (fila 2 de K)
#define K21_2_LQR       -70.7106781187  // Posicion x
#define K22_2_LQR       -84.3865906007  // Velocidad lineal v
#define K23_2_LQR      -383.6103760608  // Angulo pitch
#define K24_2_LQR       -24.1049665138  // Vel. angular pitch
#define K26_2_LQR      -158.1138830084  // Vel. angular yaw

#endif /* __LQR_GAIN_MODE2_H */
