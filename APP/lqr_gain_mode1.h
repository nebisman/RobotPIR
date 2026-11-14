/* ======================================================================= */
/* lqr_gain_mode1.h - MODO 1: accion integral de velocidad lineal          */
/* ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)        */
/* NO EDITAR MANUALMENTE - ejecutar control_lqr_mode1.jl                   */
/*                                                                         */
/* u = -(K x + Ki z),  x = [v, pitch, pitch_rate, yaw, yaw_rate]           */
/* z = integral(ref_vel - v)                                               */
/* Los indices Kij siguen el estado completo del modo 0 (j=2: v, ..., 6);  */
/* no hay K11/K21 porque el modo 1 no realimenta la posicion.              */
/* Sufijo _1: ganancias del modo 1 (conviven con las _0 del modo 0)        */
/* Las referencias estan en lqr_mode.h                                     */
/* ======================================================================= */
#ifndef __LQR_GAIN_MODE1_H
#define __LQR_GAIN_MODE1_H

// Rueda izquierda (fila 1 de K)
#define K12_1_LQR       -62.6312109323  // Velocidad lineal v
#define K13_1_LQR      -392.1259258914  // Angulo pitch
#define K14_1_LQR       -36.3734475415  // Vel. angular pitch
#define K15_1_LQR        15.8113883008  // Angulo yaw
#define K16_1_LQR         8.1741031283  // Vel. angular yaw
#define Ki1_1_LQR        15.8113883008  // Integral del error de velocidad

// Rueda derecha (fila 2 de K)
#define K22_1_LQR       -62.6312109323  // Velocidad lineal v
#define K23_1_LQR      -392.1259258914  // Angulo pitch
#define K24_1_LQR       -36.3734475415  // Vel. angular pitch
#define K25_1_LQR       -15.8113883008  // Angulo yaw
#define K26_1_LQR        -8.1741031283  // Vel. angular yaw
#define Ki2_1_LQR        15.8113883009  // Integral del error de velocidad

#endif /* __LQR_GAIN_MODE1_H */
