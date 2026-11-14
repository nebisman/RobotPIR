/* ======================================================================= */
/* lqr_mode.h - MODO INICIAL Y REFERENCIAS                                 */
/* ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)        */
/* LQR_MODE: modo con el que arranca el robot (el ultimo programado);      */
/* en ejecucion se cambia con Set_Control_Mode() (app_control.c).          */
/* Las referencias son las mismas para todos los modos.                    */
/* ======================================================================= */
#ifndef __LQR_MODE_H
#define __LQR_MODE_H

#define LQR_MODE 2  // 0: estados, 1: integral de velocidad, 2: vel. de yaw con integral

// Referencias (variables ref_pos, ref_vel, ref_yaw, ref_wd de app_control.c)
#define REFPOS_LQR        0.0000000000  // ref_pos: posicion x (m), modos 0 y 2
#define REFVEL_LQR        0.0000000000  // ref_vel: velocidad lineal (m/s), modo 1
#define REFYAW_LQR        0.0000000000  // ref_yaw: angulo yaw (rad), modos 0 y 1
#define REFWD_LQR         0.0698131701  // ref_wd: velocidad angular yaw (rad/s), modo 2

#endif /* __LQR_MODE_H */
