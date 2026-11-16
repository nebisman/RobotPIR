/* ======================================================================= */
/* lqr_mode.h - MODO PROGRAMADO Y REFERENCIAS                              */
/* ARCHIVO GENERADO AUTOMATICAMENTE DESDE JULIA (set_ss_controller)        */
/* LQR_MODE: modo programado. Al activarse el control el robot pasa 5 s    */
/* en el modo 0 y despues cambia a LQR_MODE (STARTUP_TIME_S).              */
/* En ejecucion se cambia con Set_Control_Mode() (app_control.c).          */
/* Las referencias son las mismas para todos los modos.                    */
/* ======================================================================= */
#ifndef __LQR_MODE_H
#define __LQR_MODE_H

#define LQR_MODE 0  // 0: estados, 1: integral de velocidad, 2: vel. de yaw con integral, 3: vel. lineal y de yaw con integral

// Referencias (variables ref_pos, ref_vel, ref_yaw, ref_wd de app_control.c)
#define REFPOS_LQR        0.0000000000  // ref_pos: posicion x (m), modos 0 y 2
#define REFVEL_LQR        0.0000000000  // ref_vel: velocidad lineal (m/s), modos 1 y 3
#define REFYAW_LQR        0.0000000000  // ref_yaw: angulo yaw (rad), modos 0 y 1
#define REFWD_LQR         0.0000000000  // ref_wd: velocidad angular yaw (rad/s), modos 2 y 3

#endif /* __LQR_MODE_H */
