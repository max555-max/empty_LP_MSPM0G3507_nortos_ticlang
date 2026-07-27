#ifndef __BLUETOOTH_H_
#define __BLUETOOTH_H_

#include <stdint.h>

/*
 * Bluetooth PID tuning over UART1.
 *
 * Wiring:
 *   PB6/UART1_TX -> Bluetooth RXD
 *   PB7/UART1_RX <- Bluetooth TXD
 *   GND          -> Bluetooth GND
 *   Baud rate    -> 9600
 *
 * Usage:
 *   Call bluetooth_init() after SYSCFG_DL_init().
 *   Call bluetooth_process() repeatedly in the main loop.
 *
 * Command packets are ASCII and framed by '{' and '}'.
 *
 * Angle loop:
 *   {AKP=10000} angle Kp, /1000 scale, 10000 means 10.000
 *   {AKD=0}     angle Kd, /1000 scale
 *   {ABS=300}   angle base speed, mm/s
 *   {AMX=220}   angle max correction, mm/s
 *   {ANG=90}    set target yaw to current yaw + 90 deg
 *   {ANG=-90}   set target yaw to current yaw - 90 deg
 *
 * Line-track loop:
 *   {LKP=250}   line Kp, /1000 scale
 *   {LKD=120}   line Kd, /1000 scale
 *   {LBS=300}   line base speed, mm/s
 *   {LMX=280}   line max correction, mm/s
 *
 * Query:
 *   {GET}
 *
 * Indexed commands are also accepted:
 *   {0:value}=AKP, {1:value}=AKD, {2:value}=ABS, {3:value}=AMX
 *   {4:value}=LKP, {5:value}=LKD, {6:value}=LBS, {7:value}=LMX
 */
void bluetooth_init(void);
void bluetooth_process(void);
void bluetooth_send_params(void);

#endif
