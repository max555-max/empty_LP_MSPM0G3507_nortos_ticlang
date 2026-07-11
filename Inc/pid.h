#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>

typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t integral;
    int32_t previousError;
    int32_t integralLimit;
    int32_t outputLimit;
} pid_t;

#define PID_GAIN_SCALE      (1000)

#define SPEED_PID_CONTROL_PERIOD_MS     (10)

/*
 * Default speed target in mm/s. Keep 0 for safe power-on.
 * Use speed_pid_set_target() or change these macros when testing.
 */
#define SPEED_PID_DEFAULT_LEFT_TARGET_MM_S      (500)
#define SPEED_PID_DEFAULT_RIGHT_TARGET_MM_S     (500)

/*
 * Fixed-point gains scaled by PID_GAIN_SCALE.
 * Example: 2500 means 2.500.
 */
#define SPEED_PID_LEFT_KP               (2000)
#define SPEED_PID_LEFT_KI               (80)
#define SPEED_PID_LEFT_KD               (0)
#define SPEED_PID_LEFT_INTEGRAL_LIMIT   (20000)
#define SPEED_PID_LEFT_MIN_START_PWM    (500)

#define SPEED_PID_RIGHT_KP              (1500)
#define SPEED_PID_RIGHT_KI              (0)
#define SPEED_PID_RIGHT_KD              (0)
#define SPEED_PID_RIGHT_INTEGRAL_LIMIT  (20000)
#define SPEED_PID_RIGHT_MIN_START_PWM   (500)

void pid_init(pid_t *pid,
              int32_t kp,
              int32_t ki,
              int32_t kd,
              int32_t integralLimit,
              int32_t outputLimit);
void pid_reset(pid_t *pid);
int32_t pid_calculate(pid_t *pid, int32_t target, int32_t feedback);

void speed_pid_init(void);
void speed_pid_set_target(int32_t leftTargetMmS, int32_t rightTargetMmS);
void speed_pid_set_left_gains(int32_t kp, int32_t ki, int32_t kd);
void speed_pid_set_right_gains(int32_t kp, int32_t ki, int32_t kd);
void speed_pid_set_left_integral_limit(int32_t integralLimit);
void speed_pid_set_right_integral_limit(int32_t integralLimit);
void speed_pid_control_update(void);
int32_t speed_pid_get_left_target(void);
int32_t speed_pid_get_right_target(void);
int32_t speed_pid_get_left_pwm(void);
int32_t speed_pid_get_right_pwm(void);
int32_t speed_pid_get_left_kp(void);
int32_t speed_pid_get_left_ki(void);
int32_t speed_pid_get_left_kd(void);
int32_t speed_pid_get_right_kp(void);
int32_t speed_pid_get_right_ki(void);
int32_t speed_pid_get_right_kd(void);

#endif
