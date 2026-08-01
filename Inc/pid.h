#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>

/* UART0 belongs to the vision link in ball-control tasks. */
#define SPEED_PID_VOFA_ENABLE                    (0U)

/*
 * 通用 PID 结构体
 *
 * 本项目里 PID 参数使用定点缩放：
 *   实际增益 = kp/1000、ki/1000、kd/1000。
 *
 * 例如：
 *   kp = 2200 表示 Kp = 2.200。
 */
typedef struct {
    /* 比例、积分、微分参数，均按 PID_GAIN_SCALE 缩放。 */
    int32_t kp;
    int32_t ki;
    int32_t kd;

    /* 积分累计值。 */
    int32_t integral;

    /* 上一次误差，用于计算 D 项。 */
    int32_t previousError;

    /* 积分限幅，防止积分无限累积。 */
    int32_t integralLimit;

    /* 输出限幅，速度环里对应 PWM 最大值。 */
    int32_t outputLimit;
} pid_t;

/* PID 参数缩放系数。 */
#define PID_GAIN_SCALE      (1000)

/* 速度 PID 控制周期，单位 ms。当前与编码器测速周期保持一致。 */
#define SPEED_PID_CONTROL_PERIOD_MS     (10)

/*
 * 默认速度目标，单位 mm/s。
 * 注意：speed_pid_init() 后是否立即使用该默认值，要看 pid.c 里的实现。
 * 上层任务一般会主动调用 speed_pid_set_speed() 设置目标速度。
 */
#define SPEED_PID_DEFAULT_LEFT_TARGET_MM_S      (0)
#define SPEED_PID_DEFAULT_RIGHT_TARGET_MM_S     (800)

/*
 * 左右轮速度 PID 默认参数。
 *
 * 参数均按 PID_GAIN_SCALE 缩放：
 *   2200 表示 2.200。
 *
 * MIN_START_PWM：
 *   用于克服电机静摩擦。目标速度非 0 且 PID 输出太小时，
 *   会补偿一个最小启动 PWM。
 */
#define SPEED_PID_LEFT_KP               (6500)
#define SPEED_PID_LEFT_KI               (800)
#define SPEED_PID_LEFT_KD               (0)
#define SPEED_PID_LEFT_INTEGRAL_LIMIT   (12000)
#define SPEED_PID_LEFT_MIN_START_PWM    (500)

#define SPEED_PID_RIGHT_KP              (6000)
#define SPEED_PID_RIGHT_KI              (700)
#define SPEED_PID_RIGHT_KD              (0)
#define SPEED_PID_RIGHT_INTEGRAL_LIMIT  (12000)
#define SPEED_PID_RIGHT_MIN_START_PWM   (500)

/* 初始化一个通用 PID 控制器。 */
void pid_init(pid_t *pid,
              int32_t kp,
              int32_t ki,
              int32_t kd,
              int32_t integralLimit,
              int32_t outputLimit);

/* 清空 PID 积分和历史误差。 */
void pid_reset(pid_t *pid);

/* 计算一次 PID 输出。target/feedback 单位由上层决定。 */
int32_t pid_calculate(pid_t *pid, int32_t target, int32_t feedback);

/* 初始化左右轮速度 PID。 */
void speed_pid_init(void);

/*
 * 设置左右轮速度目标，供循迹、角度环、任务状态机调用。
 * 单位：mm/s。
 * 正负值表示轮子两个方向。
 */
void speed_pid_set_target(int32_t leftTargetMmS, int32_t rightTargetMmS);

/* speed_pid_set_target() 的同义接口，名字更直观。 */
void speed_pid_set_speed(int32_t leftSpeedMmS, int32_t rightSpeedMmS);

/* 单独设置某一侧轮速目标。 */
void speed_pid_set_left_speed(int32_t leftSpeedMmS);
void speed_pid_set_right_speed(int32_t rightSpeedMmS);

/* 速度目标清零，并复位 PID 状态。 */
void speed_pid_stop(void);

/* 在线调整左右轮 PID 参数。 */
void speed_pid_set_left_gains(int32_t kp, int32_t ki, int32_t kd);
void speed_pid_set_right_gains(int32_t kp, int32_t ki, int32_t kd);

/* 在线调整积分限幅。 */
void speed_pid_set_left_integral_limit(int32_t integralLimit);
void speed_pid_set_right_integral_limit(int32_t integralLimit);

/* 执行一次速度 PID：读取编码器速度，计算 PWM，下发电机。 */
void speed_pid_control_update(void);

/* 调试读取接口：目标速度、PWM、PID 参数。 */
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
