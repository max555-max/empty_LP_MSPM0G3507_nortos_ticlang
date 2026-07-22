#ifndef __ANGLE_CONTROL_H_
#define __ANGLE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 航向角控制模块，也叫“角度环”
 *
 * 作用：
 *   用陀螺仪 yaw 角保持小车朝某个方向直行。
 *
 * 控制结构：
 *   目标 yaw / 当前 yaw
 *        ↓
 *   角度 PD，计算左右轮差速 correction
 *        ↓
 *   左轮目标速度 = baseSpeed - correction
 *   右轮目标速度 = baseSpeed + correction
 *        ↓
 *   交给 speed_pid 做速度闭环，最终输出 PWM。
 *
 * 单位：
 *   yaw angle  ：deg
 *   wheel speed：mm/s
 *   correction ：mm/s
 */

/*
 * ANGLE_CONTROL_DIRECTION：
 *   1.0f ：默认修正方向；
 *  -1.0f ：如果小车越修越偏，把这里改成 -1.0f。
 */
#define ANGLE_CONTROL_DIRECTION              (1.0f)

/*
 * 角度环参数。
 *
 * 调参建议：
 *   先只调 Kp，小车能回正但左右摆动时再少量加 Kd。
 *
 * Kp 单位：mm/s / deg
 * Kd 单位：mm/s / (deg/s)
 */
#define ANGLE_CONTROL_KP_MM_S_PER_DEG        (10.0f)
#define ANGLE_CONTROL_KD_MM_S_PER_DPS        (0.0f)

#define ANGLE_CONTROL_MAX_CORRECTION_MM_S    (220)  /* 差速修正限幅。 */
#define ANGLE_CONTROL_MAX_TARGET_MM_S        (900)  /* 输出给速度环的最大目标速度。 */

typedef struct {
    bool enabled;             /* 角度环是否使能。 */
    float targetYawDeg;       /* 目标航向角。 */
    float currentYawDeg;      /* 当前航向角。 */
    float errorDeg;           /* 航向误差，已经归一化到 -180~180。 */
    float errorRateDps;       /* 误差变化率，用于 D 项。 */
    int32_t baseSpeedMmS;     /* 直行基础速度。 */
    int32_t correctionMmS;    /* 差速修正量。 */
    int32_t leftTargetMmS;    /* 下发给速度环的左轮目标速度。 */
    int32_t rightTargetMmS;   /* 下发给速度环的右轮目标速度。 */
} angle_control_status_t;

/* 初始化角度环状态，默认不输出。 */
void angle_control_init(void);

/* 使能/关闭角度环。关闭时会把速度目标清零。 */
void angle_control_enable(bool enable);

/* 停止角度环并清零速度目标。 */
void angle_control_stop(void);

/*
 * 设置直行基础速度。
 * 正数前进，负数后退；角度环只是在此基础上叠加差速。
 */
void angle_control_set_base_speed(int32_t baseSpeedMmS);

/*
 * 锁定当前 yaw 为目标方向。
 * 适合“当前朝向就是接下来要保持方向”的直行段。
 */
void angle_control_lock_current_yaw(void);

/*
 * 手动设置目标 yaw。
 * 适合 task2/task3 这类已经知道几何目标角度的路径。
 */
void angle_control_set_target_yaw(float yawDeg);

/*
 * 执行一次角度环更新。
 * dt 单位：秒。
 * 常见调用顺序：
 *   attitude_update_from_icm42688(...);
 *   angle_control_update(dt);
 *   speed_pid_control_update();
 */
void angle_control_update(float dt);

/* 读取角度环状态，方便 VOFA/串口调试。 */
void angle_control_get_status(angle_control_status_t *status);

#endif
