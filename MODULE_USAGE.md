# C07A 模块使用说明

本文档说明当前仓库中各个模块的用途、接入顺序、常用接口和注意事项。  
证据等级按仓库规则使用：

- A：可由当前代码、配置或构建文件直接确认。
- B：根据代码推测，需要人工确认。
- C：需要原理图、开发环境或实车验证。

注意：模块文件存在不等于当前 `main()` 已经调用；构建通过也不等于实车验证通过。

## 当前 main() 运行基线

证据等级：A，来源 `empty.c`。

当前 `main()` 是超声波 OLED 测距测试入口，初始化顺序为：

1. `SYSCFG_DL_init()`
2. `ultrasonic_init()`
3. `oled_init()`

当前主循环顺序为：

1. `ultrasonic_measure(&measurement)`
2. `ultrasonic_test_show_measurement(&measurement)`
3. `delay_ms(ULTRASONIC_TEST_PERIOD_MS)`

当前 `SysTick_Handler()` 调用：

1. `delay_tick()`

因此当前实际参与主循环控制的数据链路是：

超声波模块 -> `ultrasonic` -> `oled` -> OLED 显示。

当前未在 `main()` 中调用的模块包括：`gray_serial`、`encoder`、`speed_pid`、`motor`、`line_track`、`square_track`、`mpu6050`、`attitude`、`angle_control`、`bluetooth`、`uart_cmd`、`vofa`、`icm42688` 的显式初始化或周期调用。  
其中 `speed_pid_control_update()` 内部是否发送 VOFA 数据，需要以 `pid.c` 当前实现为准。

## 通用接入规则

1. 所有硬件模块必须在 `SYSCFG_DL_init()` 之后初始化。
2. 不要在中断里做 OLED 刷新、I2C 轮询、阻塞串口发送、复杂字符串解析或 `delay_ms()`。
3. 上层控制只写左右轮目标速度，最终 PWM 写入应统一由 `speed_pid_control_update()` 触发。
4. 新增 `.c` 文件后，要检查 CCS 构建列表是否包含该文件。
5. 修改外设宏、引脚或实例名称时，先审查 `empty.syscfg`、生成的 `ti_msp_dl_config.*` 和所有源码调用点。
6. 修改控制周期时，要同步检查编码器测速周期、PID 积分/微分计算和上层控制调用顺序。

## `delay` 时间基准模块

文件：

- `Inc/delay.h`
- `Src/delay.c`

作用：提供毫秒计数、阻塞延时和系统时间读取。

使用步骤：

1. 保证 SysTick 为 1 ms 节拍。
2. 在 `SysTick_Handler()` 中调用 `delay_tick()`。
3. 主循环或初始化阶段可调用 `delay_ms(ms)`。
4. 需要非阻塞计时时调用 `delay_get_ms()`。

常用接口：

- `delay_tick()`：1 ms 调用一次。
- `delay_ms(uint32_t ms)`：阻塞延时。
- `delay_get_ms()`：读取上电以来的毫秒计数。

注意事项：

- `delay_ms()` 不要放入 ISR。
- 只看到主循环末尾 `delay_ms(10)`，不能证明控制周期严格为 10 ms，因为 OLED、I2C、串口输出等都会占用时间。

## `motor` 电机 PWM 输出模块

文件：

- `Inc/motor.h`
- `Src/motor.c`

作用：把 signed PWM 转换为方向 GPIO 和 PWM 占空比。

常用接口：

- `motor_set_pwm(int pwmL, int pwmR)`

输入含义：

- 正数：一个约定方向转动。
- 负数：反方向转动。
- 0：停止输出。
- PWM 会被限制到 `MOTOR_PWM_MIN` 到 `MOTOR_PWM_MAX`。

当前边界行为：

- `motor.c` 内部在输出前交换了一次左右 PWM，用于适配当前后驱接线映射。证据等级 A。

注意事项：

- 普通循迹、角度环、速度环不要直接调用 `motor_set_pwm()`。
- 推荐路径是：上层控制写 `speed_pid_set_speed()`，再由 `speed_pid_control_update()` 统一计算并写 PWM。
- 若要直接测试电机 PWM，必须先获得用户明确授权，并按低限幅、架空轮测、低速落地的顺序验证。

## `encoder` 编码器模块

文件：

- `Inc/encoder.h`
- `Src/encoder.c`

作用：通过 GPIO 中断累计左右轮编码器计数，并按固定周期估算轮速。

使用步骤：

1. `SYSCFG_DL_init()` 后调用 `encoder_init()`。
2. 在 `SysTick_Handler()` 中每 1 ms 调用 `encoder_tick_1ms()`。
3. 保证 GPIO 中断入口会调用 `encoder_gpio_irq_handler()`。
4. 主循环或控制环读取 `encoder_get_left_speed_mm_s()` 和 `encoder_get_right_speed_mm_s()`。

常用接口：

- `encoder_init()`
- `encoder_tick_1ms()`
- `encoder_get_left_count()`
- `encoder_get_right_count()`
- `encoder_get_left_speed_mm_s()`
- `encoder_get_right_speed_mm_s()`
- `encoder_reset_count()`

关键参数：

- `ENCODER_LINES_PER_MOTOR_REV`
- `ENCODER_QUADRATURE_MULTIPLIER`
- `ENCODER_GEAR_RATIO_X1000`
- `ENCODER_WHEEL_DIAMETER_MM`
- `ENCODER_SPEED_PERIOD_MS`
- `ENCODER_LEFT_DIR`
- `ENCODER_RIGHT_DIR`

注意事项：

- 编码器线数、减速比、轮径会直接影响 mm/s 换算。
- 左右方向修正只应改 `ENCODER_LEFT_DIR` / `ENCODER_RIGHT_DIR`，不要在多个模块里重复取反。
- 实车方向是否正确属于 C，需要架空轮测确认。

## `pid` 速度环和通用 PID 模块

文件：

- `Inc/pid.h`
- `Src/pid.c`

作用：提供通用 PID 计算器和左右轮速度闭环。

使用步骤：

1. 初始化编码器。
2. 调用 `speed_pid_init()`。
3. 上层模块调用 `speed_pid_set_speed(left, right)` 设置目标速度，单位 mm/s。
4. 按固定周期调用 `speed_pid_control_update()`。
5. `speed_pid_control_update()` 读取编码器速度，计算 PWM，并调用 `motor_set_pwm()`。

常用接口：

- `pid_init()`
- `pid_calculate()`
- `speed_pid_init()`
- `speed_pid_set_speed()`
- `speed_pid_stop()`
- `speed_pid_control_update()`
- `speed_pid_set_left_gains()`
- `speed_pid_set_right_gains()`
- `speed_pid_set_left_integral_limit()`
- `speed_pid_set_right_integral_limit()`

参数约定：

- PID 参数按 `/1000` 使用，例如 `6500` 表示实际 Kp 为 `6.500`。
- 速度单位为 mm/s。
- PWM 输出由 `motor` 模块限幅。

注意事项：

- 速度环应是最终电机 PWM 写入路径，不要让多个模块同时写 PWM。
- 调整 `SPEED_PID_CONTROL_PERIOD_MS` 时，要同步检查编码器测速周期、积分项、微分项和主循环阻塞耗时。
- 积分限幅和 PWM 限幅需要配合电机实际最大速度检查；这类结论最终需要实车确认。

## `gray_serial` 八路灰度串行读取模块

文件：

- `Inc/gray_serial.h`
- `Src/gray_serial.c`

作用：通过 CLK/DAT 两根线读取 8 路灰度传感器数字量。

使用步骤：

1. `SYSCFG_DL_init()` 后调用 `gray_serial_init()`。
2. 主循环中调用 `gray_serial_read()` 读取一次 8 bit 原始值。
3. 需要调试时可用 `gray_serial_print(raw)` 打印从左到右的通道状态。

常用接口：

- `gray_serial_init()`
- `gray_serial_read()`
- `gray_serial_print(uint8_t value)`

注意事项：

- 原始 bit 顺序不一定等于物理从左到右顺序；循迹模块里使用映射表转换。
- 当前循迹宏 `LINE_TRACK_ACTIVE_LEVEL` 表示“检测到黑线”的有效电平。
- 传感器真实接线、电平和安装方向需要实车确认。

## `ultrasonic` 超声波测距模块

文件：

- `Inc/ultrasonic.h`
- `Src/ultrasonic.c`

作用：驱动 HC-SR04 类 TRIG/ECHO 超声波模块，读取距离，单位 mm。

默认软件引脚：

- PA24：TRIG 输出。
- PA9：ECHO 输入。

证据等级：

- A：当前 `ultrasonic.c` 本地定义 PA24/PA9，并在 `ultrasonic_init()` 中初始化为普通 GPIO。
- A：当前 `empty.syscfg` 和生成的 `ti_msp_dl_config.*` 没有为该模块生成专用宏。
- C：真实接线、供电电压、ECHO 电平是否适配 MCU、模块型号和实际测距效果需要硬件确认。

使用步骤：

1. `SYSCFG_DL_init()` 后调用 `ultrasonic_init()`。
2. 主循环中低频调用 `ultrasonic_measure(&measurement)` 或 `ultrasonic_read_mm(&distanceMm)`。
3. 若需要显示，可把 `distanceMm` 用 OLED 或 VOFA 输出。

常用接口：

- `ultrasonic_init()`
- `ultrasonic_measure(ultrasonic_measurement_t *measurement)`
- `ultrasonic_read_mm(uint16_t *distanceMm)`
- `ultrasonic_get_last_measurement()`

返回值说明：

- `ULTRASONIC_STATUS_OK`：测距成功。
- `ULTRASONIC_STATUS_ECHO_START_TIMEOUT`：触发后没有等到 ECHO 拉高。
- `ULTRASONIC_STATUS_ECHO_END_TIMEOUT`：ECHO 拉高后长时间没有落下。
- `ULTRASONIC_NO_OBJECT_MM`：无有效距离，当前定义为 `0xFFFF`。

注意事项：

- 当前 `main()` 已接入该模块作为 OLED 测距测试入口。证据等级 A。
- 当前测距函数是阻塞式，会等待 ECHO 起始和结束，最长可阻塞约几十毫秒。
- 不要在 ISR 中调用测距函数。
- 不建议放在 10 ms 速度环每个周期里调用；可以每 50 ms 到 100 ms 调一次。
- 若使用 5V 超声波模块，ECHO 可能是 5V 电平，是否需要分压或电平转换属于硬件确认项。
- 如果希望由 SysConfig 长期管理 PA24/PA9，应另行添加 SysConfig GPIO 资源并重新生成配置，不要只手改生成文件。

## `line_track` 普通循迹模块

文件：

- `Inc/line_track.h`
- `Src/line_track.c`

作用：把 8 路灰度数据转换为循迹误差，并用 PD 生成左右轮目标速度。

使用步骤：

1. 初始化灰度模块、编码器和速度环。
2. 调用 `line_track_init()`。
3. 周期调用以下接口之一：
   - `line_track_update()`：模块内部读取灰度，丢线后按历史误差旋转找线。
   - `line_track_update_with_raw(raw)`：使用外部传入 raw，丢线时停车。
   - `line_track_update_with_raw_search_on_lost(raw)`：使用外部传入 raw，丢线后找线。
4. 随后调用 `speed_pid_control_update()`。

常用接口：

- `line_track_init()`
- `line_track_set_base_speed()`
- `line_track_set_turn_kp()`
- `line_track_set_turn_kd()`
- `line_track_set_max_correction()`
- `line_track_get_status()`

参数约定：

- `LINE_TRACK_ACTIVE_LEVEL`：灰度检测到黑线时的有效电平。
- `LINE_TRACK_BASE_SPEED_MM_S`：基础前进速度。
- `LINE_TRACK_TURN_KP` / `LINE_TRACK_TURN_KD`：按 `/1000` 使用。
- `LINE_TRACK_MAX_CORRECTION_MM_S`：最大差速修正量。

注意事项：

- 当前误差约定由代码确认：左侧通道权重为正，右侧通道权重为负。
- 当前混控为 `left = base + correction`，`right = base - correction`。
- 如果实车越修越偏，不要先改 PID；先确认传感器左右映射、电机方向、编码器方向和 `motor` 左右交换。

## `square_track` 正方形循迹状态机

文件：

- `Inc/square_track.h`
- `Src/square_track.c`

作用：封装正方形 ABCD 路径逻辑。

当前状态：

- `SQUARE_TRACK_STATE_TRACK`：正常沿边循迹。
- `SQUARE_TRACK_STATE_ADVANCE_AFTER_LOST`：全丢线后继续向前一段距离。
- `SQUARE_TRACK_STATE_TURN_RIGHT`：原地右转，直到中间两个通道重新检测到黑线。

使用步骤：

1. 初始化灰度、编码器和速度环。
2. 调用 `square_track_init()`。
3. 主循环中先调用 `square_track_update()`。
4. 再调用 `speed_pid_control_update()`。
5. 需要显示状态时调用 `square_track_get_status()`。

常用接口：

- `square_track_init()`
- `square_track_update()`
- `square_track_get_status()`
- `square_track_get_segment_start_label()`
- `square_track_get_segment_end_label()`

当前关键参数在 `Src/square_track.c` 中：

- 基础速度：`SQUARE_TRACK_BASE_SPEED_MM_S`
- 右转速度：`SQUARE_TRACK_TURN_SPEED_MM_S`
- 丢线后前进距离：`SQUARE_TRACK_FORWARD_AFTER_LOST_MM`
- 判定横线/宽线的通道数量：`SQUARE_TRACK_STRAIGHT_ACTIVE_MIN`
- 判定转弯结束的中间通道：`SQUARE_TRACK_CENTER_LEFT_INDEX` 和 `SQUARE_TRACK_CENTER_RIGHT_INDEX`

注意事项：

- 本模块会调用 `speed_pid_set_speed()` 写左右目标速度。
- 当前 `main()` 已接入该模块。证据等级 A。
- 正方形尺寸、14 cm 前进距离、右转是否正好 90 度都需要实车确认。

## `oled` OLED 显示模块

文件：

- `Inc/oled.h`
- `Src/oled.c`

作用：初始化 OLED 并显示字符串、整数、浮点数和十六进制数。

使用步骤：

1. `SYSCFG_DL_init()` 后调用 `oled_init()`。
2. 使用 `oled_clear()` 清屏。
3. 使用 `oled_clear_line(page)` 清指定页。
4. 使用打印接口显示调试信息。

常用接口：

- `oled_init()`
- `oled_clear()`
- `oled_clear_line(uint8_t page)`
- `oled_set_cursor(uint8_t page, uint8_t column)`
- `oled_print_string()`
- `oled_print_int()`
- `oled_print_float()`
- `oled_print_hex_u8()`

注意事项：

- OLED 刷新放在主循环中，避免放入 ISR。
- OLED 刷新会占用时间，频繁刷新会影响控制周期。
- 当前 `main()` 每约 100 ms 刷新一次 OLED。证据等级 A。

## `vofa` 串口曲线输出模块

文件：

- `Inc/vofa.h`
- `Src/vofa.c`

作用：通过 UART0 输出 VOFA 可解析的多通道数据。

常用接口：

- `uart0_send_byte()`
- `uart0_send_string()`
- `uart0_send_int()`
- `uart0_send_float()`
- `vofa_send_two_int()`
- `vofa_send_six_int()`
- `vofa_send_six_float()`

输出格式：

- 推荐格式为 `samples:ch0,ch1,ch2,...\n`。

注意事项：

- 串口发送通常是阻塞式，不能放在 ISR。
- VOFA 输出频率过高会拖慢主循环。
- 当前 `main()` 未显式调用 VOFA；速度环内部是否输出，以 `pid.c` 当前实现为准。

## `uart_cmd` UART0 速度环调参模块

文件：

- `Inc/uart_cmd.h`
- `Src/uart_cmd.c`

作用：从 UART0 接收一行命令，解析速度目标和左右轮速度 PID 参数。

使用步骤：

1. `SYSCFG_DL_init()` 后调用 `uart_cmd_init()`。
2. UART0 中断入口调用 `uart_cmd_irq_handler()`；当前模块文件中也定义了 `UART0_IRQHandler()`。
3. 主循环中频繁调用 `uart_cmd_process()`。

当前命令格式：

一行中解析 8 个整数，顺序为：

1. 左轮目标速度
2. 左轮 Kp
3. 左轮 Ki
4. 左轮 Kd
5. 右轮目标速度
6. 右轮 Kp
7. 右轮 Ki
8. 右轮 Kd

示例：

```text
300 6500 800 0 300 6000 700 0
```

注意事项：

- 解析在主循环中完成，中断只收字节。
- 当前 `main()` 未接入 `uart_cmd_init()` 和 `uart_cmd_process()`。证据等级 A。
- 如果同时使用 VOFA 和 UART0 调参，要检查 UART0 输出和输入是否相互影响。

## `bluetooth` 蓝牙调参模块

文件：

- `Inc/bluetooth.h`
- `Src/bluetooth.c`

作用：通过 UART1 接收 `{...}` 格式命令，调节角度环和循迹环参数。

使用步骤：

1. `SYSCFG_DL_init()` 后调用 `bluetooth_init()`。
2. 主循环中频繁调用 `bluetooth_process()`。
3. UART1 中断入口由 `UART_1_INST_IRQHandler()` 接收字节到环形缓冲区。

常用命令：

- `{AKP=10000}`：角度环 Kp，`/1000` 缩放。
- `{AKD=0}`：角度环 Kd，`/1000` 缩放。
- `{ABS=300}`：角度环基础速度，单位 mm/s。
- `{AMX=220}`：角度环最大差速修正，单位 mm/s。
- `{ANG=90}`：以当前 yaw 为基准，目标角增加 90 度。
- `{LKP=250}`：循迹 Kp，`/1000` 缩放。
- `{LKD=120}`：循迹 Kd，`/1000` 缩放。
- `{LBS=300}`：循迹基础速度，单位 mm/s。
- `{LMX=280}`：循迹最大差速修正，单位 mm/s。
- `{GET}`：查询当前参数。

注意事项：

- 当前 `main()` 未接入蓝牙模块。证据等级 A。
- 蓝牙命令可能改变角度环和循迹环参数，使用前要确认当前主循环实际调用了对应控制环。
- `{ANG=...}` 依赖 `attitude_get_euler()` 的 yaw，因此需要姿态解算正在更新。

## `mpu6050` MPU6050 IMU 模块

文件：

- `Inc/mpu6050.h`
- `Src/mpu6050.c`

作用：使用硬件 I2C 初始化 MPU6050，并读取加速度、温度和陀螺仪原始数据。

使用步骤：

1. `SYSCFG_DL_init()` 后调用 `mpu6050_init()`。
2. 初始化成功后周期调用 `mpu6050_read_raw(&raw)`。
3. 需要诊断时调用 `mpu6050_get_who_am_i()` 或 `mpu6050_get_diag()`。
4. I2C 异常时可调用 `mpu6050_recover_i2c_bus()` 后重新初始化。

常用接口：

- `mpu6050_init()`
- `mpu6050_read_raw(mpu6050_raw_t *raw)`
- `mpu6050_get_who_am_i()`
- `mpu6050_get_diag(mpu6050_diag_t *diag)`
- `mpu6050_recover_i2c_bus()`

量程换算：

- 加速度：`MPU6050_ACCEL_G_PER_LSB`
- 陀螺仪：`MPU6050_GYRO_DPS_PER_LSB`

注意事项：

- 用户当前方向是后续只使用 MPU6050；ICM42688 保留为旧模块或兼容模块。
- 当前 `main()` 未初始化 MPU6050。证据等级 A。
- MPU6050 实际地址、供电、电平和安装方向需要硬件或实车确认。

## `attitude` 姿态解算模块

文件：

- `Inc/attitude.h`
- `Src/attitude.c`

作用：根据 IMU 原始数据更新 roll、pitch、yaw，并提供角速度补偿结果。

使用步骤：

1. 初始化 IMU。
2. 调用 `attitude_init()`。
3. 上电静止时做陀螺仪零偏校准。
4. 每个控制周期读取 IMU 原始数据。
5. 调用 `attitude_update_from_icm42688(&raw, dt)` 更新姿态。
6. 调用 `attitude_get_euler(&euler)` 读取欧拉角。

常用接口：

- `attitude_init()`
- `attitude_calibrate_gyro()`
- `attitude_calibrate_gyro_step()`
- `attitude_is_gyro_calibrated()`
- `attitude_update_from_icm42688()`
- `attitude_get_euler()`
- `attitude_get_gyro_z_dps()`

注意事项：

- 该接口当前使用 `icm42688_raw_t` 类型命名，但可以通过适配层把 MPU6050 原始值转换后复用解算算法。
- 零偏校准时车必须保持静止。
- 静止时 yaw 单方向缓慢漂移，通常优先检查陀螺仪零偏、死区、比例系数和采样周期。
- 当前 `main()` 未接入姿态解算。证据等级 A。

## `angle_control` 角度环模块

文件：

- `Inc/angle_control.h`
- `Src/angle_control.c`

作用：根据目标 yaw 和当前 yaw 计算左右轮差速修正，并下发给速度环。

使用步骤：

1. 初始化编码器、速度环、IMU 和姿态解算。
2. 调用 `angle_control_init()`。
3. 校准完成后调用 `angle_control_lock_current_yaw()` 或 `angle_control_set_target_yaw()`。
4. 调用 `angle_control_enable(true)`。
5. 周期顺序必须是：读取 IMU -> 更新姿态 -> `angle_control_update(dt)` -> `speed_pid_control_update()`。

常用接口：

- `angle_control_init()`
- `angle_control_enable(bool enable)`
- `angle_control_stop()`
- `angle_control_set_base_speed()`
- `angle_control_set_gains_scaled()`
- `angle_control_set_target_yaw()`
- `angle_control_lock_current_yaw()`
- `angle_control_update(float dt)`
- `angle_control_get_status()`

参数约定：

- yaw 单位为 deg。
- 基础速度和差速修正单位为 mm/s。
- Kp/Kd 调参接口按 `/1000` 缩放。

注意事项：

- 角度环只写速度目标，不直接写 PWM。
- 使用蓝牙 `{ANG=...}` 指令时，必须保证姿态 yaw 正在更新。
- 当前 `main()` 未接入角度环。证据等级 A。

## `icm42688` ICM42688 旧 IMU 模块

文件：

- `Inc/icm42688.h`
- `Src/icm42688.c`

作用：初始化和读取 ICM42688 原始数据。

常用接口：

- `icm42688_init()`
- `icm42688_get_who_am_i()`
- `icm42688_read_raw()`
- `icm42688_print_raw()`

注意事项：

- 用户当前已要求后续只使用 MPU6050，因此新 IMU 主流程应优先接入 `mpu6050`。
- `attitude` 当前接口名仍带 `icm42688_raw_t`，这是代码类型命名事实，不代表当前必须使用 ICM42688。
- 当前 `main()` 未接入 ICM42688。证据等级 A。

## 推荐组合模板

### 速度环调参模板

初始化：

```c
SYSCFG_DL_init();
encoder_init();
speed_pid_init();
uart_cmd_init();
```

主循环：

```c
uart_cmd_process();
speed_pid_control_update();
delay_ms(SPEED_PID_CONTROL_PERIOD_MS);
```

SysTick：

```c
delay_tick();
encoder_tick_1ms();
```

### 普通循迹模板

初始化：

```c
SYSCFG_DL_init();
gray_serial_init();
encoder_init();
speed_pid_init();
line_track_init();
```

主循环：

```c
line_track_update();
speed_pid_control_update();
delay_ms(SPEED_PID_CONTROL_PERIOD_MS);
```

### 正方形循迹模板

初始化：

```c
SYSCFG_DL_init();
gray_serial_init();
encoder_init();
speed_pid_init();
square_track_init();
oled_init();
```

主循环：

```c
square_track_update();
speed_pid_control_update();
```

### MPU6050 角度环模板

初始化：

```c
SYSCFG_DL_init();
encoder_init();
speed_pid_init();
mpu6050_init();
attitude_init();
angle_control_init();
bluetooth_init();
oled_init();
```

主循环顺序：

```c
bluetooth_process();
mpu6050_read_raw(&mpuRaw);
/* 将 MPU6050 原始数据适配给 attitude 更新接口。 */
attitude_update_from_icm42688(&attRaw, dt);
angle_control_update(dt);
speed_pid_control_update();
```

注意：上面的 MPU6050 到姿态输入的适配方式必须以当前代码实现为准，不要凭经验硬套量程。

### 超声波测距模板

初始化：

```c
SYSCFG_DL_init();
ultrasonic_init();
oled_init();
```

主循环：

```c
ultrasonic_measurement_t ultrasonic;

if (ultrasonic_measure(&ultrasonic)) {
    oled_clear_line(0U);
    oled_print_string("US:");
    oled_print_int(ultrasonic.distanceMm);
    oled_print_string("mm");
} else {
    oled_clear_line(0U);
    oled_print_string("US ERR:");
    oled_print_int((int32_t)ultrasonic.status);
}

delay_ms(100U);
```

注意：这个模板只用于读取距离，不会自动控制电机。

## 修改模块时的检查清单

1. 是否需要新增或修改 `.h` 公共接口。
2. 是否搜索了所有定义和调用点。
3. 是否新增 `.c` 文件，且确认 CCS 构建列表包含它。
4. 是否改变了控制周期或中断负载。
5. 是否新增了电机 PWM 写入点。
6. 是否改变了引脚、外设实例、UART 波特率、I2C 地址或 PWM 通道。
7. 是否区分了代码确认、推测和实车确认。
8. 是否只做了本次目标所需的最小修改。
