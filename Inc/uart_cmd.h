#ifndef __UART_CMD_H_
#define __UART_CMD_H_

#include <stdbool.h>
#include <stdint.h>

#define UART_CMD_VISION_FRAME_HEAD            (0xA5U)
#define UART_CMD_VISION_FRAME_TAIL            (0x5AU)
#define UART_CMD_VISION_CMD_BALL_STATE        (0x01U)
#define UART_CMD_VISION_CMD_DETECTION_STATE   (0x02U)
#define UART_CMD_VISION_CMD_HEARTBEAT         (0x03U)

/* These values match the supplied vision protocol template. */
#define UART_CMD_VISION_FRAME_TIMEOUT_MS       (10U)
#define UART_CMD_VISION_DATA_TIMEOUT_MS        (80U)
#define UART_CMD_VISION_LINK_TIMEOUT_MS        (600U)
#define UART_CMD_VISION_POSITION_LIMIT_X10     (1300)
#define UART_CMD_VISION_VELOCITY_LIMIT_X10     (20000)
#define UART_CMD_VISION_MAX_JUMP_X10           (300)
#define UART_CMD_VISION_JUMP_WINDOW_MS         (50U)
#define UART_CMD_VISION_STATS_WINDOW_MS        (1000U)

/* Apply these after confirming the camera coordinate convention. */
#define UART_CMD_VISION_POSITION_INVERT        (0U)
#define UART_CMD_VISION_POSITION_ZERO_X10      (0)

typedef struct {
    /* CMD 0x01: signed big-endian values, 0.1 mm and 0.1 mm/s per LSB. */
    int16_t positionX10;
    int16_t velocityX10;
    uint32_t dataTimestampMs;
    uint32_t linkTimestampMs;
    bool valid;
    bool fresh;
    bool linkOnline;
} uart_cmd_vision_sample_t;

typedef struct {
    uint16_t acceptedFrameRateHz;
    uint16_t ballStateFrameRateHz;
    uint16_t badFrameRatioPermille;
    uint16_t acceptedFrameCount;
    uint16_t badFrameCount;
    uint16_t rxOverflowCount;
    bool dataFresh;
    bool linkOnline;
} uart_cmd_vision_link_status_t;

/*
 * UART0 串口命令解析模块。
 * 接收中断只负责收字节，真正解析在 uart_cmd_process() 中进行，
 * 避免在中断里做复杂字符串处理或阻塞发送。
 *
 * 常用命令，回车或换行结束：
 *   GET                 查询步进状态
 *   S 3200 200          正向 3200 步，200Hz
 *   S -3200 200         反向 3200 步，200Hz
 *   SD 3200 200 0       指定方向运行，dir=0/1
 *   RUN 200 1           按 200Hz 连续转，方向为 1
 *   STOP                停止发 STEP 脉冲，保持使能
 *   EN 0                释放步进电机
 *   EN 1                使能步进电机
 *   ESTOP               紧急停止并锁定，禁止后续运动命令
 *   RESET               仅解除紧急锁定，仍需 EN 1 才可启动
 */

void uart_cmd_init(void);
void uart_cmd_process(void);
void uart_cmd_irq_handler(void);
/* Returns true only for a fresh, link-online, detection-valid vision sample. */
bool uart_cmd_get_vision_sample(uart_cmd_vision_sample_t *sample);
bool uart_cmd_is_vision_online(void);
void uart_cmd_get_vision_link_status(uart_cmd_vision_link_status_t *status);
uint16_t uart_cmd_get_vision_good_frame_count(void);
uint16_t uart_cmd_get_vision_bad_frame_count(void);

#endif
