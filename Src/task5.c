#include <stdbool.h>
#include <stdint.h>

#include "delay.h"
#include "bluetooth.h"
#include "encoder.h"
#include "gray_serial.h"
#include "line_track.h"
#include "oled.h"
#include "pid.h"
#include "task5.h"
#include "uart_cmd.h"

#define TASK5_OLED_REFRESH_PERIOD_MS      (200U)
#define TASK5_STOP_ACTIVE_SENSOR_COUNT    (3U)
#define TASK5_STOP_ENABLE_DELAY_MS        (2000U)
#define TASK5_STOP_APPROACH_SPEED_MM_S    (300)
#define TASK5_STOP_APPROACH_DISTANCE_MM   (150)
#define TASK5_STOP_APPROACH_SLOW_DELAY_MS (0U)
#define TASK5_GRAY_STARTUP_DISCARD_COUNT  (20U)
#define TASK5_GRAY_STARTUP_DISCARD_MS     (2U)

#define TASK5_STABLE_LINE_BASE_SPEED_MM_S       LINE_TRACK_BASE_SPEED_MM_S
#define TASK5_STABLE_SMALL_TURN_PERCENT          LINE_TRACK_SMALL_TURN_INNER_PERCENT
#define TASK5_STABLE_LARGE_TURN_PERCENT          LINE_TRACK_LARGE_TURN_INNER_PERCENT

typedef enum {
    TASK5_LINE_STATE_RUNNING = 0,
    TASK5_LINE_STATE_APPROACHING_STOP,
    TASK5_LINE_STATE_STOPPED
} task5_line_state_t;

static void task5_apply_stable_line_params(void)
{
    line_track_set_base_speed(TASK5_STABLE_LINE_BASE_SPEED_MM_S);
    (void)line_track_set_turn_ratios(
        TASK5_STABLE_SMALL_TURN_PERCENT,
        TASK5_STABLE_LARGE_TURN_PERCENT);
}

static void task5_discard_startup_gray_samples(void)
{
    for (uint8_t i = 0U; i < TASK5_GRAY_STARTUP_DISCARD_COUNT; i++) {
        (void)gray_serial_read();
        delay_ms(TASK5_GRAY_STARTUP_DISCARD_MS);
    }
}

static uint8_t task5_count_active_gray_sensors(uint8_t raw)
{
    uint8_t activeCount = 0U;

    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t level = (uint8_t)((raw >> i) & 0x01U);

        if (level == LINE_TRACK_ACTIVE_LEVEL) {
            activeCount++;
        }
    }

    return activeCount;
}

static bool task5_should_exit_line_track(uint8_t activeCount)
{
    return activeCount >= TASK5_STOP_ACTIVE_SENSOR_COUNT;
}

static int32_t task5_abs_i32(int32_t value)
{
    if (value < 0) {
        return -value;
    }

    return value;
}

static int32_t task5_encoder_counts_to_mm(int32_t counts)
{
    int64_t distanceMmX1000;
    int64_t countsPerWheelRevX1000 =
        (int64_t)ENCODER_LINES_PER_MOTOR_REV *
        ENCODER_QUADRATURE_MULTIPLIER *
        ENCODER_GEAR_RATIO_X1000;
    int64_t wheelCircumferenceMmX1000 =
        ((int64_t)ENCODER_WHEEL_DIAMETER_MM * 3141593LL) / 1000LL;

    distanceMmX1000 =
        (int64_t)task5_abs_i32(counts) * wheelCircumferenceMmX1000;

    return (int32_t)((distanceMmX1000 +
                      (countsPerWheelRevX1000 / 2LL)) /
                     countsPerWheelRevX1000);
}

static int32_t task5_get_traveled_distance_mm(int32_t startLeftCount,
                                              int32_t startRightCount)
{
    int32_t leftDistanceMm = task5_encoder_counts_to_mm(
        encoder_get_left_count() - startLeftCount);
    int32_t rightDistanceMm = task5_encoder_counts_to_mm(
        encoder_get_right_count() - startRightCount);

    return (leftDistanceMm + rightDistanceMm) / 2;
}

static bool task5_car_speed_is_zero(void)
{
    return (encoder_get_left_speed_mm_s() == 0) &&
           (encoder_get_right_speed_mm_s() == 0);
}

static void task5_oled_print_line_header(uint8_t page, const char *text)
{
    oled_clear_line(page);
    oled_print_string(text);
}

static void task5_oled_update(bool oledOk,
                              uint8_t grayRaw,
                              uint8_t activeCount,
                              bool stopArmed,
                              task5_line_state_t lineState,
                              uint32_t elapsedMs)
{
    static uint32_t lastRefreshMs = 0U;
    uint32_t nowMs;
    line_track_status_t status;

    if (!oledOk) {
        return;
    }

    nowMs = delay_get_ms();
    if ((uint32_t)(nowMs - lastRefreshMs) < TASK5_OLED_REFRESH_PERIOD_MS) {
        return;
    }
    lastRefreshMs = nowMs;

    line_track_get_status(&status);

    task5_oled_print_line_header(0U, "Task5 Line");

    task5_oled_print_line_header(1U, "RAW:");
    oled_print_hex_u8(grayRaw);
    oled_print_string(" A:");
    oled_print_int(activeCount);
    if (lineState == TASK5_LINE_STATE_STOPPED) {
        status.correction = 0;
        status.leftTargetMmS = 0;
        status.rightTargetMmS = 0;
        oled_print_string(" STOP");
    } else if (lineState == TASK5_LINE_STATE_APPROACHING_STOP) {
        oled_print_string(" SLOW");
    } else if (!stopArmed) {
        oled_print_string(" WAIT");
    } else if (!status.lineDetected) {
        oled_print_string(" LOST");
    } else {
        oled_print_string(" RUN");
    }

    task5_oled_print_line_header(2U, "ERR:");
    oled_print_int(status.error);
    oled_print_string(" C:");
    oled_print_int(status.correction);

    task5_oled_print_line_header(3U, "L:");
    oled_print_int(status.leftTargetMmS);
    oled_print_string(" R:");
    oled_print_int(status.rightTargetMmS);

    task5_oled_print_line_header(4U, "Small:");
    oled_print_int(line_track_get_small_turn_percent());
    oled_print_string("% Big:");
    oled_print_int(line_track_get_large_turn_percent());
    oled_print_string("%");

    task5_oled_print_line_header(5U, "Base:");
    oled_print_int(line_track_get_base_speed());
    oled_print_string(" mm/s");

    oled_print_time_large(elapsedMs);
}

void task5_run(void)
{
    uint8_t grayRaw;
    uint8_t activeCount;
    uint32_t taskStartMs;
    uint32_t nowMs;
    uint32_t elapsedMs = 0U;
    bool timerStopped = false;
    uint32_t stopDetectMs = 0U;
    int32_t stopStartLeftCount = 0;
    int32_t stopStartRightCount = 0;
    bool stopArmed;
    task5_line_state_t lineState = TASK5_LINE_STATE_RUNNING;
    bool oledOk;

    gray_serial_init();
    task5_discard_startup_gray_samples();
    encoder_init();
    speed_pid_init();
    line_track_init();
    task5_apply_stable_line_params();
    uart_cmd_init();
    bluetooth_init();
    oledOk = oled_init();
    taskStartMs = delay_get_ms();

    while (1) {
        uart_cmd_process();
        bluetooth_process();

        nowMs = delay_get_ms();
        grayRaw = gray_serial_read();
        activeCount = task5_count_active_gray_sensors(grayRaw);
        stopArmed = ((uint32_t)(nowMs - taskStartMs) >=
                     TASK5_STOP_ENABLE_DELAY_MS);

        if ((lineState == TASK5_LINE_STATE_RUNNING) && stopArmed &&
            task5_should_exit_line_track(activeCount)) {
            lineState = TASK5_LINE_STATE_APPROACHING_STOP;
            stopDetectMs = nowMs;
            stopStartLeftCount = encoder_get_left_count();
            stopStartRightCount = encoder_get_right_count();
        }

        if (lineState == TASK5_LINE_STATE_APPROACHING_STOP) {
            if ((uint32_t)(nowMs - stopDetectMs) >=
                TASK5_STOP_APPROACH_SLOW_DELAY_MS) {
                line_track_set_base_speed(TASK5_STOP_APPROACH_SPEED_MM_S);
            }
            line_track_update_with_raw(grayRaw);

            if (task5_get_traveled_distance_mm(stopStartLeftCount,
                                               stopStartRightCount) >=
                TASK5_STOP_APPROACH_DISTANCE_MM) {
                lineState = TASK5_LINE_STATE_STOPPED;
                speed_pid_stop();
            }
        } else if (lineState == TASK5_LINE_STATE_STOPPED) {
            speed_pid_stop();
        } else {
            line_track_update_with_raw(grayRaw);
        }

        speed_pid_control_update();
        if (!timerStopped) {
            elapsedMs = nowMs;
            if ((lineState == TASK5_LINE_STATE_STOPPED) && task5_car_speed_is_zero()) {
                timerStopped = true;
            }
        }
        task5_oled_update(oledOk, grayRaw, activeCount,
                          stopArmed, lineState, elapsedMs);
        delay_ms(SPEED_PID_CONTROL_PERIOD_MS);
    }
}
