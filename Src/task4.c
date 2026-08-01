#include <stdbool.h>
#include <stdint.h>

#include "bluetooth.h"
#include "delay.h"
#include "encoder.h"
#include "gray_serial.h"
#include "line_track.h"
#include "oled.h"
#include "pid.h"
#include "task4.h"

/*
 * Task4 parameters are deliberately separate from Task2.  Change these
 * values when Task4 is selected from main(), without changing Task2.
 */
#define TASK4_OLED_REFRESH_PERIOD_MS              (200U)
#define TASK4_STOP_DETECT_ENABLE_DISTANCE_MM      (6000)
#define TASK4_GRAY_STARTUP_DISCARD_COUNT          (20U)
#define TASK4_GRAY_STARTUP_DISCARD_MS             (2U)
#define TASK4_LINE_BASE_SPEED_MM_S                (350)
#define TASK4_SMALL_TURN_PERCENT                  (90)
#define TASK4_LARGE_TURN_PERCENT                  (60)

typedef enum {
    TASK4_LINE_STATE_RUNNING = 0,
    TASK4_LINE_STATE_STOPPED
} task4_line_state_t;

static void task4_apply_line_params(void)
{
    line_track_set_base_speed(TASK4_LINE_BASE_SPEED_MM_S);
    (void)line_track_set_turn_ratios(TASK4_SMALL_TURN_PERCENT,
                                     TASK4_LARGE_TURN_PERCENT);
}

static void task4_discard_startup_gray_samples(void)
{
    for (uint8_t i = 0U; i < TASK4_GRAY_STARTUP_DISCARD_COUNT; i++) {
        (void)gray_serial_read();
        delay_ms(TASK4_GRAY_STARTUP_DISCARD_MS);
    }
}

static uint8_t task4_count_active_gray_sensors(uint8_t raw)
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

static bool task4_stop_marker_detected(uint8_t raw)
{
    uint8_t o2Level = (uint8_t)((raw >> 1U) & 0x01U);
    uint8_t o3Level = (uint8_t)((raw >> 2U) & 0x01U);

    return (o2Level == LINE_TRACK_ACTIVE_LEVEL) &&
           (o3Level == LINE_TRACK_ACTIVE_LEVEL);
}

static int32_t task4_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t task4_encoder_counts_to_mm(int32_t counts)
{
    int64_t distanceMmX1000;
    int64_t countsPerWheelRevX1000 =
        (int64_t)ENCODER_LINES_PER_MOTOR_REV *
        ENCODER_QUADRATURE_MULTIPLIER *
        ENCODER_GEAR_RATIO_X1000;
    int64_t wheelCircumferenceMmX1000 =
        ((int64_t)ENCODER_WHEEL_DIAMETER_MM * 3141593LL) / 1000LL;

    distanceMmX1000 =
        (int64_t)task4_abs_i32(counts) * wheelCircumferenceMmX1000;

    return (int32_t)((distanceMmX1000 +
                      (countsPerWheelRevX1000 / 2LL)) /
                     countsPerWheelRevX1000);
}

static int32_t task4_get_traveled_distance_mm(int32_t startLeftCount,
                                              int32_t startRightCount)
{
    int32_t leftDistanceMm = task4_encoder_counts_to_mm(
        encoder_get_left_count() - startLeftCount);
    int32_t rightDistanceMm = task4_encoder_counts_to_mm(
        encoder_get_right_count() - startRightCount);

    return (leftDistanceMm + rightDistanceMm) / 2;
}

static void task4_oled_print_line_header(uint8_t page, const char *text)
{
    oled_clear_line(page);
    oled_print_string(text);
}

static void task4_oled_update(bool oledOk,
                              uint8_t grayRaw,
                              uint8_t activeCount,
                              task4_line_state_t lineState,
                              bool stopDetectEnabled,
                              int32_t traveledDistanceMm,
                              uint32_t elapsedMs)
{
    static uint32_t lastRefreshMs = 0U;
    uint32_t nowMs;
    line_track_status_t status;

    if (!oledOk) {
        return;
    }

    nowMs = delay_get_ms();
    if ((uint32_t)(nowMs - lastRefreshMs) < TASK4_OLED_REFRESH_PERIOD_MS) {
        return;
    }
    lastRefreshMs = nowMs;

    line_track_get_status(&status);

    task4_oled_print_line_header(0U, "Task4 Line");

    task4_oled_print_line_header(1U, "RAW:");
    oled_print_hex_u8(grayRaw);
    oled_print_string(" A:");
    oled_print_int(activeCount);
    if (lineState == TASK4_LINE_STATE_STOPPED) {
        status.correction = 0;
        status.leftTargetMmS = 0;
        status.rightTargetMmS = 0;
        oled_print_string(" STOP");
    } else if (!status.lineDetected) {
        oled_print_string(" LOST");
    } else if (stopDetectEnabled) {
        oled_print_string(" ARM");
    } else {
        oled_print_string(" RUN");
    }

    task4_oled_print_line_header(2U, "ERR:");
    oled_print_int(status.error);
    oled_print_string(" C:");
    oled_print_int(status.correction);

    task4_oled_print_line_header(3U, "L:");
    oled_print_int(status.leftTargetMmS);
    oled_print_string(" R:");
    oled_print_int(status.rightTargetMmS);

    task4_oled_print_line_header(4U, "Small:");
    oled_print_int(line_track_get_small_turn_percent());
    oled_print_string("% Big:");
    oled_print_int(line_track_get_large_turn_percent());
    oled_print_string("%");

    task4_oled_print_line_header(5U, "Dist:");
    oled_print_int(traveledDistanceMm);
    oled_print_string("/");
    oled_print_int(TASK4_STOP_DETECT_ENABLE_DISTANCE_MM);

    oled_print_time_large(elapsedMs);
}

void task4_run(void)
{
    uint8_t grayRaw;
    uint8_t activeCount;
    uint32_t taskStartMs;
    uint32_t nowMs;
    uint32_t elapsedMs = 0U;
    bool timerStopped = false;
    bool stopDetectEnabled = false;
    int32_t traveledDistanceMm = 0;
    int32_t runStartLeftCount;
    int32_t runStartRightCount;
    task4_line_state_t lineState = TASK4_LINE_STATE_RUNNING;
    bool oledOk;

    gray_serial_init();
    task4_discard_startup_gray_samples();
    encoder_init();
    speed_pid_init();
    line_track_init();
    task4_apply_line_params();
    bluetooth_init();
    oledOk = oled_init();
    taskStartMs = delay_get_ms();
    runStartLeftCount = encoder_get_left_count();
    runStartRightCount = encoder_get_right_count();

    while (1) {
        bluetooth_process();

        nowMs = delay_get_ms();
        grayRaw = gray_serial_read();
        activeCount = task4_count_active_gray_sensors(grayRaw);

        if (lineState == TASK4_LINE_STATE_RUNNING) {
            line_track_update_with_raw_hold_on_lost(grayRaw);
            traveledDistanceMm = task4_get_traveled_distance_mm(
                runStartLeftCount, runStartRightCount);
            stopDetectEnabled =
                (traveledDistanceMm >= TASK4_STOP_DETECT_ENABLE_DISTANCE_MM);

            if (stopDetectEnabled && task4_stop_marker_detected(grayRaw)) {
                lineState = TASK4_LINE_STATE_STOPPED;
                speed_pid_stop();
                elapsedMs = nowMs - taskStartMs;
                timerStopped = true;
            }
        } else {
            speed_pid_stop();
        }

        speed_pid_control_update();
        if (!timerStopped) {
            elapsedMs = nowMs - taskStartMs;
        }
        task4_oled_update(oledOk, grayRaw, activeCount,
                          lineState, stopDetectEnabled,
                          traveledDistanceMm, elapsedMs);
        delay_ms(SPEED_PID_CONTROL_PERIOD_MS);
    }
}
