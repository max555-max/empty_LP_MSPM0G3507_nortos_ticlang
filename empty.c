/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "delay.h"
#include "encoder.h"
#include "motor.h"
#include "oled.h"
#include "task2.h"
#include "task3.h"
#include "task4.h"
#include "task5.h"
#include "uart_cmd.h"

#define TASK_MENU_FIRST_ID       (2U)
#define TASK_MENU_LAST_ID        (6U)
#define TASK_MENU_DEBOUNCE_MS    (40U)
#define TASK_MENU_POLL_MS        (5U)

typedef struct {
    bool rawPressed;
    bool stablePressed;
    uint32_t rawChangedMs;
} task_menu_button_t;

static bool task_menu_next_pressed(void)
{
    return (DL_GPIO_readPins(TASK_BUTTON_NEXT_PORT,
                             TASK_BUTTON_NEXT_PIN) == 0U);
}

static bool task_menu_start_pressed(void)
{
    return (DL_GPIO_readPins(TASK_BUTTON_START_PORT,
                             TASK_BUTTON_START_PIN) == 0U);
}

static void task_menu_uart1_send_boot_marker(void)
{
    static const char text[] = "MENU READY\r\n";
    uint8_t index = 0U;

    while ((index < (sizeof(text) - 1U)) &&
           !DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
        DL_UART_Main_transmitData(UART_1_INST, (uint8_t)text[index]);
        index++;
    }
}

static void task_menu_button_init(task_menu_button_t *button,
                                  bool pressed,
                                  uint32_t nowMs)
{
    button->rawPressed = pressed;
    button->stablePressed = pressed;
    button->rawChangedMs = nowMs;
}

static bool task_menu_button_press_event(task_menu_button_t *button,
                                         bool pressed,
                                         uint32_t nowMs)
{
    if (pressed != button->rawPressed) {
        button->rawPressed = pressed;
        button->rawChangedMs = nowMs;
    }

    if ((button->rawPressed != button->stablePressed) &&
        ((uint32_t)(nowMs - button->rawChangedMs) >=
         TASK_MENU_DEBOUNCE_MS)) {
        button->stablePressed = button->rawPressed;
        return button->stablePressed;
    }

    return false;
}

static void task_menu_show(bool oledOk, uint8_t selectedTask)
{
    if (!oledOk) {
        return;
    }

    oled_clear_line(0U);
    oled_print_string("Task Select");
    oled_clear_line(1U);
    oled_print_string("Selected: T");
    oled_print_int(selectedTask);
    oled_clear_line(2U);
    oled_print_string("PB8: Next");
    oled_clear_line(3U);
    oled_print_string("PA7: Start");
    oled_clear_line(4U);
    if (selectedTask == 6U) {
        oled_print_string("Task6: TBD");
    } else {
        oled_print_string("Ready");
    }
}

static void task_menu_start(uint8_t selectedTask)
{
    switch (selectedTask) {
    case 2U:
        task2_run();
        break;
    case 3U:
        (void)task3_run();
        break;
    case 4U:
        task4_run();
        break;
    case 5U:
        task5_run();
        break;
    case 6U:
    default:
        /* Task6 has not been implemented yet; leave all actuators stopped. */
        break;
    }
}

static void task_menu_run(void)
{
    uint8_t selectedTask = TASK_MENU_FIRST_ID;
    uint32_t nowMs = delay_get_ms();
    bool oledOk = oled_init();
    task_menu_button_t nextButton;
    task_menu_button_t startButton;

    /* Finish the OLED reset/display sequence before enabling UART0 RX IRQ. */
    uart_cmd_init();
    task_menu_uart1_send_boot_marker();

    /* Before a task is selected, keep the two drive motors at zero PWM. */
    motor_set_pwm(0, 0);
    task_menu_button_init(&nextButton, task_menu_next_pressed(), nowMs);
    task_menu_button_init(&startButton, task_menu_start_pressed(), nowMs);
    task_menu_show(oledOk, selectedTask);

    while (1) {
        /* Keep UART0 vision frames parsed even before a task is started. */
        uart_cmd_process();
        nowMs = delay_get_ms();

        if (task_menu_button_press_event(&nextButton,
                                         task_menu_next_pressed(), nowMs)) {
            selectedTask++;
            if (selectedTask > TASK_MENU_LAST_ID) {
                selectedTask = TASK_MENU_FIRST_ID;
            }
            task_menu_show(oledOk, selectedTask);
        }

        if (task_menu_button_press_event(&startButton,
                                         task_menu_start_pressed(), nowMs)) {
            task_menu_start(selectedTask);
            task_menu_show(oledOk, selectedTask);
        }

        delay_ms(TASK_MENU_POLL_MS);
    }
}

/* Minimal OLED diagnostic: initialize once and write one fixed line. */
static void oled_direct_test_run(void)
{
    (void)oled_init();
    oled_set_cursor(0U, 0U);
    oled_print_string("OLED OK");

    while (1) {
    }
}

int main(void)
{
    task3_exit_t nextTask;

    SYSCFG_DL_init();
    nextTask = task3_run();

    if (nextTask == TASK3_EXIT_TO_TASK2) {
        task2_run();
    } else {
        task4_run();
    }

    while (1) {
    }
}

void SysTick_Handler(void)
{
    delay_tick();
    encoder_tick_1ms();
}
