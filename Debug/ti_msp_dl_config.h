/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM */
#define PWM_INST                                                           TIMG6
#define PWM_INST_IRQHandler                                     TIMG6_IRQHandler
#define PWM_INST_INT_IRQN                                       (TIMG6_INT_IRQn)
#define PWM_INST_CLK_FREQ                                               80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_C0_PORT                                                   GPIOB
#define GPIO_PWM_C0_PIN                                            DL_GPIO_PIN_2
#define GPIO_PWM_C0_IOMUX                                        (IOMUX_PINCM15)
#define GPIO_PWM_C0_IOMUX_FUNC                       IOMUX_PINCM15_PF_TIMG6_CCP0
#define GPIO_PWM_C0_IDX                                      DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_C1_PORT                                                   GPIOB
#define GPIO_PWM_C1_PIN                                            DL_GPIO_PIN_3
#define GPIO_PWM_C1_IOMUX                                        (IOMUX_PINCM16)
#define GPIO_PWM_C1_IOMUX_FUNC                       IOMUX_PINCM16_PF_TIMG6_CCP1
#define GPIO_PWM_C1_IDX                                      DL_TIMER_CC_1_INDEX




/* Defines for I2C_ICM42688 */
#define I2C_ICM42688_INST                                                   I2C0
#define I2C_ICM42688_INST_IRQHandler                             I2C0_IRQHandler
#define I2C_ICM42688_INST_INT_IRQN                                 I2C0_INT_IRQn
#define I2C_ICM42688_BUS_SPEED_HZ                                         100000
#define GPIO_I2C_ICM42688_SDA_PORT                                         GPIOA
#define GPIO_I2C_ICM42688_SDA_PIN                                  DL_GPIO_PIN_0
#define GPIO_I2C_ICM42688_IOMUX_SDA                               (IOMUX_PINCM1)
#define GPIO_I2C_ICM42688_IOMUX_SDA_FUNC                IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_ICM42688_SCL_PORT                                         GPIOA
#define GPIO_I2C_ICM42688_SCL_PIN                                  DL_GPIO_PIN_1
#define GPIO_I2C_ICM42688_IOMUX_SCL                               (IOMUX_PINCM2)
#define GPIO_I2C_ICM42688_IOMUX_SCL_FUNC                IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART0 */
#define UART0_INST                                                         UART0
#define UART0_INST_FREQUENCY                                            40000000
#define UART0_INST_IRQHandler                                   UART0_IRQHandler
#define UART0_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_UART0_RX_PORT                                                 GPIOA
#define GPIO_UART0_TX_PORT                                                 GPIOA
#define GPIO_UART0_RX_PIN                                         DL_GPIO_PIN_11
#define GPIO_UART0_TX_PIN                                         DL_GPIO_PIN_10
#define GPIO_UART0_IOMUX_RX                                      (IOMUX_PINCM22)
#define GPIO_UART0_IOMUX_TX                                      (IOMUX_PINCM21)
#define GPIO_UART0_IOMUX_RX_FUNC                       IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART0_IOMUX_TX_FUNC                       IOMUX_PINCM21_PF_UART0_TX
#define UART0_BAUD_RATE                                                 (115200)
#define UART0_IBRD_40_MHZ_115200_BAUD                                       (21)
#define UART0_FBRD_40_MHZ_115200_BAUD                                       (45)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_1_FBRD_40_MHZ_9600_BAUD                                        (27)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for LED1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define LED_LED1_PIN                                             (DL_GPIO_PIN_9)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM26)
/* Port definition for Pin Group OLED_RST */
#define OLED_RST_PORT                                                    (GPIOB)

/* Defines for PIN_RST: GPIOB.14 with pinCMx 31 on package pin 2 */
#define OLED_RST_PIN_RST_PIN                                    (DL_GPIO_PIN_14)
#define OLED_RST_PIN_RST_IOMUX                                   (IOMUX_PINCM31)
/* Port definition for Pin Group OLED_DC */
#define OLED_DC_PORT                                                     (GPIOB)

/* Defines for PIN_DC: GPIOB.15 with pinCMx 32 on package pin 3 */
#define OLED_DC_PIN_DC_PIN                                      (DL_GPIO_PIN_15)
#define OLED_DC_PIN_DC_IOMUX                                     (IOMUX_PINCM32)
/* Port definition for Pin Group OLED_SCL */
#define OLED_SCL_PORT                                                    (GPIOA)

/* Defines for PIN_SCL: GPIOA.28 with pinCMx 3 on package pin 35 */
#define OLED_SCL_PIN_SCL_PIN                                    (DL_GPIO_PIN_28)
#define OLED_SCL_PIN_SCL_IOMUX                                    (IOMUX_PINCM3)
/* Port definition for Pin Group OLED_SDA */
#define OLED_SDA_PORT                                                    (GPIOA)

/* Defines for PIN_SDA: GPIOA.31 with pinCMx 6 on package pin 39 */
#define OLED_SDA_PIN_SDA_PIN                                    (DL_GPIO_PIN_31)
#define OLED_SDA_PIN_SDA_IOMUX                                    (IOMUX_PINCM6)
/* Port definition for Pin Group AIN */
#define AIN_PORT                                                         (GPIOA)

/* Defines for AIN2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_14)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM36)
/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_13)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM35)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOA)

/* Defines for BIN2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define BIN_BIN2_PIN                                            (DL_GPIO_PIN_16)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM38)
/* Defines for BIN1: GPIOA.17 with pinCMx 39 on package pin 10 */
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_17)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM39)
/* Defines for E1B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODER_E1B_PORT                                                 (GPIOA)
// pins affected by this interrupt request:["E1B","E1A"]
#define ENCODER_GPIOA_INT_IRQN                                  (GPIOA_INT_IRQn)
#define ENCODER_GPIOA_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODER_E1B_IIDX                                    (DL_GPIO_IIDX_DIO25)
#define ENCODER_E1B_PIN                                         (DL_GPIO_PIN_25)
#define ENCODER_E1B_IOMUX                                        (IOMUX_PINCM55)
/* Defines for E1A: GPIOA.26 with pinCMx 59 on package pin 30 */
#define ENCODER_E1A_PORT                                                 (GPIOA)
#define ENCODER_E1A_IIDX                                    (DL_GPIO_IIDX_DIO26)
#define ENCODER_E1A_PIN                                         (DL_GPIO_PIN_26)
#define ENCODER_E1A_IOMUX                                        (IOMUX_PINCM59)
/* Defines for E2B: GPIOB.20 with pinCMx 48 on package pin 19 */
#define ENCODER_E2B_PORT                                                 (GPIOB)
// pins affected by this interrupt request:["E2B","E2A"]
#define ENCODER_GPIOB_INT_IRQN                                  (GPIOB_INT_IRQn)
#define ENCODER_GPIOB_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODER_E2B_IIDX                                    (DL_GPIO_IIDX_DIO20)
#define ENCODER_E2B_PIN                                         (DL_GPIO_PIN_20)
#define ENCODER_E2B_IOMUX                                        (IOMUX_PINCM48)
/* Defines for E2A: GPIOB.24 with pinCMx 52 on package pin 23 */
#define ENCODER_E2A_PORT                                                 (GPIOB)
#define ENCODER_E2A_IIDX                                    (DL_GPIO_IIDX_DIO24)
#define ENCODER_E2A_PIN                                         (DL_GPIO_PIN_24)
#define ENCODER_E2A_IOMUX                                        (IOMUX_PINCM52)
/* Defines for DAT: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GRAY_SERIAL_DAT_PORT                                             (GPIOA)
#define GRAY_SERIAL_DAT_PIN                                     (DL_GPIO_PIN_22)
#define GRAY_SERIAL_DAT_IOMUX                                    (IOMUX_PINCM47)
/* Defines for CLK: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GRAY_SERIAL_CLK_PORT                                             (GPIOB)
#define GRAY_SERIAL_CLK_PIN                                     (DL_GPIO_PIN_17)
#define GRAY_SERIAL_CLK_IOMUX                                    (IOMUX_PINCM43)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_init(void);
void SYSCFG_DL_I2C_ICM42688_init(void);
void SYSCFG_DL_UART0_init(void);
void SYSCFG_DL_UART_1_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
