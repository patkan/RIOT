/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    board_stm32f0discovery STM32F0Discovery
 * @ingroup     boards
 * @brief       Support for the STM32F0Discovery board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the STM32F0Discovery evaluation board.
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef __BOARD_H
#define __BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name The nominal CPU core clock in this board
 */
#define F_CPU               (48000000UL)

/**
 * @name Assign the peripheral timer to be used as hardware timer
 */
#define HW_TIMER            TIMER_0

/**
 * @name Assign the UART interface to be used for stdio
 */
#define STDIO               UART_0
#define STDIO_BAUDRATE      (115200U)
#define STDIO_RX_BUFSIZE    (64U)

/**
 * @name LED pin definitions
 * @{
 */
#define LED_PORT            GPIOA
#define LD0_PIN             (1 << 0)
#define LD1_PIN             (1 << 1)
#define LD2_PIN             (1 << 2)
#define LD3_PIN             (1 << 3)
/** @} */

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LD0_ON              (LED_PORT->BSRRL = LD0_PIN)
#define LD0_OFF             (LED_PORT->BSRRH = LD0_PIN)
#define LD0_TOGGLE          (LED_PORT->ODR ^= LD0_PIN)
#define LD1_ON              (LED_PORT->BSRRL = LD1_PIN)
#define LD1_OFF             (LED_PORT->BSRRH = LD1_PIN)
#define LD1_TOGGLE          (LED_PORT->ODR ^= LD1_PIN)
#define LD2_ON              (LED_PORT->BSRRL = LD2_PIN)
#define LD2_OFF             (LED_PORT->BSRRH = LD2_PIN)
#define LD2_TOGGLE          (LED_PORT->ODR ^= LD2_PIN)
#define LD3_ON              (LED_PORT->BSRRL = LD3_PIN)
#define LD3_OFF             (LED_PORT->BSRRH = LD3_PIN)
#define LD3_TOGGLE          (LED_PORT->ODR ^= LD3_PIN)

/* for compatibility to other boards */
#define LED_GREEN_ON        LD3_ON
#define LED_GREEN_OFF       LD3_OFF
#define LED_GREEN_TOGGLE    LD3_TOGGLE
#define LED_RED_ON          LD0_ON
#define LED_RED_OFF         LD0_OFF
#define LED_RED_TOGGLE      LD0_TOGGLE
/** @} */

typedef uint8_t radio_packet_length_t;
#define RX_BUF_SIZE  (10)
#define TRANSCEIVER_BUFFER_SIZE (10)

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /** __BOARD_H */
/** @} */
