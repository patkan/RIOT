/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_stm32f0discovery STM32F0Discovery
 * @ingroup     boards
 * @brief       Support for the STM32F0Discovery board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the STM32F0Discovery evaluation board.
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Michael Zapf <michael.zapf@fau.de>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name The nominal CPU core clock in this board
 */
#define F_CPU               (16000000UL)

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


// TODO messen? und anpassen
/**
 * @name xtimer tuning values
 * @{
 */
#define XTIMER_BACKOFF      25
#define XTIMER_USLEEP_UNTIL_OVERHEAD 20
#define XTIMER_OVERHEAD     5
//#define XTIMER_MASK         0xFFFF0000
//#define XTIMER_SHOOT_EARLY  2
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
