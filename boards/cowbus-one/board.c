/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_stm32f0discovery
 * @{
 *
 * @file
 * @brief       Board specific implementations for the STM32F0Discovery evaluation board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Michael Zapf <michael.zapf@fau.de>
 *
 * @}
 */

#include "board.h"
#include "periph/uart.h"


void board_init(void)
{
    /* initialize the boards LEDs */
    //leds_init();

    /* initialize the CPU */
    cpu_init();
}

