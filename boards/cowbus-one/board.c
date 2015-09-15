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

#include "xtimer.h"

#include "led.h"
#include "switch.h"
//#include "eeprom.h"
// TODO move generic eeprom functions to board and leave cowconfig stuff in app
#include "temp.h"
#include "buzzer.h"


void board_init(void)
{
    /* initialize the CPU */
    cpu_init();

    /* initialize timer */
    xtimer_init();

    /* initialize the selected periphery */
#ifdef PERIPH_EN_BUZZER
    buzzer_init();
#endif
//#ifdef PERIPH_EN_EEPROM
//    eeprom_init();
//#endif
#ifdef PERIPH_EN_TEMP
    temp_init();
#endif
#ifdef PERIPH_EN_LED
    led_init();
#endif
#ifdef PERIPH_EN_SWITCH
    switch_init();
#endif
}

