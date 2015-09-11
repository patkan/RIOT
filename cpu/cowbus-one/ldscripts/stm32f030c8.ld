/*
 * Copyright (C) 2015 cowbus project team
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup      cpu_stm32f0
 * @{
 *
 * @file
 * @brief           Memory definitions for the STM32F030C8
 *
 * @author          Michael Zapf <michael.zapf@fau.de>
 *
 * @}
 */

MEMORY
{
    rom (rx)    : ORIGIN = 0x08000000, LENGTH = 64K
    ram (rwx)   : ORIGIN = 0x20000000, LENGTH = 8K
}

INCLUDE cortexm_base.ld
