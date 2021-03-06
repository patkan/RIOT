/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32f0
 * @{
 *
 * @file
 * @brief       Implementation of the CPU initialization
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @}
 */

#include "cpu.h"
#include "periph_conf.h"


static void clock_init(void);

/**
 * @brief Initialize the CPU, set IRQ priorities
 */
void cpu_init(void)
{
    /* initialize the Cortex-M core */
    cortexm_init();
    /* initialize the clock system */
    clock_init();
}

/**
 * @brief Configure the controllers clock system
 *
 * The clock initialization make the following assumptions:
 * - the external HSE clock from an external oscillator is used as base clock
 * - the internal PLL circuit is used for clock refinement
 *
 * Use the following formulas to calculate the needed values:
 *
 * SYSCLK = ((HSE_VALUE / CLOCK_PLL_M) * CLOCK_PLL_N) / CLOCK_PLL_P
 * USB, SDIO and RNG Clock =  ((HSE_VALUE / CLOCK_PLL_M) * CLOCK_PLL_N) / CLOCK_PLL_Q
 *
 * The actual used values are specified in the board's `periph_conf.h` file.
 *
 * NOTE: currently there is not timeout for initialization of PLL and other locks
 *       -> when wrong values are chosen, the initialization could stall
 */
static void clock_init(void)
{
    // enable clock for GPIO_F-block
	//(RCC->AHBENR |= RCC_AHBENR_GPIOFEN);

    /* configure the HSE clock */

    /* enable the HSI clock */
    RCC->CR |= RCC_CR_HSION;

    ///* disable the HSI clock */
    //RCC->CR &= (~RCC_CR_HSION);
    //while ((RCC->CR & RCC_CR_HSIRDY));

    /* test external low speed crystal */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;          //enable clock for power control
    PWR->CR |= PWR_CR_DBP;                      //disable write protection of RTC register domain
    RCC->BDCR |= RCC_BDCR_LSEON;                //enable LSE
    //RCC->BDCR &= ~(RCC_BDCR_LSEBYP);
    while (!(RCC->BDCR & RCC_BDCR_LSERDY));     //wait for LSE to become stable

    /* reset clock configuration register */
    RCC->CFGR = 0;
    RCC->CFGR2 = 0;

    /* disable HSE, CSS and PLL */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON | RCC_CR_PLLON);

    /* disable all clock interrupts */
    RCC->CIR = 0;

    /* disable clock output */
    //RCC->CFGR &= ~(RCC_CFGR_MCO);

#ifdef USE_HSE
    /* enable the HSE clock */
    RCC->CR |= RCC_CR_HSEON;
    RCC->BDCR &= ~(RCC_BDCR_LSEBYP);    // disable HSE bypass

    //for (volatile unsigned int i = 0; i < 10000; ++i) { asm volatile ("nop"); }

    /* wait for HSE to be ready */
    while (!(RCC->CR & RCC_CR_HSERDY));
#else
    /* enable the HSI clock */
    RCC->CR |= RCC_CR_HSION;

    /* wait for HSI to be ready */
    while (!(RCC->CR & RCC_CR_HSIRDY));
#endif

    /* setup the peripheral bus prescalers */

    /* set HCLK = SYSCLK, so no clock division here */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    /* set PCLK = HCLK, so its not divided */
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

    /* configure the PLL */

    /* reset PLL configuration bits */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL);

    /* set PLL configuration */
#ifdef USE_HSE
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1 |
                 (((CLOCK_PLL_MUL - 2) & 0xf) << 18);
#else
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2 |
                 (((CLOCK_PLL_MUL - 2) & 0xf) << 18);
#endif

    //RCC->CFGR &= ~(0xF << 18);             // set PLLMUL to 0 (factor 2)
    //RCC->CFGR |= 0x2 << 18;                // set PLLMUL to 2 (factor 4)
    //RCC->CFGR |= 0x3 << 18;              // set PLLMUL to 3 (factor 8)

    ///* enable PLL again */
    RCC->CR |= RCC_CR_PLLON;
    ///* wait until PLL is stable */
    while(!(RCC->CR & RCC_CR_PLLRDY));

    /* configure flash latency */

    /* enable pre-fetch buffer and set flash latency to 1 cycle*/
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    /* configure the sysclock and the peripheral clocks */

    ///* set sysclock to be driven by the PLL clock */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* wait for sysclock to be stable */
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}
