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
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Hauke Petersen <mail@haukepetersen.de>
 *
 * @}
 */

#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph/gpio.h"
#include "periph_conf.h"

typedef struct {
    gpio_cb_t cb;
    void *arg;
} gpio_state_t;


/**
 * @brief   The STM32F0 has 16 EXTI channels
 */
#define EXTI_NUMOF      (16U)

static gpio_state_t gpio_config[EXTI_NUMOF];

/**
 * @brief   Extract the port base address from the given pin identifier
 */
static inline GPIO_TypeDef *_port(gpio_t pin)
{
    return (GPIO_TypeDef *)(pin & ~(0x0f));
}

/**
 * @brief   Extract the port number form the given identifier
 *
 * The port number is extracted by looking at bits 10, 11, 12, 13 of the base
 * register addresses.
 */
static inline int _port_num(gpio_t pin)
{
    return ((pin >> 10) & 0x0f);
}

/**
 * @brief   Extract the pin number from the last 4 bit of the pin identifier
 */
static inline int _pin_num(gpio_t pin)
{
    return (pin & 0x0f);
}


int gpio_init(gpio_t dev, gpio_dir_t dir, gpio_pp_t pullup)
{
    GPIO_TypeDef *port;
    uint8_t pin;

    port = _port(dev);
    pin = _pin_num(dev);

    //RCC->AHBENR |= (1 << gpio_clock_map[dev]);
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN << _port_num(pin));

    port->PUPDR &= ~(3 << (2 * pin));           /* configure push-pull resistors */
    port->PUPDR |= (pullup << (2 * pin));

    if (dir == GPIO_DIR_OUT) {
        port->MODER &= ~(2 << (2 * pin));           /* set pin to output mode */
        port->MODER |= (1 << (2 * pin));
        port->OTYPER &= ~(1 << pin);                /* set to push-pull configuration */
        port->OSPEEDR |= (3 << (2 * pin));          /* set to high speed */
        port->ODR &= ~(1 << pin);                   /* set pin to low signal */
    }
    else {
        port->MODER &= ~(3 << (2 * pin));           /* configure pin as input */

    }

    return 0; /* all OK */
}

int gpio_init_int(gpio_t dev, gpio_pp_t pullup, gpio_flank_t flank, gpio_cb_t cb, void *arg)
{
    int res;
    uint8_t pin;

    pin = _pin_num(dev);

    /* configure pin as input */
    res = gpio_init(dev, GPIO_DIR_IN, pullup);
    if (res < 0) {
        return res;
    }

    /* set interrupt priority (its the same for all EXTI interrupts) */
    // TODO

    /* enable clock of the SYSCFG module for EXTI configuration */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    /* read pin number, set EXIT channel and enable global interrupt for EXTI channel */
    
    /* configure the active edge(s) */
    switch (flank) {
        case GPIO_RISING:
            EXTI->RTSR |= (1 << pin);
            EXTI->FTSR &= ~(1 << pin);
            break;
        case GPIO_FALLING:
            EXTI->RTSR &= ~(1 << pin);
            EXTI->FTSR |= (1 << pin);
            break;
        case GPIO_BOTH:
            EXTI->RTSR |= (1 << pin);
            EXTI->FTSR |= (1 << pin);
            break;
    }
    /* enable specific pin as exti sources */
    SYSCFG->EXTICR[pin >> 2] &= ~(0xf << ((pin & 0x03) * 4));
    SYSCFG->EXTICR[pin >> 2] |= (_port_num(dev) << ((pin & 0x03) * 4));
    
    // TODO works?
    /* enable global pin interrupt */
    if (pin < 2) {
        NVIC_EnableIRQ(EXTI0_1_IRQn);
    }
    else if (pin < 4) {
        NVIC_EnableIRQ(EXTI2_3_IRQn);
    }
    else {
        NVIC_EnableIRQ(EXTI4_15_IRQn);
    }

    /* set callback */
    gpio_config[pin].cb = cb;
    gpio_config[pin].arg = arg;

    /* configure the event that triggers an interrupt */
    switch (flank) {
        case GPIO_RISING:
            EXTI->RTSR |= (1 << pin);
            EXTI->FTSR &= ~(1 << pin);
            break;
        case GPIO_FALLING:
            EXTI->RTSR &= ~(1 << pin);
            EXTI->FTSR |= (1 << pin);
            break;
        case GPIO_BOTH:
            EXTI->RTSR |= (1 << pin);
            EXTI->FTSR |= (1 << pin);
            break;
    }

    /* clear any pending requests */
    EXTI->PR = (1 << pin);
    /* unmask the pins interrupt channel */
    EXTI->IMR |= (1 << pin);

    return 0;
}

void gpio_init_af(gpio_t pin, gpio_af_t af)
{
    GPIO_TypeDef *port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    /* set pin to AF mode */
    port->MODER &= ~(3 << (2 * pin_num));
    port->MODER |= (2 << (2 * pin_num));
    /* set selected function */
    port->AFR[(pin_num > 7) ? 1 : 0] &= ~(0xf << ((pin_num & 0x07) * 4));
    port->AFR[(pin_num > 7) ? 1 : 0] |= (af << ((pin_num & 0x07) * 4));
}

void gpio_irq_enable(gpio_t dev)
{
    uint8_t pin;

    pin = _pin_num(dev);
    EXTI->IMR |= (1 << pin);
}

void gpio_irq_disable(gpio_t dev)
{
    uint8_t pin;

    pin = _pin_num(dev);
    EXTI->IMR &= ~(1 << pin);
}

int gpio_read(gpio_t dev)
{
    GPIO_TypeDef *port;
    uint8_t pin;

    port = _port(dev);
    pin = _pin_num(dev);

    if (port->MODER & (1 << (pin * 2))) {       /* if configured as output */
        return port->ODR & (1 << pin);          /* read output data register */
    } else {
        return port->IDR & (1 << pin);          /* else read input data register */
    }
}

void gpio_set(gpio_t dev)
{
    GPIO_TypeDef *port;
    uint8_t pin;

    port = _port(dev);
    pin = _pin_num(dev);

    port->ODR |= (1 << pin);
}

void gpio_clear(gpio_t dev)
{
    GPIO_TypeDef *port;
    uint8_t pin;

    port = _port(dev);
    pin = _pin_num(dev);

    port->ODR &= ~(1 << pin);
}

void gpio_toggle(gpio_t dev)
{
    if (gpio_read(dev)) {
        gpio_clear(dev);
    } else {
        gpio_set(dev);
    }
}

void gpio_write(gpio_t dev, int value)
{
    if (value) {
        gpio_set(dev);
    } else {
        gpio_clear(dev);
    }
}

void isr_exti(void)
{
    for (int i = 0; i < EXTI_NUMOF; i++) {
        if (EXTI->PR & (1 << i)) {
            EXTI->PR |= (1 << i);               /* clear by writing a 1 */
            gpio_config[i].cb(gpio_config[i].arg);
        }
    }
    if (sched_context_switch_request) {
        thread_yield();
    }
}

