/**
 * @file
 * @author  Michael Zapf <michael.zapf@fau.de>
 * @date    2015-09-13
 */

#ifdef PERIPH_EN_SWITCH

#include "board.h"
#include "cpu.h"
#include "periph/gpio.h"

#include "switch.h"

void (*cb_sw1)(void) = NULL;
void (*cb_sw2)(void) = NULL;
void (*cb_sw3)(void) = NULL;
void (*cb_sw4)(void) = NULL;

void switch1_pressed(void) {
    if (cb_sw1 != NULL) {
        cb_sw1();
    }
}
void switch2_pressed(void) {
    if (cb_sw2 != NULL) {
        cb_sw2();
    }
}
void switch3_pressed(void) {
    if (cb_sw3 != NULL) {
        cb_sw3();
    }
}
void switch4_pressed(void) {
    if (cb_sw4 != NULL) {
        cb_sw4();
    }
}

void switch_init(void) {
    gpio_init_int(GPIO_SWITCH_1, GPIO_PULLDOWN, GPIO_RISING,
            (void *)switch1_pressed, 0);

    gpio_init_int(GPIO_SWITCH_2, GPIO_PULLDOWN, GPIO_RISING,
            (void *)switch2_pressed, 0);

    gpio_init_int(GPIO_SWITCH_3, GPIO_PULLDOWN, GPIO_RISING,
            (void *)switch3_pressed, 0);

    gpio_init_int(GPIO_SWITCH_4, GPIO_PULLDOWN, GPIO_RISING,
            (void *)switch4_pressed, 0);
}

bool switch1_get_state(void) {
    return (gpio_read(GPIO_SWITCH_1) > 0);
}

bool switch2_get_state(void) {
    return (gpio_read(GPIO_SWITCH_2) > 0);
}

bool switch3_get_state(void) {
    return (gpio_read(GPIO_SWITCH_3) > 0);
}

bool switch4_get_state(void) {
    return (gpio_read(GPIO_SWITCH_4) > 0);
}


void switch1_set_isr(void (*cb)(void)) {
    cb_sw1 = cb;
}

void switch2_set_isr(void (*cb)(void)) {
    cb_sw2 = cb;
}

void switch3_set_isr(void (*cb)(void)) {
    cb_sw3 = cb;
}

void switch4_set_isr(void (*cb)(void)) {
    cb_sw4 = cb;
}


#endif // PERIPH_EN_SWITCH
