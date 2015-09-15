/**
 * @file
 *
 * @author  Michael Zapf <michael.zapf@fau.de>
 * @date    2015-09-15
 */


#ifdef PERIPH_EN_BUZZER

#include <stdint.h>
#include "buzzer.h"

#include "board.h"
#include "cpu.h"
#include "periph/gpio.h"
#include "xtimer.h"

void buzzer_init(void) {
    gpio_init(GPIO_BUZZER_PIN, GPIO_DIR_OUT, GPIO_NOPULL);
}

void buzzer(uint16_t ms, uint8_t times) {
    buzzer_multiple(ms, ms, times);
}

void buzzer_multiple(uint16_t ms_on, uint16_t ms_off, uint8_t times) {
    for (uint8_t i = 0; i < times; ++i) {
        GPIO_BUZZER_ON();
        xtimer_usleep(ms_on * 1000);
        GPIO_BUZZER_OFF();

        if (i < times - 1) {
            xtimer_usleep(ms_off * 1000);
        }
    }
}

#endif // PERIPH_EN_BUZZER
