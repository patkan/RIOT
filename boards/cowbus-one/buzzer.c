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

int pause_times[] = {1908, 1701, 1515, 1433, 1275, 1136, 1012, 956};


void buzzer_init(void) {
    gpio_init(GPIO_BUZZER_PIN, GPIO_DIR_OUT, GPIO_NOPULL);
}

void buzzer(buzzer_note_t note, uint16_t ms){
    int times = (1000*ms) / (pause_times[note] * 2);

    for (int i = 0; i < times; ++i) {
        GPIO_BUZZER_ON();
        xtimer_usleep(pause_times[note]);
        GPIO_BUZZER_OFF();
        xtimer_usleep(pause_times[note]);
    }

    //buzzer_multiple(ms, ms, times);
}

#endif // PERIPH_EN_BUZZER
