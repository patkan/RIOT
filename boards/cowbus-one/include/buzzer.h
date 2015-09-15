/**
 * @brief   Functionality for piezo buzzer on PERIPH subboard
 *
 * @author  Michael Zapf <michael.zapf@fau.de>
 * @date    2015-09-15
 * @file
 */


#ifndef BUZZER_H
#define BUZZER_H


/**
 * @brief   Initializes the Buzzer
 */
void buzzer_init(void);

/**
 * @brief   Switches the buzzer on for a given time then given number of times.
 * @param   ms          The time the buzzer is on (same as time for off).
 * @param   times
 */
void buzzer(uint16_t ms, uint8_t times);

/**
 * @brief   Switches the buzzer on and off the given number of times.
 * @param   ms_on       Time in ms the buzzer should beep
 * @param   ms_off      Time in ms the buzzer should be off between beeps
 * @param   times       How often the buzzer should beep
 */
void buzzer_multiple(uint16_t ms_on, uint16_t ms_off, uint8_t times);



#endif // BUZZER_H
