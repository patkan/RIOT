/**
 * @brief   Functionality for piezo buzzer on PERIPH subboard
 *
 * @author  Michael Zapf <michael.zapf@fau.de>
 * @date    2015-09-15
 * @file
 */


#ifndef BUZZER_H
#define BUZZER_H


typedef enum buzzer_note_t {
    note_c2 = 0,
    note_d2 = 1,
    note_e2 = 2,
    note_f2 = 3,
    note_g2 = 4,
    note_a2 = 5,
    note_b2 = 6,
    note_c3 = 7
} buzzer_note_t;

/**
 * @brief   Initializes the Buzzer
 */
void buzzer_init(void);

/**
 * @brief   Plays a specific note with the buzzerme then given number of times.
 * @param   note        The note the buzzer shall play.
 * @param   ms          The time the buzzer plays the note in milliseconds.
 * @param   times
 */
void buzzer(buzzer_note_t note, uint16_t ms);



#endif // BUZZER_H
