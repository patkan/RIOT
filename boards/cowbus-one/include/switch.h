/**
 * @brief   Switches on cowbus protoboard.
 *
 * @author  Michael Zapf <michael.zapf@fau.de>
 * @date    2015-09-13
 * @file
 */


#ifndef SWITCH_H
#define SWITCH_H


/**
 * @brief   Initializes the Switches
 */
void switch_init(void);

/**
 * @brief   Get the state of switch #1.
 * @return  \b true for "pressed", \b false otherwise
 */
bool switch1_get_state(void);

/**
 * @brief   Get the state of switch #2.
 * @return  \b true for "pressed", \b false otherwise
 */
bool switch2_get_state(void);

/**
 * @brief   Get the state of switch #3.
 * @return  \b true for "pressed", \b false otherwise
 */
bool switch3_get_state(void);

/**
 * @brief   Get the state of switch #4.
 * @return  \b true for "pressed", \b false otherwise
 */
bool switch4_get_state(void);


/**
 * @brief   Set the ISR function pointer for switch #1.
 */
void switch1_set_isr(void (*cb)(void));

/**
 * @brief   Set the ISR function pointer for switch #2.
 */
void switch2_set_isr(void (*cb)(void));

/**
 * @brief   Set the ISR function pointer for switch #3.
 */
void switch3_set_isr(void (*cb)(void));

/**
 * @brief   Set the ISR function pointer for switch #4.
 */
void switch4_set_isr(void (*cb)(void));

#endif // SWITCH_H

