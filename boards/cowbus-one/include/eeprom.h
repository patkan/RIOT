/**
 * @defgroup    board_cowbus cowbus
 * @ingroup     boards
 * @brief       Support for the cowbus board
 * @{
 *
 * @file
 * @brief
 *
 * @author      Michael Zapf <michael.zapf@fau.de>
 */

#ifndef __EEPROM_H
#define __EEPROM_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif


void eeprom_init(void);
char eeprom_get_status(void);



#ifdef __cplusplus
}
#endif

#endif /** __EEPROM_H */
/** @} */

