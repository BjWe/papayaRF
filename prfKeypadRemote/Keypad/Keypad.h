/*
 * Keypad.h
 *
 * Created: 14.01.2013 23:42:16
 *  Author: Bjoern
 */ 


#ifndef KEYPAD_H_
#define KEYPAD_H_

#include <avr/io.h>
#include "../Global.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern void keypad_init(void);
extern void keypad_prepare_for_standby(void);
extern int8_t keypad_get_row(void);
extern uint8_t keypad_get_key(void);
extern uint8_t keypad_get_key_wait(uint8_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* KEYPAD_H_ */