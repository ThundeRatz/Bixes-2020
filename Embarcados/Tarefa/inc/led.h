/**
 * @file led.h
 *
 * @brief LED control functions
 */

#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>



/*****************************************
 * Public Function Prototypes
 *****************************************/

/**
 * @brief Initializes ADC configured to 12 bits
 */

void ADC_init(void);

/**
 * @brief Initializes PWM
 */

void PWM_init(void);

/**
 * @brief Gives ADC result in 16 bit int
 */

uint16_t get_ADC_result(void);

/**
 * @brief sets the brightness of LED using the PWM
 * The parameter is the dead cycle of the PWM
 *
 * @params brightness 16 bit int
 */

void set_LED_brightness(uint16_t brightness);

#endif  // __LED_H__
