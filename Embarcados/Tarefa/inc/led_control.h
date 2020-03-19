/**
 * @file led_control.h
 *
 * @brief LED control functions
 */

#ifndef __LED_CONTROL_H__
#define __LED_CONTROL_H__

#include <stdint.h>

/*****************************************
 * Public Function Prototypes
 *****************************************/

/**
 * @brief Initializes ADC configured to 12 bits
 */
void adc_init(void);

/**
 * @brief Initializes PWM
 */
void pwm_init(void);

/**
 * @brief Gives ADC result in 16 bit uint
 *
 * @return ADC reading value
 */
uint16_t get_adc_result(void);

/**
 * @brief sets the brightness of LED using the PWM
 * The parameter is the dead cycle of the PWM
 *
 * @param brightness 16 bit uint
 */
void set_led_brightness(uint16_t brightness);

#endif  // __LED_CONTROL_H__
