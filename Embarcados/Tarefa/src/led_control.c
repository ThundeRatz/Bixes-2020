/**
 * @file led_control.c
 *
 * @brief LED control functions
 */

#include <stdbool.h>
#include <stdint.h>

#include "led_control.h"
#include "utils.h"

#include "adc.h"
#include "gpio.h"
#include "tim.h"


/*****************************************
 * Public Function Body Definitions
 *****************************************/

void adc_init(void) {
    MX_ADC1_Init();
    HAL_ADC_Start(&hadc1);
}

void pwm_init(void) {
    /**
     * Implemente aqui a função de init para a geração de PWM_init
     * @see https://github.com/ThundeRatz/STM32Guide#pwm
     */
}

uint16_t get_adc_result(void) {
    return HAL_ADC_GetValue(&hadc1);
}

void set_led_brightness(uint16_t brightness) {
    /**
    * Aqui você deve configurar o brilho do LED usando a PWM
    * @see https://github.com/ThundeRatz/STM32Guide#pwm
    */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        /**
        * Aqui você deve implementar um código que faça a lógica do LED mudar quando o botão azul é apertado
        */
    }
}
