/**
 * @file led.c
 *
 * @brief LED control functions
 */

#include <stdbool.h>
#include <stdint.h>

#include "led.h"

#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "utils.h"

/*****************************************
 * Private Macro Definitions
 *****************************************/

/**
 * @brief Constrains v between x and y, returning at least x and at most y.
 */
#define constrain(v, x, y) ((v) < (x) ? (x) : ((v) > (y) ? (y) : (v)))


/*****************************************
 * Public Function Body Definitions
 *****************************************/

void ADC_init(void) {
    MX_ADC1_Init();
    HAL_ADC_Start(&hadc1);
}

void PWM_init(void) {
    MX_TIM2_Init();

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

uint16_t get_ADC_result(void) {
    return HAL_ADC_GetValue(&hadc1);
}

void set_LED_brightness(uint16_t brightness) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, brightness);

    /**
    * Aqui você deve configurar o brilho do LED usando a PWM
    * Vale avisar que a PWM está configurada com um counter period de 2000ms
    */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {

        /**
        * Aqui você deve implementar um código que faça a lógica do LED mudar quando o botão azul é apertado
        */

    }
}
