#ifndef __GLOBAL_H
#define __GLOBAL_H

/* Global inclusions -------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Global definitions ------------------------------------------------------*/
#define MCU_CLOCK_FREQ 72000000
#define TIM1_CLOCK_FREQ MCU_CLOCK_FREQ
#define DEFAULT_PWM_FREQ 23590
#define TIM1_RELOAD_VALUE ((TIM1_CLOCK_FREQ/DEFAULT_PWM_FREQ)-1)
#define TIM1_PULSE_VALUE (TIM1_RELOAD_VALUE / 2)
#define TIM4_CLOCK_FREQ MCU_CLOCK_FREQ
#define TIM4_PRESCALER 71
#define DEFAULT_LINE_FREQ 50
#define TIM4_RELOAD_VALUE (((TIM4_CLOCK_FREQ/(TIM4_PRESCALER+1)) / DEFAULT_LINE_FREQ)-1)
#define TIM3_CLOCK_FREQ MCU_CLOCK_FREQ
#define TIM3_PRESCALER 71
#define TIM2_CLOCK_FREQ MCU_CLOCK_FREQ
#define TIM2_PRESCALER 71
#define TIM2_RELOAD_VALUE (((TIM2_CLOCK_FREQ/(TIM2_PRESCALER+1)) / DEFAULT_LINE_FREQ)-1)
#define LED_GREEN_Pin GPIO_PIN_13
#define LED_GREEN_GPIO_Port GPIOC
#define ZERO_CROSSING_RISING_DETECT_Pin GPIO_PIN_0
#define ZERO_CROSSING_RISING_DETECT_GPIO_Port GPIOB
#define ZERO_CROSSING_FALLING_DETECT_Pin GPIO_PIN_1
#define ZERO_CROSSING_FALLING_DETECT_GPIO_Port GPIOB
#define OVERCURRENT_SENS_Pin GPIO_PIN_12
#define OVERCURRENT_SENS_GPIO_Port GPIOB
#define HBRIDGE_A_LSIDE_Pin GPIO_PIN_13
#define HBRIDGE_A_LSIDE_GPIO_Port GPIOB
#define HBRIDGE_B_HSIDE_Pin GPIO_PIN_14
#define HBRIDGE_B_HSIDE_GPIO_Port GPIOB
#define HBRIDGE_A_HSIDE_Pin GPIO_PIN_8
#define HBRIDGE_A_HSIDE_GPIO_Port GPIOA
#define HBRIDGE_B_LSIDE_Pin GPIO_PIN_9
#define HBRIDGE_B_LSIDE_GPIO_Port GPIOA
#define THYRISTOR1_DRIVE_Pin GPIO_PIN_8
#define THYRISTOR1_DRIVE_GPIO_Port GPIOB
#define THYRISTOR0_DRIVE_Pin GPIO_PIN_9
#define THYRISTOR0_DRIVE_GPIO_Port GPIOB

/* Global types ------------------------------------------------------------*/
enum
{
    FALSE = 0,
    TRUE = !FALSE
};
typedef uint8_t Bool_t;

/* Global constants --------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f

#endif

#endif