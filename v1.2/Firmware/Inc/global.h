#ifndef __GLOBAL_H
#define __GLOBAL_H

/* Global inclusions -------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Global definitions ------------------------------------------------------*/
#define MCU_CLOCK_FREQ 72000000
#define HRTIM_CLOCK_FREQ 1152e6
#define MAX_PWM_FREQ 24e3
#define MIN_PWM_FREQ 19e3
#define MAX_PWM_DUTY 0.5f
#define MIN_PWM_DUTY 0.05f
#define SEARCHING_NORMALIZED_POWER 0.2
#define PWM_CYCLES_PER_CONTROL_CYCLE 32
#define SAMPLES_PER_CYCLE 12
#define TIM3_CLOCK_FREQ MCU_CLOCK_FREQ
#define TIM3_PRESCALER 71
#define DEFAULT_LINE_FREQ 50
#define ISENS__Pin GPIO_PIN_0
#define ISENS__GPIO_Port GPIOA
#define ISENS_A1_Pin GPIO_PIN_1
#define ISENS_A1_GPIO_Port GPIOA
#define VSENS__Pin GPIO_PIN_2
#define VSENS__GPIO_Port GPIOA
#define VSENS_A3_Pin GPIO_PIN_3
#define VSENS_A3_GPIO_Port GPIOA
#define TRIGIN_RISING_Pin GPIO_PIN_4
#define TRIGIN_RISING_GPIO_Port GPIOA
#define TRIGIN_RISING_EXTI_IRQn EXTI4_IRQn
#define TRIGIN_FALLING_Pin GPIO_PIN_5
#define TRIGIN_FALLING_GPIO_Port GPIOA
#define TRIGIN_FALLING_EXTI_IRQn EXTI9_5_IRQn
#define ZC_RISING_DETECT_Pin GPIO_PIN_0
#define ZC_RISING_DETECT_GPIO_Port GPIOB
#define ZC_FALLING_DETECT_Pin GPIO_PIN_1
#define ZC_FALLING_DETECT_GPIO_Port GPIOB
#define RS485_DEnRE_Pin GPIO_PIN_2
#define RS485_DEnRE_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_12
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_13
#define LED_GREEN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOB
#define HB_HSIDE_Pin GPIO_PIN_8
#define HB_HSIDE_GPIO_Port GPIOA
#define HB_LSIDE_Pin GPIO_PIN_9
#define HB_LSIDE_GPIO_Port GPIOA
#define MISC0_Pin GPIO_PIN_3
#define MISC0_GPIO_Port GPIOB
#define MISC1_Pin GPIO_PIN_4
#define MISC1_GPIO_Port GPIOB
#define MISC2_Pin GPIO_PIN_5
#define MISC2_GPIO_Port GPIOB
#define HMI_SELF_TX_Pin GPIO_PIN_6
#define HMI_SELF_TX_GPIO_Port GPIOB
#define HMI_SELF_RX_Pin GPIO_PIN_7
#define HMI_SELF_RX_GPIO_Port GPIOB
#define THYRISTOR0_DRIVE_Pin GPIO_PIN_8
#define THYRISTOR0_DRIVE_GPIO_Port GPIOB
#define THYRISTOR1_DRIVE_Pin GPIO_PIN_9
#define THYRISTOR1_DRIVE_GPIO_Port GPIOB

/* Global types ------------------------------------------------------------*/
enum
{
    FALSE = 0,
    TRUE = !FALSE
};
typedef uint8_t Bool_t;

/* Global constants --------------------------------------------------------*/
#ifndef M_2PI
#define M_2PI 6.283185307179586f
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

#ifndef M_ROOT2
#define M_ROOT2 1.4142135623f
#endif

#endif