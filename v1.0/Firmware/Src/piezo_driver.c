#include "piezo_driver.h"

/* Private functions -------------------------------------------------------*/
static void startPwm(void);
static void stopPwm(void);
static void setTIMRegisters(uint32_t period);

/* Imported variables ------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* Private variables -------------------------------------------------------*/
static Bool_t HBridgeEnabled = FALSE;
static uint32_t Period = (TIM2_CLOCK_FREQ / DEFAULT_PWM_FREQ);

/* Exported functions ------------------------------------------------------*/
void Pd_SetFrequency(float frequency)
{
    Period = (uint32_t)(TIM2_CLOCK_FREQ / frequency + 0.5f);

    if (HBridgeEnabled)
    {
        setTIMRegisters(Period);
    }
}

void Pd_SetPeriod(uint32_t period)
{
    Period = period;

    if (HBridgeEnabled)
    {
        setTIMRegisters(period);
    }
}

void Pd_CmdSignalGeneration(Bool_t enabled)
{
    if (enabled && !HBridgeEnabled)
    {
        setTIMRegisters(Period);
        startPwm();
        HBridgeEnabled = TRUE;
    }

    if (!enabled && HBridgeEnabled)
    {
        stopPwm();
        HBridgeEnabled = FALSE;
    }
}

/* Private functions -------------------------------------------------------*/
void startPwm(void)
{
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
}

void stopPwm(void)
{
    // Stop PWM generation.
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_2);
}

void setTIMRegisters(uint32_t period)
{
    // Set compare and reload values.
    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period >> 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period >> 1);
}