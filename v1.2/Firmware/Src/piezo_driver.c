#include "piezo_driver.h"

#define FREQ_SWEEP_RATE_PER_SEC 100.0f
#define SWEEP_START_FREQ 25000.0f

/* Private functions -------------------------------------------------------*/
static void startPwm(void);
static void stopPwm(void);
static void setReload(float frequency);

/* Imported variables ------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* Private variables -------------------------------------------------------*/
static Bool_t HBridgeEnabled = FALSE;
static float Frequency = DEFAULT_PWM_FREQ;

/* Exported functions ------------------------------------------------------*/
void Pd_CmdSignalGeneration(Bool_t enabled)
{
    if (enabled && !HBridgeEnabled)
    {
        setReload(Frequency);
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

void setReload(float frequency)
{
    uint16_t reload;
    reload = ((uint16_t)(((float)TIM1_CLOCK_FREQ) / frequency + 0.5f)) - 1;

    // Set compare and reload values.
    __HAL_TIM_SET_AUTORELOAD(&htim1, reload);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (reload + 1) >> 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (reload + 1) >> 1);
}