#include "ammeter.h"
#include <math.h>

/* Imported variables ------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* Private definitions -----------------------------------------------------*/
#define MAX_BREAK_COUNT (NUM_OF_MEASUREMENT_CYCLES * AMMETER_MAX_BREAK_OCCURRANCE)
#define NUM_OF_MEASUREMENT_CYCLES 128
#define TOTAL_DATAPOINTS (NUM_OF_MEASUREMENT_CYCLES * SAMPLES_OVER_PERIOD)
#define RSENS 6.8e-3
#define ACP7900_GAIN 8.2f
#define SUBTRACTOR_GAIN (3.6f / 4.7f)
#define SENS_CIRCUIT_GAIN (RSENS * ACP7900_GAIN * SUBTRACTOR_GAIN)
#define ADC_BITS 12
#define ADC_BIAS 2052

/* Private types -----------------------------------------------------------*/
enum
{
    KICKSTART = (1 << 0),
    CONVERSION_COMPLETED = (1 << 1),
    PITFALL_DETECTED = (1 << 2)
};

/* Private variables -------------------------------------------------------*/
// Flags.
static volatile uint8_t Events;
static volatile Bool_t Measuring;

// Runtime.
static uint16_t BreakCount;

// Buffer.
static uint16_t ADCBuffer[TOTAL_DATAPOINTS];

/* Exported functions ------------------------------------------------------*/
void Ammeter_Start(void)
{
    // Calibrate ADC.
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        while (TRUE)
            ;
    }

    // Enable tim1 update interrupts.
    HAL_TIM_Base_Start_IT(&htim1);

    Events = 0;
    Measuring = FALSE;
}

void Ammeter_Execute(void)
{
    if (Events & PITFALL_DETECTED)
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        HAL_ADC_Stop_DMA(&hadc1);
        Measuring = FALSE;
        Events ^= PITFALL_DETECTED;
    }

    if (Events & CONVERSION_COMPLETED)
    {
        if (Measuring)
        {
            int32_t sum = 0;
            float currentrms;

            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

            // Process input data.
            for (uint16_t i = 0; i < TOTAL_DATAPOINTS; i++)
            {
                sum += ADCBuffer[i];
            }

            currentrms = (((float)sum / TOTAL_DATAPOINTS) - ADC_BIAS) *
                          (2 * M_ROOT2 / (SENS_CIRCUIT_GAIN * (1 << ADC_BITS)));

            // Send the result.
            Ammeter_MeasurementCompletedCb(currentrms);
            
            Measuring = FALSE;

            Events ^= CONVERSION_COMPLETED;
        }
    }
}

void Ammeter_Stop(void)
{
    if (Measuring)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        Measuring = FALSE;
    }
}

void Ammeter_Measure(uint32_t driveSignalPeriod)
{
    uint16_t period;

    // Set period and the compare value of the sampling timer.
    period = (uint16_t)((driveSignalPeriod / SAMPLES_OVER_PERIOD) + 0.5f);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period >> 1);

    // Start ADC.
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCBuffer, TOTAL_DATAPOINTS);

    // Set event to start triggering ADC.
    Events = 0;
    Events |= KICKSTART;

    // Start rtv.
    Measuring = TRUE;
    BreakCount = 0;
}

Bool_t Ammeter_IsBusy(void)
{
    return Measuring;
}

/* Private functions -------------------------------------------------------*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (Measuring)
    {
        Events |= CONVERSION_COMPLETED;
    }
}

/* Delegates ---------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        if (Events & KICKSTART)
        {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
            Events ^= KICKSTART;
        }
    }
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    // If this point is a pitfall; record it as a pitfall.
    if (htim == &htim1)
    {
        if (BreakCount++ > MAX_BREAK_COUNT)
        {
            Ammeter_PitfallDetectedCb();
            Events |= PITFALL_DETECTED;
        }
    }
}

/* Callbacks ---------------------------------------------------------------*/
__weak void Ammeter_MeasurementCompletedCb(float rmsCurrent)
{
}

__weak void Ammeter_PitfallDetectedCb(void)
{
}