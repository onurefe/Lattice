#include "synchronous_rectifier.h"
#include "math.h"

/* Private definitions -----------------------------------------------------*/
#define CROSSING_EMA_FILTER_COEFF 0.2f
#define CHOCK_IND 10e-3
#define FILTER_CAP 1.5e-3
#define MAX_SURGE_CURRENT 10.0f
#define PEAK_VOLTAGE 311.1269f
#define MAINS_FREQ 50.0f
#define MAINS_AMPLITUDE 311.1269837220809f
#define ALPHA_OF_BRIDGE_TRANSITION 0.6f

/* Imported variables ------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;

/* Private variables --------------------------------------------------------*/
// Runtime variables.
static volatile float Alpha;
static volatile uint16_t LastCapture;
static float AlphaSlewRate;
static float AlphaDestination;
static volatile float PositivePeriodInUs;
static volatile float NegativePeriodInUs;
static Bool_t PhaseLocked;
static Bool_t ToSetDrive0;
static Bool_t ToSetDrive1;

/* Exported functions ------------------------------------------------------*/
void Sr_Start(void)
{
    // Initialize runtime variables.
    Alpha = 0.0f;
    AlphaDestination = 0.0f;
    AlphaSlewRate = 0.0f;
    PositivePeriodInUs = (1e6 / (2.0f * MAINS_FREQ));
    NegativePeriodInUs = (1e6 / (2.0f * MAINS_FREQ));
    LastCapture = 0;
    ToSetDrive0 = FALSE;
    ToSetDrive1 = FALSE;
    PhaseLocked = FALSE;

    HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_RESET);

    // Start input capture interrupts.
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
}

Bool_t Sr_IsStabilized(void)
{
    return TRUE;
    if (AlphaDestination <= Alpha)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void Sr_Stop(void)
{
    // Stop input capture interrupts.
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_4);
}

void Sr_SetVoltage(float normalizedVoltage)
{
    float norm_destination_voltage;
    if (AlphaDestination >= 0.499f)
    {
        norm_destination_voltage = 1.0f;
    }
    else
    {
        norm_destination_voltage = sinf(M_PI * AlphaDestination);
    }

    if (normalizedVoltage >= norm_destination_voltage)
    {
        float settling_period;
        float current_volt2 = norm_destination_voltage * norm_destination_voltage;
        float dest_volt2 = normalizedVoltage * normalizedVoltage;

        settling_period = MAINS_AMPLITUDE * MAINS_AMPLITUDE * FILTER_CAP * (dest_volt2 - current_volt2) /
                          (2 * CHOCK_IND * MAX_SURGE_CURRENT * MAX_SURGE_CURRENT * MAINS_FREQ);

        AlphaDestination = asinf(normalizedVoltage) / M_PI;
        AlphaSlewRate = (AlphaDestination - Alpha) / (settling_period * 1e6);

        // If full turned on; set destination angle to M_PI.
        if (AlphaDestination > 0.499f)
        {
            AlphaDestination = 1.0f;
        }
    }
    else
    {
        AlphaDestination = asinf(normalizedVoltage) / M_PI;

        // If fully turned on; set destination angle 1.
        if (AlphaDestination > 0.499f)
        {
            AlphaDestination = 1.0f;
        }

        Alpha = AlphaDestination;
    }
}

/* Callback implementations ------------------------------------------------*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // Findout new capture value. Some kind of moving average filter may help resolving this issue.
    if (htim == &htim3)
    {
        // If the line is rising
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            if (!PhaseLocked)
            {
                LastCapture = htim->Instance->CCR3;
                PhaseLocked = TRUE;
                return;
            }

            ToSetDrive0 = FALSE;
            ToSetDrive1 = FALSE;

            volatile uint16_t ccr;
            volatile float negative_period;

            ccr = htim->Instance->CCR3;
            negative_period = (float)(ccr - LastCapture);

            if (negative_period < 0.0)
                negative_period += 65536.0f;

            NegativePeriodInUs = negative_period * CROSSING_EMA_FILTER_COEFF +
                                         NegativePeriodInUs * (1.0 - CROSSING_EMA_FILTER_COEFF);

            if (AlphaDestination > Alpha)
            {
                Alpha += (negative_period * AlphaSlewRate);
            }

            LastCapture = ccr;

            if (Alpha < ALPHA_OF_BRIDGE_TRANSITION)
            {
                HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_RESET);
                ToSetDrive0 = TRUE;
                
                uint16_t time2setdrive0;
                time2setdrive0 = ccr + (uint16_t)(PositivePeriodInUs * (1.0f - Alpha));

                // Start output compare interrupt.
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, time2setdrive0);
                HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
            }
            else
            {
                Alpha = AlphaDestination;

                HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_SET);
            }
        }

        // If falling edge has been detected(This means line is falling)
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            if (!PhaseLocked)
            {
                LastCapture = htim->Instance->CCR4;
                PhaseLocked = TRUE;
                return;
            }

            ToSetDrive0 = FALSE;
            ToSetDrive1 = FALSE;

            volatile uint16_t ccr;
            volatile float positive_period;

            ccr = htim->Instance->CCR4;
            positive_period = (float)(ccr - LastCapture);

            if (positive_period < 0.0)
                positive_period += 65536.0f;

            PositivePeriodInUs = positive_period * CROSSING_EMA_FILTER_COEFF +
                                PositivePeriodInUs * (1.0 - CROSSING_EMA_FILTER_COEFF);

            if (AlphaDestination > Alpha)
            {
                Alpha += (positive_period * AlphaSlewRate);
            }

            LastCapture = ccr;

            if (Alpha < ALPHA_OF_BRIDGE_TRANSITION)
            {
                HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_RESET);
                ToSetDrive1 = TRUE;

                uint16_t time2setdrive1;
                time2setdrive1 = ccr + (uint16_t)(NegativePeriodInUs * (1.0f - Alpha));

                // Start output compare interrupt.
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, time2setdrive1);
                HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);
            }
            else
            {
                Alpha = AlphaDestination;
                HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_SET);
            }
        }
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            if (ToSetDrive0)
            {
                ToSetDrive0 = FALSE;
                HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_SET);
            }
        }

        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if (ToSetDrive1)
            {
                ToSetDrive1 = FALSE;
                HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_SET);
            }
        }
    }
}
