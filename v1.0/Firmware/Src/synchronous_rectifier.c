#include "synchronous_rectifier.h"
#include "math.h"

/* Private definitions -----------------------------------------------------*/
#define ZERO_CROSSING_PERIOD_EMA_FILTER_COEFF 0.01f
#define CHOCK_IND 17.8e-3
#define FILTER_CAP 1.5e-3
#define MAX_SURGE_CURRENT 2.0f
#define PEAK_VOLTAGE 311.1269f
#define MAINS_FREQ 50.0f
#define MAINS_AMPLITUDE 311.1269837220809f
#define ALPHA_OF_BRIDGE_TRANSITION 0.6f

/* Imported variables ------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private variables --------------------------------------------------------*/
// Runtime variables.
static volatile float Alpha;
static volatile uint16_t LastCapture;
static float AlphaSlewRate;
static float AlphaDestination;
static volatile float MeanZeroCrossingPeriodInUs;
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
    MeanZeroCrossingPeriodInUs = (1e6 / (2.0f * MAINS_FREQ));
    LastCapture = 0;
    ToSetDrive0 = FALSE;
    ToSetDrive1 = FALSE;
    PhaseLocked = FALSE;

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_RESET);

    // Start input capture interrupts.
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
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
        // If rising edge has been detected.
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            if (!PhaseLocked)
            {
                LastCapture = htim->Instance->CCR3;
                PhaseLocked = TRUE;
                return;
            }

            HAL_TIM_Base_Stop_IT(&htim2);

            volatile uint16_t ccr;
            volatile float zcr_period;

            ccr = htim->Instance->CCR3;
            zcr_period = (float)(ccr - LastCapture);

            if (zcr_period < 0.0)
                zcr_period += 65536.0f;

            MeanZeroCrossingPeriodInUs = zcr_period * ZERO_CROSSING_PERIOD_EMA_FILTER_COEFF +
                                         MeanZeroCrossingPeriodInUs * (1.0 - ZERO_CROSSING_PERIOD_EMA_FILTER_COEFF);

            if (AlphaDestination > Alpha)
            {
                Alpha += (zcr_period * AlphaSlewRate);
            }

            LastCapture = ccr;

            if (Alpha < ALPHA_OF_BRIDGE_TRANSITION)
            {
                HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_RESET);
                ToSetDrive0 = TRUE;
                ToSetDrive1 = FALSE;

                __HAL_TIM_SET_AUTORELOAD(&htim2, (uint16_t)(MeanZeroCrossingPeriodInUs * (1.0f - Alpha)));
                HAL_TIM_Base_Start_IT(&htim2);
                __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
            }
            else
            {
                ToSetDrive0 = FALSE;
                ToSetDrive1 = FALSE;

                HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_SET);
            }
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            if (!PhaseLocked)
            {
                LastCapture = htim->Instance->CCR4;
                PhaseLocked = TRUE;
                return;
            }

            HAL_TIM_Base_Stop_IT(&htim4);

            volatile uint16_t ccr;
            volatile float zcr_period;

            ccr = htim->Instance->CCR4;
            zcr_period = (float)(ccr - LastCapture);

            if (zcr_period < 0.0)
                zcr_period += 65536.0f;

            MeanZeroCrossingPeriodInUs = zcr_period * ZERO_CROSSING_PERIOD_EMA_FILTER_COEFF +
                                         MeanZeroCrossingPeriodInUs * (1.0 - ZERO_CROSSING_PERIOD_EMA_FILTER_COEFF);

            if (AlphaDestination > Alpha)
            {
                Alpha += (zcr_period * AlphaSlewRate);
            }

            LastCapture = ccr;

            if (Alpha < ALPHA_OF_BRIDGE_TRANSITION)
            {
                HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_RESET);
                ToSetDrive0 = FALSE;
                ToSetDrive1 = TRUE;

                __HAL_TIM_SET_AUTORELOAD(&htim4, (uint16_t)(MeanZeroCrossingPeriodInUs * (1.0f - Alpha)));
                HAL_TIM_Base_Start_IT(&htim4);
                __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
            }
            else
            {
                ToSetDrive0 = FALSE;
                ToSetDrive1 = FALSE;

                HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_SET);
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        if (ToSetDrive0)
        {
            ToSetDrive0 = FALSE;
            HAL_GPIO_WritePin(THYRISTOR0_DRIVE_GPIO_Port, THYRISTOR0_DRIVE_Pin, GPIO_PIN_SET);
        }
    }

    if (htim == &htim4)
    {
        if (ToSetDrive1)
        {
            ToSetDrive1 = FALSE;
            HAL_GPIO_WritePin(THYRISTOR1_DRIVE_GPIO_Port, THYRISTOR1_DRIVE_Pin, GPIO_PIN_SET);
        }
    }
}