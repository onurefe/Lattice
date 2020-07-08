#include "lattice.h"
#include "math.h"

/* Private definitions -----------------------------------------------------*/
#define SHUNT_RESISTOR 10e-3
#define VDIV_RATIO (324 / 2.2e6)

// Signal processing definitions.
#define SIGNAL_PROCESSING_BLANKED_CYCLES 4

// ADC definitions.
#define ADC_CLOCK_FREQ MCU_CLOCK_FREQ
#define ADC_SAMPLING_CYLES 19.5f
#define ADC_DEADTIME 2.0f
#define ADC_CHANNELS 2
#define ADC_BITS 12
#define ADC_BUFFER_ELEMENT_COUNT (ADC_CHANNELS * PWM_CYCLES_PER_CONTROL_CYCLE * SAMPLES_PER_CYCLE)
#define ADC_BLANKED_ELEMENT_COUNT (SAMPLES_PER_CYCLE * ADC_CHANNELS * SIGNAL_PROCESSING_BLANKED_CYCLES)

// Analog frontend definitions.
#define AFE_ISOAMP_GAIN 8.2f
#define AFE_POSTAMP_GAIN (1.0f + 1.0f / 3.3f)
#define AFE_VOLTAGE_GAIN (VDIV_RATIO * AFE_ISOAMP_GAIN * AFE_POSTAMP_GAIN)
#define AFE_CURRENT_GAIN (SHUNT_RESISTOR * AFE_ISOAMP_GAIN * AFE_POSTAMP_GAIN)

// Resonance frequency searching definitions.
#define RES_SEARCHING_RESOLUTION 2.0f

// Resonance tracking algorithm definitions.
#define RES_TRACKING_MAX_FREQ_JUMP 0.2e3
#define RES_TRACKING_LOCKED_WINDOW_IN_MEASURE 0.075f
#define RES_TRACKING_DEFAULT_QUALITY_FACTOR 500.0f
#define RES_TRACKING_QUALITY_FACTOR_FILTER_COEFF 0.20f
#define RES_TRACKING_LOADED_QUALITY_FACTOR_THRESHOLD 200.0f
#define RES_TRACKING_LOADED_POWER_IN_WATTS 1000.0f
#define RES_TRACKING_IDLE_POWER_IN_WATTS 200.0f

// Power tracking algorithm definitions.
#define POWER_TRACKING_PI_GAIN ((MAX_PWM_DUTY - MIN_PWM_DUTY) / RES_TRACKING_LOADED_POWER_IN_WATTS)
#define POWER_TRACKING_PI_INTEGRAL_TC 0.01f
#define POWER_TRACKING_PI_KI (POWER_TRACKING_PI_GAIN / POWER_TRACKING_PI_INTEGRAL_TC)
#define POWER_TRACKING_ON_TRACK_WINDOW 100.0f

#if (SAMPLES_PER_CYCLE != 12)
#error "Implementation is not compatible with different sample count than 12."
#endif

/* Private types -----------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------*/
static inline void control(uint16_t *adcBuffer);
static inline void getPhasors(uint16_t *bf, float *current_x, float *current_y,
                       float *voltage_x, float *voltage_y);
static inline float getDuty(float normalizedPower);
static inline float fastacos(float x);
static inline void updateHrtimRegisters(void);

/* Imported variables ------------------------------------------------------*/
extern HRTIM_HandleTypeDef hhrtim1;
extern ADC_HandleTypeDef hadc1;

/* Private variables -------------------------------------------------------*/
// Runtime variables------------------.
static Lattice_Status_t LastStatus;
static volatile Lattice_Status_t Status;

// Variables used when resonance point searching mode.
static volatile float ResonanceFrequency;
static volatile float ResonanceRealPower;
static volatile float ResonanceImgPower;

// Variables used when resonance point tracking mode.
static volatile float LastTrackingMeasure;
static volatile float LastPwmFrequency;
static volatile float QualityFactor;
static volatile float FilteredQualityFactor;
static volatile float PowerTrackingIntegral;

// Variables used in common.
static volatile float PwmFrequency;
static volatile float PwmDuty;

// Buffers---------------------------.
static uint16_t AdcBuffer[ADC_BUFFER_ELEMENT_COUNT];

/* Exported functions ------------------------------------------------------*/
void Lattice_Start(void)
{
    // Initialize run-time variables.
    LastStatus.controlMode = LATTICE_CONTROL_MODE_RES_FREQ_SEARCHING;
    LastStatus.loaded = FALSE;
    LastStatus.powerTrackingOnTrack = FALSE;
    LastStatus.resonanceTrackingOnTrack = FALSE;
    Status.controlMode = LATTICE_CONTROL_MODE_RES_FREQ_SEARCHING;
    Status.loaded = FALSE;
    Status.powerTrackingOnTrack = FALSE;
    Status.resonanceTrackingOnTrack = FALSE;
    ResonanceFrequency = MIN_PWM_FREQ;
    ResonanceRealPower = 0.0f;
    ResonanceImgPower = 0.0f;
    QualityFactor = RES_TRACKING_DEFAULT_QUALITY_FACTOR;
    FilteredQualityFactor = RES_TRACKING_DEFAULT_QUALITY_FACTOR;
    PowerTrackingIntegral = 0.0f;
    PwmFrequency = MIN_PWM_FREQ;
    PwmDuty = getDuty(SEARCHING_NORMALIZED_POWER);

    // Start ADC calibration.
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);

    // Start ADC.
    HAL_ADC_Start_DMA(&hadc1, ((uint32_t *)AdcBuffer), ADC_BUFFER_ELEMENT_COUNT);

    updateHrtimRegisters();

    // Start timer to trigger ADC.
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);

    // Start pulse generation.
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                     HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_ACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                     HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
}

void Lattice_Execute(void)
{
    // If the status has been changed; inform it to the client.
    if ((LastStatus.controlMode != Status.controlMode) ||
        (LastStatus.loaded != Status.loaded) ||
        (LastStatus.powerTrackingOnTrack != Status.powerTrackingOnTrack) ||
        (LastStatus.resonanceTrackingOnTrack != Status.resonanceTrackingOnTrack))
    {
        LastStatus.controlMode = Status.controlMode;
        LastStatus.loaded = Status.loaded;
        LastStatus.powerTrackingOnTrack = Status.powerTrackingOnTrack;
        LastStatus.resonanceTrackingOnTrack = Status.resonanceTrackingOnTrack;

        Lattice_StatusChangeCb(&LastStatus);
    }
}

void Lattice_Stop(void)
{
    // Stop triggering ADC.
    HAL_HRTIM_WaveformCounterStop(&hhrtim1, HRTIM_TIMERID_TIMER_B);

    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformCounterStop(&hhrtim1, HRTIM_TIMERID_TIMER_A);

    // Put both outputs into the inactive state.
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                     HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                     HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_INACTIVE);
}

/* Delegates ---------------------------------------------------------------*/
/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @param  hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        control(&AdcBuffer[0]);
    }
}
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        control(&AdcBuffer[ADC_BUFFER_ELEMENT_COUNT >> 1]);
    }
}

/* Callbacks ---------------------------------------------------------------*/
__weak void Lattice_StatusChangeCb(Lattice_Status_t *status)
{
}

/* Private functions -------------------------------------------------------*/
inline void control(uint16_t *adcBuffer)
{
    float power_abs, power_real, power_img;
    float current_x, current_y, voltage_x, voltage_y;

    // Process data in the adc buffer and obtain current and voltage in phasor form.
    getPhasors(adcBuffer, &current_x, &current_y, &voltage_x, &voltage_y);

    // Calculate power in phasor form.
    power_real = voltage_x * current_x - voltage_y * current_y;
    power_img = voltage_x * current_y + voltage_y * current_x;
    power_abs = sqrtf(power_real * power_real + power_img * power_img);

    if (Status.controlMode == LATTICE_CONTROL_MODE_RES_FREQ_TRACKING)
    {
        float tracking_measure;
        float next_pwm_freq;

        // Secant like algorithm would do the work.
        tracking_measure = power_img / power_real;

        if (tracking_measure > RES_TRACKING_LOCKED_WINDOW_IN_MEASURE)
        {
            next_pwm_freq = PwmFrequency - tracking_measure * ((PwmFrequency - LastPwmFrequency) /
                                                               (tracking_measure - LastTrackingMeasure));

            // Limit jumping.
            if (next_pwm_freq > (PwmFrequency + RES_TRACKING_MAX_FREQ_JUMP))
            {
                next_pwm_freq = (PwmFrequency + RES_TRACKING_MAX_FREQ_JUMP);
            }

            if (next_pwm_freq < (PwmFrequency - RES_TRACKING_MAX_FREQ_JUMP))
            {
                next_pwm_freq = (PwmFrequency - RES_TRACKING_MAX_FREQ_JUMP);
            }

            float p;

            // Update Q value.
            p = PwmFrequency / ResonanceFrequency;
            QualityFactor = fabsf((power_real / power_img) * (1 / p - p));

            // Set current values last values.
            LastTrackingMeasure = tracking_measure;
            LastPwmFrequency = PwmFrequency;

            // Update pwm frequency.
            PwmFrequency = next_pwm_freq;

            Status.resonanceTrackingOnTrack = FALSE;
        }
        else
        {
            Status.resonanceTrackingOnTrack = TRUE;
        }

        // Filter quality factor in order to use it in controlling output power.
        FilteredQualityFactor = QualityFactor * RES_TRACKING_QUALITY_FACTOR_FILTER_COEFF +
                                FilteredQualityFactor * (1.0f - RES_TRACKING_QUALITY_FACTOR_FILTER_COEFF);

        float dest_output_power;

        // Get destination output power.
        if (FilteredQualityFactor > RES_TRACKING_LOADED_QUALITY_FACTOR_THRESHOLD)
        {
            dest_output_power = RES_TRACKING_IDLE_POWER_IN_WATTS;
            Status.loaded = FALSE;
        }
        else
        {
            dest_output_power = RES_TRACKING_LOADED_POWER_IN_WATTS;
            Status.loaded = TRUE;
        }

        float pi_in;
        float pi_out;

        // Apply duty cycle control to control output power.
        pi_in = dest_output_power - power_abs;
        pi_out = (1.0f + PowerTrackingIntegral / POWER_TRACKING_PI_INTEGRAL_TC) * POWER_TRACKING_PI_GAIN;

        // Update integral with anti-windup protection.
        if (!(((pi_out > MAX_PWM_DUTY) && (pi_in > 0.0f)) || ((pi_out < MIN_PWM_DUTY) && (pi_in < 0.0f))))
        {
            PowerTrackingIntegral += (PWM_CYCLES_PER_CONTROL_CYCLE / LastPwmFrequency) * pi_in;
        }

        // Set status if power tracking is performed well or not.
        if (fabsf(pi_in) < POWER_TRACKING_ON_TRACK_WINDOW)
        {
            Status.powerTrackingOnTrack = TRUE;
        }
        else
        {
            Status.powerTrackingOnTrack = FALSE;
        }

        // Update PWM duty.
        PwmDuty = getDuty(pi_out);

        updateHrtimRegisters();
    }
    else if (Status.controlMode == LATTICE_CONTROL_MODE_RES_FREQ_SEARCHING)
    {
        // Update the maxima.
        if (power_real > ResonanceRealPower)
        {
            ResonanceFrequency = PwmFrequency;
            ResonanceRealPower = power_real;
            ResonanceImgPower = power_img;
        }

        PwmFrequency += RES_SEARCHING_RESOLUTION;

        // Searching completed?
        if (PwmFrequency > MAX_PWM_FREQ)
        {
            // Use resonance point as a start point to tracking algorithm.
            LastTrackingMeasure = ResonanceImgPower / ResonanceRealPower;
            LastPwmFrequency = ResonanceFrequency;
            PwmFrequency = ResonanceFrequency + RES_SEARCHING_RESOLUTION;

            Status.controlMode = LATTICE_CONTROL_MODE_RES_FREQ_TRACKING;
        }

        updateHrtimRegisters();
    }
}

inline void getPhasors(uint16_t *bf, float *current_x, float *current_y,
                       float *voltage_x, float *voltage_y)
{
    float isx = 0.0;
    float isy = 0.0;
    float vsx = 0.0;
    float vsy = 0.0;
    uint16_t off = SAMPLES_PER_CYCLE * ADC_CHANNELS * SIGNAL_PROCESSING_BLANKED_CYCLES;

    // Process all the elements.
    while (off < ADC_BUFFER_ELEMENT_COUNT)
    {
        isx += bf[off];
        off++;
        vsx += bf[off];
        off++;

        isx += 0.8660254037844386f * bf[off];
        isy += 0.5f * bf[off];
        off++;
        vsx += 0.8660254037844386f * bf[off];
        vsy += 0.5f * bf[off];
        off++;

        isx += 0.5f * bf[off];
        isy += 0.8660254037844386f * bf[off];
        off++;
        vsx += 0.5f * bf[off];
        vsy += 0.8660254037844386f * bf[off];
        off++;

        isy += bf[off];
        off++;
        vsy += bf[off];
        off++;

        isx -= 0.5f * bf[off];
        isy += 0.8660254037844386f * bf[off];
        off++;
        vsx -= 0.5f * bf[off];
        vsy += 0.8660254037844386f * bf[off];
        off++;

        isx -= 0.8660254037844386f * bf[off];
        isy += 0.5f * bf[off];
        off++;
        vsx -= 0.8660254037844386f * bf[off];
        vsy += 0.5f * bf[off];
        off++;

        isx -= bf[off];
        off++;
        vsx -= bf[off];
        off++;

        isx -= 0.8660254037844386f * bf[off];
        isy -= 0.5f * bf[off];
        off++;
        vsx -= 0.8660254037844386f * bf[off];
        vsy -= 0.5f * bf[off];
        off++;

        isx -= 0.5f * bf[off];
        isy -= 0.8660254037844386f * bf[off];
        off++;
        vsx -= 0.5f * bf[off];
        vsy -= 0.8660254037844386f * bf[off];
        off++;

        isy -= bf[off];
        off++;
        vsy -= bf[off];
        off++;

        isx += 0.5f * bf[off];
        isy -= 0.8660254037844386f * bf[off];
        off++;
        vsx += 0.5f * bf[off];
        vsy -= 0.8660254037844386f * bf[off];
        off++;

        isx += 0.8660254037844386f * bf[off];
        isy -= 0.5f * bf[off];
        off++;
        vsx += 0.8660254037844386f * bf[off];
        vsy -= 0.5f * bf[off];
        off++;
    }

    float fix_x, fix_y;

    // Calculate sampling related phase shift. Small angle approximation for sine is used implicitly.
    fix_y = -PwmFrequency * ((M_2PI * ADC_SAMPLING_CYLES + ADC_BITS + ADC_DEADTIME) / ADC_CLOCK_FREQ);
    fix_x = sqrtf(1.0f - fix_y * fix_y);

    // Apply the fix.
    vsx = vsx * fix_x - vsy * fix_y;
    vsy = vsx * fix_y + vsy * fix_x;

    static float divisor;
    divisor = AFE_VOLTAGE_GAIN * (1 << ADC_BITS) *
              ((ADC_BUFFER_ELEMENT_COUNT - ADC_BLANKED_ELEMENT_COUNT) / ADC_CHANNELS);

    // Calculate voltage and current phasors.
    *voltage_x = vsx / divisor;
    *voltage_y = vsy / divisor;
    *current_x = isx / divisor;
    *current_y = isy / divisor;
}

inline float getDuty(float normalizedPower)
{
    return (fastacos(1.0f - 2.0f * normalizedPower) / M_2PI);
}

inline float fastacos(float x)
{
    float a = -0.939115566365855f;
    float b = 0.9217841528914573f;
    float c = -1.2845906244690837f;
    float d = 0.295624144969963174f;

    float x2;
    float num;
    float denom;

    x2 = x * x;
    num = x * (a + b * x2);
    denom = 1 + x2 * (c + d * x2);

    return (M_PI_2 + num / denom);
}

inline void updateHrtimRegisters(void)
{
    // Update TimerA registers.
    uint16_t period;
    uint16_t comp;
    period = (uint16_t)((HRTIM_CLOCK_FREQ / PwmFrequency) + 0.5f);
    comp = (uint16_t)((HRTIM_CLOCK_FREQ * PwmDuty / PwmFrequency) + 0.5f);

    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, comp);

    // Update TimerB registers.
    period = (uint16_t)((HRTIM_CLOCK_FREQ / (SAMPLES_PER_CYCLE * PwmFrequency)) + 0.5f);
    comp = period >> 1;

    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, comp);
}