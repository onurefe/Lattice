#include "core.h"
#include "math.h"
#include "matrix.h"
#include "eeprom_emulator.h"
#include "polyreg.h"

/* Private definitions -----------------------------------------------------*/
// Measurement.
#define SHUNT_RESISTOR 0.01f
#define VDIV_RATIO (324.0f / 2200000.0f)

// ADC definitions.
#define ADC_CHANNELS 2
#define ADC_BITS 12
#define ADC_DOUBLE_BUFFER_ELEMENT_COUNT (2 * ADC_CHANNELS * SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT)
#define ADC_SINGLE_BUFFER_ELEMENT_COUNT (ADC_DOUBLE_BUFFER_ELEMENT_COUNT >> 1)
#define ADC_VREF 3.3f

// Analog frontend definitions.
#define AFE_ISOAMP_GAIN 8.2f
#define AFE_POSTAMP_GAIN (1.0f + (1.0f / 3.3f))
#define AFE_LPASS_GAIN 0.8952749158393931f
#define AFE_VOLTAGE_GAIN (AFE_LPASS_GAIN * VDIV_RATIO * AFE_ISOAMP_GAIN * AFE_POSTAMP_GAIN)
#define AFE_CURRENT_GAIN (AFE_LPASS_GAIN * SHUNT_RESISTOR * AFE_ISOAMP_GAIN * AFE_POSTAMP_GAIN)

// Signal processing definitions.
#define SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT (PWM_CYCLES_PER_CONTROL_CYCLE * SAMPLES_PER_CYCLE)
#define SIGNAL_PROCESSING_BLANKED_CYCLES 4
#define SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT (SIGNAL_PROCESSING_BLANKED_CYCLES * SAMPLES_PER_CYCLE)
#define SIGNAL_PROCESSING_VOLTAGE_MULTIPLIER ((4.0f * ADC_VREF) / (M_ROOT2 * AFE_VOLTAGE_GAIN * (1 << ADC_BITS) * \
                                                                   (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT - SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT)))
#define SIGNAL_PROCESSING_CURRENT_MULTIPLIER ((4.0f * ADC_VREF) / (M_ROOT2 * AFE_CURRENT_GAIN * (1 << ADC_BITS) * \
                                                                   (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT - SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT)))

#if (SAMPLES_PER_CYCLE != 12)
#error "Implementation is not compatible with different sample count than 12."
#endif

/* Private typedefs --------------------------------------------------------*/
enum
{
    EVENT_SCAN_COMPLETED = (1 << 0),
    EVENT_SEARCH_COMPLETED = (1 << 1),
    EVENT_PERIODIC_MEASUREMENT = (1 << 2),
    EVENT_ERROR_OCCURRED = (1 << 3)
};
typedef uint16_t Event_t;

/* Private function prototypes ---------------------------------------------*/
static inline void control(uint16_t *adcBuffer);
static inline void scanner(Complex_t *voltage, Complex_t *current);
static inline void searcher(Complex_t *voltage, Complex_t *current);
static inline void tracker(Complex_t *voltage, Complex_t *current);
static inline void getPhasors(uint16_t *bf, Complex_t *voltage, Complex_t *current);
static void engineStart(void);
static void engineStop(void);
static inline void updateClockCircuitry(float pwmFrequency, float pwmDuty);
static inline float getDuty(float normalizedPower);
static inline void ema(float *param, float value, float cfilt);
static inline void emaComplex(Complex_t *param, Complex_t *value, float cfilt);
static inline float clamp(float value, float min, float max);
static inline float fastacos(float x);

/* Imported variables ------------------------------------------------------*/
extern HRTIM_HandleTypeDef hhrtim1;
extern ADC_HandleTypeDef hadc1;

/* Private variables -------------------------------------------------------*/
// State machine implementation.
static volatile Core_Status_t Status = CORE_STATUS_READY;
static volatile uint16_t Events;

// Factory data.
static float ConstraintMinFrequency;
static float ConstraintMaxFrequency;
static float ConstraintMinNormPower;
static float ConstraintMaxNormPower;
static Complex_t CalPoly[CALIBRATION_POLY_DEGREE + 1];
static Bool_t Calibrated;

// Control variables used in tracking mode.
static float TrackingAnchorFrequency;
static float TrackingFullPower;
static float TrackingDestinationPower;
static Pid_t TrackingPowerPid;
static Pid_t TrackingFrequencyPid;
static Bool_t TrackingTriggeredLagged;
static Bool_t TrackingTriggered;

// Monitored variables and their shadow registers.
static float MonitoringPeriod;
static Bool_t MonitoringEnabled;
static float MonitoringStopwatch;
static Complex_t MonitoringPower;
static Complex_t MonitoringImpedance;
static float MonitoringFrequency;
static float MonitoringLoading;
static Complex_t MonitoringPowerShadow;
static Complex_t MonitoringImpedanceShadow;
static float MonitoringFrequencyShadow;
static float MonitoringLoadingShadow;

// Control variables used in searching mode.
static float SearchingResonanceFrequency;
static float SearchingResonanceImpedance;
static float SearchingFullPower;

// Variables used in scanning mode.
static float *ScanningImpedanceReal;
static float *ScanningImpedanceImg;

// Runtime variable used in searching mode and scanning mode.
static float NormalizedPower;
static float StartFrequency;
static float Deltaf;
static uint16_t Steps;
static uint16_t Iteration;

// Runtime variables used mutually in all modules.
static float PwmFrequency;
static float PwmDuty;

// Buffers---------------------------.
static uint16_t AdcBuffer[ADC_DOUBLE_BUFFER_ELEMENT_COUNT];

/* Exported functions ------------------------------------------------------*/
void Core_Init(Complex_t *calibrationCoeffs, float minFrequency, float maxFrequency,
               float minNormPower, float maxNormPower)
{
    if (Status != CORE_STATUS_UNINIT)
    {
        return;
    }

    // Set constraints of the device.
    ConstraintMinFrequency = minFrequency;
    ConstraintMaxFrequency = maxFrequency;
    ConstraintMinNormPower = minNormPower;
    ConstraintMaxNormPower = maxNormPower;

    // Not-triggered initially.
    TrackingTriggered = FALSE;
    TrackingTriggeredLagged = FALSE;

    // Periodic measurements disabled.
    MonitoringEnabled = FALSE;

    // Set calibration polynomial.
    if (calibrationCoeffs)
    {
        Complex_CopyArr(calibrationCoeffs, CalPoly, CALIBRATION_POLY_DEGREE + 1);
        Calibrated = TRUE;
    }
    else
    {
        Calibrated = FALSE;
    }
}

void Core_Scan(float startFrequency, float stopFrequency,
               uint16_t steps, float *impedanceReal, float *impedanceImg)
{
    // Status check.
    if (Status != CORE_STATUS_READY)
    {
        return;
    }

    // Copy return pointer.
    ScanningImpedanceReal = impedanceReal;
    ScanningImpedanceImg = impedanceImg;

    // Set control variables.
    StartFrequency = startFrequency;
    Deltaf = (stopFrequency - startFrequency) / steps;
    Steps = steps;

    // Clear searching runtime variables.
    Iteration = 0;

    // Update timers.
    updateClockCircuitry(startFrequency, getDuty(1.0f));

    // Start the machinery.
    engineStart();

    Events = 0;
    Status = CORE_STATUS_SCANNING;
}

void Core_Search(float normalizedPower, float startFrequency, float stopFrequency,
                 uint16_t steps)
{
    // Status check.
    if (Status != CORE_STATUS_READY)
    {
        return;
    }

    // Set control variables.
    NormalizedPower = normalizedPower;
    StartFrequency = startFrequency;
    Deltaf = (stopFrequency - startFrequency) / steps;
    Steps = steps;

    // Clear searching runtime variables.
    Iteration = 0;
    SearchingFullPower = 0.0f;
    SearchingResonanceImpedance = __FLT_MAX__;
    SearchingResonanceFrequency = startFrequency;

    // Update timers.
    updateClockCircuitry(startFrequency, getDuty(NormalizedPower));

    // Start the machinery.
    engineStart();

    Events = 0;
    Status = CORE_STATUS_SEARCHING;
}

void Core_Track(Pid_Params_t *powerTrackingPidParams,
                Pid_Params_t *frequencyTrackingPidParams,
                float anchorFrequency, float fullPower, float destinationPower)
{
    // Status check.
    if (Status != CORE_STATUS_READY)
    {
        return;
    }

    // Setup tracking PIDs.
    Pid_Setup(&TrackingPowerPid, powerTrackingPidParams, ConstraintMinNormPower,
              ConstraintMaxNormPower);

    Pid_Setup(&TrackingFrequencyPid, frequencyTrackingPidParams,
              (ConstraintMinFrequency - anchorFrequency),
              (ConstraintMaxFrequency - anchorFrequency));

    // Set control variables.
    TrackingAnchorFrequency = anchorFrequency;
    TrackingDestinationPower = destinationPower;
    TrackingFullPower = fullPower;

    // Clear monitoring runtime variables.
    Complex_Set(&MonitoringPower, destinationPower, 0.0f);
    Complex_Set(&MonitoringImpedance, 0.0f, 0.0f);
    MonitoringFrequency = anchorFrequency;
    MonitoringLoading = destinationPower / fullPower;
    MonitoringStopwatch = 0.0f;

    // Update timers.
    updateClockCircuitry(anchorFrequency, getDuty(ConstraintMinNormPower));

    // Start the engine. This will enable ADC operation and ISRs.
    engineStart();

    Events = 0;
    Status = CORE_STATUS_TRACKING;
}

void Core_Execute(void)
{
    // Validate state.
    if ((Status != CORE_STATUS_SCANNING) &&
        (Status != CORE_STATUS_SEARCHING) && (Status != CORE_STATUS_TRACKING))
    {
        return;
    }

    // Handle scan completed event.
    if ((Events & EVENT_SCAN_COMPLETED) != 0)
    {
        Status = CORE_STATUS_READY;

        engineStop();
        Core_ScanCompletedCallback();

        Events ^= EVENT_SCAN_COMPLETED;
    }

    // Handle search completed event.
    if ((Events & EVENT_SEARCH_COMPLETED) != 0)
    {
        Status = CORE_STATUS_READY;
        
        engineStop();
        Core_SearchCompletedCallback(SearchingResonanceFrequency,
                                     SearchingResonanceImpedance,
                                     SearchingFullPower);

        
        Events ^= EVENT_SEARCH_COMPLETED;
    }

    // Handle periodic measurement event.
    if ((Events & EVENT_PERIODIC_MEASUREMENT) != 0)
    {
        Core_PeriodicMeasurementCallback(MonitoringFrequencyShadow,
                                         MonitoringLoadingShadow, &MonitoringPowerShadow,
                                         &MonitoringImpedanceShadow);

        Events ^= EVENT_PERIODIC_MEASUREMENT;
    }
}

void Core_Stop(void)
{
    // Validate state.
    if ((Status != CORE_STATUS_SEARCHING) && (Status != CORE_STATUS_TRACKING))
    {
        return;
    }

    // Stop the engine.
    engineStop();

    Status = CORE_STATUS_READY;
}

void Core_TriggerStatusUpdate(Bool_t triggered)
{
    TrackingTriggered = triggered;
}

void Core_PeriodicMeasurements(Bool_t enabled, float period)
{
    if (enabled)
    {
        MonitoringEnabled = TRUE;
        MonitoringPeriod = period;
    }
    else
    {
        MonitoringEnabled = FALSE;
    }
}

Core_Status_t Core_GetStatus(void)
{
    return Status;
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
        control(&AdcBuffer[ADC_SINGLE_BUFFER_ELEMENT_COUNT]);
    }
}

/* Private functions -------------------------------------------------------*/
inline void control(uint16_t *adcBuffer)
{
    Complex_t voltage, current;

    // Process data in the adc buffer and obtain current and voltage in phasor form.
    getPhasors(adcBuffer, &voltage, &current);

    if (Status == CORE_STATUS_SCANNING)
    {
        scanner(&voltage, &current);
    }
    else if (Status == CORE_STATUS_SEARCHING)
    {
        searcher(&voltage, &current);
    }
    else if (Status == CORE_STATUS_TRACKING)
    {
        tracker(&voltage, &current);
    }
}

inline void scanner(Complex_t *voltage, Complex_t *current)
{
    if (Iteration < Steps)
    {
        Complex_t impedance;
        
        // Calculate impedance and store it.
        Complex_Divide(voltage, current, &impedance);
        ScanningImpedanceReal[Iteration] = impedance.real;
        ScanningImpedanceImg[Iteration] = impedance.img;

        // Increase iteration.
        Iteration++;

        // Update PWM frequency.
        float pwm_freq;
        pwm_freq = StartFrequency + Deltaf * Iteration;
        updateClockCircuitry(pwm_freq, getDuty(1.0f));
    }
    else
    {
        Events |= EVENT_SCAN_COMPLETED;
    }
}

inline void searcher(Complex_t *voltage, Complex_t *current)
{
    if (Iteration < Steps)
    {
        float power_real;
        float impedance_sqr;

        // Calculate real power.
        power_real = (voltage->real * current->real + voltage->img * current->img);

        // Calculate impedance.
        impedance_sqr = Complex_NormSqr(voltage) / Complex_NormSqr(current);

        // Look for minimum impedance.
        if (impedance_sqr < SearchingResonanceImpedance)
        {
            SearchingResonanceFrequency = PwmFrequency;
            SearchingResonanceImpedance = impedance_sqr;
            SearchingFullPower = power_real;
        }

        // Increase iteration.
        Iteration++;

        // Update PWM frequency.
        float pwm_freq;
        pwm_freq = StartFrequency + Deltaf * Iteration;
        updateClockCircuitry(pwm_freq, getDuty(NormalizedPower));
    }
    else
    {
        SearchingResonanceImpedance = sqrtf(SearchingResonanceImpedance);
        SearchingFullPower /= NormalizedPower;
        Events |= EVENT_SEARCH_COMPLETED;
    }
}

inline void tracker(Complex_t *voltage, Complex_t *current)
{
    Complex_t power;
    Complex_t impedance;
    Complex_t conj_current;
    float deltat;

    // Calculate four quadrant power.
    Complex_Conjugate(current, &conj_current);
    Complex_Multiply(voltage, &conj_current, &power);

    // Calculate four quadrant impedance.
    Complex_Divide(voltage, current, &impedance);

    // Calculate time passed since last call.
    deltat = PWM_CYCLES_PER_CONTROL_CYCLE / PwmFrequency;

    // Don't execute PIDs if not triggered.
    if (TrackingTriggeredLagged)
    {
        float duty;
        float frequency;

        // Get duty.
        duty = getDuty(pidExe(&TrackingPowerPid,
                              ((TrackingDestinationPower - power.real) / TrackingFullPower),
                              deltat));

        // Get frequency.
        frequency = pidExe(&TrackingFrequencyPid,
                           clamp((power.img / power.real), -1.0f, 1.0f), deltat) +
                    TrackingAnchorFrequency;

        updateClockCircuitry(frequency, duty);
    }
    else if (TrackingTriggered)
    {
        updateClockCircuitry(PwmFrequency, PwmDuty);
    }
    else
    {
        updateClockCircuitry(PwmFrequency, getDuty(ConstraintMinNormPower));
    }

    // Update monitoring values(if enabled).
    if (MonitoringEnabled)
    {
        float cfilt;
        cfilt = deltat / MonitoringPeriod;

        emaComplex(&MonitoringPower, &power, cfilt);
        emaComplex(&MonitoringImpedance, &impedance, cfilt);
        ema(&MonitoringFrequency, PwmFrequency, cfilt);
        ema(&MonitoringLoading, (power.real / TrackingFullPower), cfilt);

        MonitoringStopwatch += deltat;

        // If periodic measurements are enabled; update shadow registers.
        if (MonitoringStopwatch > MonitoringPeriod)
        {
            MonitoringStopwatch -= MonitoringPeriod;

            // Copy measurements to shadow registers.
            Complex_Copy(&MonitoringPower, &MonitoringPowerShadow);
            Complex_Copy(&MonitoringImpedance, &MonitoringImpedanceShadow);
            MonitoringFrequencyShadow = MonitoringFrequency;
            MonitoringLoadingShadow = MonitoringLoading;

            // Set periodic measurement event.
            Events |= EVENT_PERIODIC_MEASUREMENT;
        }
    }

    TrackingTriggeredLagged = TrackingTriggered;
}

inline void getPhasors(uint16_t *bf, Complex_t *voltage, Complex_t *current)
{
    static const float cosine[] = {1.0f, 0.8660254037844386f, 0.5f, 0.0f, -0.5f, -0.8660254037844386f};
    static const float sine[] = {0.0f, 0.5f, 0.8660254037844386f, 1.0f, 0.8660254037844386f, 0.5f};
    uint16_t off = SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT * ADC_CHANNELS;
    float sum[24] = {0.0f};

    // Sum elements which are to be multiplied with same modulation phase.
    while (off < (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT * ADC_CHANNELS))
    {
        sum[0] += bf[off++]; // current sample(phase 0).
        sum[1] += bf[off++]; // voltage sample(phase 0).
        sum[2] += bf[off++]; // current sample(phase 30).
        sum[3] += bf[off++]; // voltage sample(phase 30).
        sum[4] += bf[off++]; // current sample(phase 60).
        sum[5] += bf[off++]; // Goes on like this...
        sum[6] += bf[off++];
        sum[7] += bf[off++];
        sum[8] += bf[off++];
        sum[9] += bf[off++];
        sum[10] += bf[off++];
        sum[11] += bf[off++];
        sum[12] += bf[off++];
        sum[13] += bf[off++];
        sum[14] += bf[off++];
        sum[15] += bf[off++];
        sum[16] += bf[off++];
        sum[17] += bf[off++];
        sum[18] += bf[off++];
        sum[19] += bf[off++];
        sum[20] += bf[off++];
        sum[21] += bf[off++];
        sum[22] += bf[off++];
        sum[23] += bf[off++];
    }

    Complex_t v = {0.0f, 0.0f};
    Complex_t i = {0.0f, 0.0f};

    // Apply weighted sum to find unnormalized voltage and current phasors.
    for (uint8_t n = 0; n < 6; n++)
    {
        v.real += sum[2 * n + 1] - sum[2 * n + 13] * cosine[n];
        v.img += sum[2 * n + 1] - sum[2 * n + 13] * sine[n];
        i.real += sum[2 * n] - sum[2 * n + 12] * cosine[n];
        i.img += sum[2 * n] - sum[2 * n + 12] * sine[n];
    }

    // Correct analog circuitry errors if  calibrated.
    if (Calibrated)
    {
        // Calculate voltage calibration phasor.
        float in;
        Complex_t vcalib;
        in = PwmFrequency / 1e4f;
        
        Complex_PolyCalc(in, &CalPoly, &vcalib, CALIBRATION_POLY_DEGREE);
        Complex_Multiply(&v, &vcalib, &v);
    }

    // Normalize voltage and current phasors.
    Complex_MultiplyReal(&v, SIGNAL_PROCESSING_VOLTAGE_MULTIPLIER, &v);
    Complex_MultiplyReal(&i, SIGNAL_PROCESSING_CURRENT_MULTIPLIER, &i);
}

inline void updateClockCircuitry(float pwmFrequency, float pwmDuty)
{
    // Update PWM frequency & duty.
    PwmFrequency = pwmFrequency;
    PwmDuty = pwmDuty;

    // Update TimerA registers.
    uint16_t period;
    uint16_t comp;
    period = (uint16_t)((HRTIM_CLOCK_FREQ / pwmFrequency) + 0.5f);
    comp = (uint16_t)((HRTIM_CLOCK_FREQ * pwmDuty / pwmFrequency) + 0.5f);

    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, period);

    // Ensure that HRTIM is capable of making that compare value work.
    comp = (comp < 0x0018U) ? 0x0018U : comp;
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, comp);

    // Update TimerB registers.
    period = (uint16_t)((((float)period) / SAMPLES_PER_CYCLE) + 0.5f);
    comp = period >> 1;

    __HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, period);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, comp);
}

void engineStart(void)
{
    // Start ADC calibration.
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);

    // Start ADC.
    HAL_ADC_Start_DMA(&hadc1, ((uint32_t *)AdcBuffer), ADC_DOUBLE_BUFFER_ELEMENT_COUNT);

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

void engineStop(void)
{
    // Stop triggering ADC.
    HAL_HRTIM_WaveformCounterStop(&hhrtim1, HRTIM_TIMERID_TIMER_B);

    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformCounterStop(&hhrtim1, HRTIM_TIMERID_TIMER_A);

    // Stop ADC:
    HAL_ADC_Stop_DMA(&hadc1);

    // Put both outputs into the inactive state.
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                     HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_INACTIVE);
    HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                     HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_INACTIVE);
}

/* Default callbacks -------------------------------------------------------*/
__weak void Core_ScanCompletedCallback(void)
{
}

__weak void Core_SearchCompletedCallback(float resonanceFrequency, float resonanceImpedance,
                                         float fullPower)
{
}

__weak void Core_PeriodicMeasurementCallback(float frequency, float duty,
                                             Complex_t *power, Complex_t *impedance)
{
}

/* Utilities ---------------------------------------------------------------*/
inline float getDuty(float normalizedPower)
{
    return (fastacos(1.0f - 2.0f * normalizedPower) / M_2PI);
}

inline void ema(float *param, float value, float cfilt)
{
    (*param) = value * cfilt + (1.0f - cfilt) * (*param);
}

inline void emaComplex(Complex_t *param, Complex_t *value, float cfilt)
{
    param->real = value->real * cfilt + (1.0f - cfilt) * param->real;
    param->img = value->img * cfilt + (1.0f - cfilt) * param->img;
}

inline float clamp(float value, float min, float max)
{
    value = (value > max) ? max : value;
    value = (value < min) ? min : value;

    return value;
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
    denom = 1.0f + x2 * (c + d * x2);

    return (M_PI_2 + num / denom);
}