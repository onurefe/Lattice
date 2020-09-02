#include "lattice.h"
#include "math.h"
#include "matrix.h"
#include "eeprom_emulator.h"
#include "polyreg.h"

/* Private definitions -----------------------------------------------------*/
// Measurement
#define SHUNT_RESISTOR 0.01f
#define VDIV_RATIO (324.0f / 2200000.0f)

// ADC definitions.
#define ADC_CLOCK_FREQ MCU_CLOCK_FREQ
#define ADC_SAMPLING_CYLES 19.5f
#define ADC_DEADTIME 2.0f
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
#define SIGNAL_PROCESSING_VOLTAGE_DIVISOR (AFE_VOLTAGE_GAIN * ((1 << ADC_BITS) / (4.0f * ADC_VREF)) * \
                                           (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT - SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT))
#define SIGNAL_PROCESSING_CURRENT_DIVISOR (AFE_CURRENT_GAIN * ((1 << ADC_BITS) / (4.0f * ADC_VREF)) * \
                                           (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT - SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT))

// Resonance tracking algorithm definitions.
#define RES_TRACKING_WINDOW 250.0f
#define RES_TRACKING_HALF_WINDOW (RES_TRACKING_WINDOW / 2.0f)
#define RES_TRACKING_MEASURE_ON_TRACK_WINDOW 0.20f
#define RES_TRACKING_PI_GAIN 10.0f
#define RES_TRACKING_PI_INTEGRAL_TC 1.0f
#define RES_TRACKING_MEASURE_MIN -1.0f
#define RES_TRACKING_MEASURE_MAX 1.0f

// Power tracking algorithm definitions.
#define POWER_TRACKING_PI_GAIN (1.0f / (2.0f * LATTICE_MAX_POWER))
#define POWER_TRACKING_PI_INTEGRAL_TC 0.1f
#define POWER_TRACKING_PI_KI (POWER_TRACKING_PI_GAIN / POWER_TRACKING_PI_INTEGRAL_TC)

// Phase fix polynomial creating, storing, restoring.
#define CALIBRATION_POLYNOMIAL_DEGREE 2U
#define CALIBRATION_VOLTAGE_POLYX_EEID 1U
#define CALIBRATION_VOLTAGE_POLYY_EEID 2U
#define CALIBRATION_CURRENT_POLYX_EEID 3U
#define CALIBRATION_CURRENT_POLYY_EEID 4U
#define CALIBRATION_NUM_OF_SAMPLES 100U

// Anchor frequency finding.
#define STABILIZATION_CYCLES 2U
#define SEARCH_NUM_OF_MEASUREMENTS 2000U

#if (SAMPLES_PER_CYCLE != 12)
#error "Implementation is not compatible with different sample count than 12."
#endif

// Costants.
#define COS_M_PI_12 0.9659258262890683f
#define SINE_M_PI_12 0.25881904510252074f

/* Private types -----------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------*/
static inline void control(uint16_t *adcBuffer);
static inline void calibrator(float currentX, float currentY,
                              float voltageX, float voltageY);
static inline void anchorSearcher(float currentX, float currentY,
                                  float voltageX, float voltageY);
static inline void tracker(float currentX, float currentY,
                           float voltageX, float voltageY);
static inline void getPhasors(uint16_t *bf, float *currentX, float *currentY,
                              float *voltageX, float *voltageY);
static inline float getDuty(float normalizedPower);
static inline float polyCalc(float x, float *coeff);
static inline float fastacos(float x);
static inline float fastsin(float x);
static inline float fastcos(float x);
static inline void updateHrtimRegisters(void);

/* Imported variables ------------------------------------------------------*/
extern HRTIM_HandleTypeDef hhrtim1;
extern ADC_HandleTypeDef hadc1;

/* Private variables -------------------------------------------------------*/
// Runtime variables------------------.
static Lattice_Status_t LastStatus;
static volatile Lattice_Status_t Status;

// Variables used in searching mode.
static volatile float AnchorFrequency;
static volatile float AnchorRealPower;
static volatile float AnchorImgPower;
static volatile uint8_t StabilizationIteration;
static volatile uint16_t SearchIteration;
static volatile Bool_t SearchCompleted;

// Variables used in tracking mode.
static volatile float PowerTrackingIntegral;
static volatile float ResTrackingIntegral;
static volatile float ResTrackingPiLower;
static volatile float ResTrackingPiUpper;

// Variables used in common.
static volatile float PwmFrequency;
static volatile float PwmDuty;
static volatile float DestinationOutputPower;

// Variables used when calibrating the device.
static float CalibrationVoltageFixPolyCoeffsX[CALIBRATION_POLYNOMIAL_DEGREE + 1];
static float CalibrationVoltageFixPolyCoeffsY[CALIBRATION_POLYNOMIAL_DEGREE + 1];
static float CalibrationCurrentFixPolyCoeffsX[CALIBRATION_POLYNOMIAL_DEGREE + 1];
static float CalibrationCurrentFixPolyCoeffsY[CALIBRATION_POLYNOMIAL_DEGREE + 1];
static float CalibrationVoltageSamplesX[CALIBRATION_NUM_OF_SAMPLES];
static float CalibrationVoltageSamplesY[CALIBRATION_NUM_OF_SAMPLES];
static float CalibrationCurrentSamplesX[CALIBRATION_NUM_OF_SAMPLES];
static float CalibrationCurrentSamplesY[CALIBRATION_NUM_OF_SAMPLES];
static volatile uint16_t CalibrationIteration;
static volatile Bool_t CalibrationCompleted;

// Buffers---------------------------.
static uint16_t AdcBuffer[ADC_DOUBLE_BUFFER_ELEMENT_COUNT];

/* Exported functions ------------------------------------------------------*/
void Lattice_Start(void)
{
    // Restore calibration data.
    Bool_t rvx, rvy, rix, riy;
    rvx = EepromEmulator_ReadObject(CALIBRATION_VOLTAGE_POLYX_EEID, 0,
                                    sizeof(CalibrationVoltageFixPolyCoeffsX), NULL,
                                    (uint8_t *)CalibrationVoltageFixPolyCoeffsX);

    rvy = EepromEmulator_ReadObject(CALIBRATION_VOLTAGE_POLYY_EEID, 0,
                                    sizeof(CalibrationVoltageFixPolyCoeffsY), NULL,
                                    (uint8_t *)CalibrationVoltageFixPolyCoeffsY);

    rix = EepromEmulator_ReadObject(CALIBRATION_CURRENT_POLYX_EEID, 0,
                                    sizeof(CalibrationCurrentFixPolyCoeffsX), NULL,
                                    (uint8_t *)CalibrationCurrentFixPolyCoeffsX);

    riy = EepromEmulator_ReadObject(CALIBRATION_CURRENT_POLYY_EEID, 0,
                                    sizeof(CalibrationCurrentFixPolyCoeffsY), NULL,
                                    (uint8_t *)CalibrationCurrentFixPolyCoeffsY);

    // Initialize run-time variables.
    if (rvx && rvy && rix && riy)
    {
        Status.controlMode = LATTICE_CONTROL_MODE_RES_FREQ_SEARCHING;
        PwmDuty = getDuty(LATTICE_SEARCHING_NORMALIZED_POWER);
    }
    else
    {
        for (uint16_t i = 0; i < CALIBRATION_POLYNOMIAL_DEGREE; i++)
        {
            CalibrationVoltageFixPolyCoeffsX[i] = 0.0f;
            CalibrationVoltageFixPolyCoeffsY[i] = 0.0f;
            CalibrationCurrentFixPolyCoeffsX[i] = 0.0f;
            CalibrationCurrentFixPolyCoeffsY[i] = 0.0f;
        }

        CalibrationVoltageFixPolyCoeffsX[0] = 1.0f;
        CalibrationCurrentFixPolyCoeffsX[0] = 1.0f;

        Status.controlMode = LATTICE_CONTROL_MODE_CALIBRATING;
        PwmDuty = getDuty(LATTICE_CALIBRATION_NORMALIZED_POWER);
    }

    LastStatus.onTrack = FALSE;
    Status.onTrack = FALSE;
    SearchIteration = 0;
    StabilizationIteration = 0;
    CalibrationIteration = 0;
    CalibrationCompleted = FALSE;
    SearchCompleted = FALSE;
    AnchorFrequency = LATTICE_MIN_PWM_FREQ;
    AnchorRealPower = 0.0f;
    AnchorImgPower = 0.0f;
    PwmFrequency = LATTICE_MIN_PWM_FREQ;
    DestinationOutputPower = LATTICE_TRIGGERED_POWER;

    // Start ADC calibration.
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);

    // Start ADC.
    HAL_ADC_Start_DMA(&hadc1, ((uint32_t *)AdcBuffer), ADC_DOUBLE_BUFFER_ELEMENT_COUNT);

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
        (LastStatus.onTrack != Status.onTrack))
    {
        LastStatus.controlMode = Status.controlMode;
        LastStatus.onTrack = Status.onTrack;

        Lattice_StatusChangeCb(&LastStatus);
    }

    // Calibration completed. Calculate fix polynomial coefficients and save the result
    //to flash memory.
    if (CalibrationCompleted)
    {
        Lattice_Stop();

        // Calculate fit polynomial coefficients.
        Polyreg_Fit1(CalibrationVoltageSamplesX, CalibrationVoltageFixPolyCoeffsX);
        Polyreg_Fit1(CalibrationVoltageSamplesY, CalibrationVoltageFixPolyCoeffsY);
        Polyreg_Fit1(CalibrationCurrentSamplesX, CalibrationCurrentFixPolyCoeffsX);
        Polyreg_Fit1(CalibrationCurrentSamplesY, CalibrationCurrentFixPolyCoeffsY);

        // Save them to flash.
        EepromEmulator_WriteObject(CALIBRATION_VOLTAGE_POLYX_EEID,
                                   sizeof(CalibrationVoltageFixPolyCoeffsX),
                                   ((uint8_t *)CalibrationVoltageFixPolyCoeffsX));

        EepromEmulator_WriteObject(CALIBRATION_VOLTAGE_POLYY_EEID,
                                   sizeof(CalibrationVoltageFixPolyCoeffsY),
                                   ((uint8_t *)CalibrationVoltageFixPolyCoeffsY));

        EepromEmulator_WriteObject(CALIBRATION_CURRENT_POLYX_EEID,
                                   sizeof(CalibrationCurrentFixPolyCoeffsX),
                                   ((uint8_t *)CalibrationCurrentFixPolyCoeffsX));

        EepromEmulator_WriteObject(CALIBRATION_CURRENT_POLYY_EEID,
                                   sizeof(CalibrationCurrentFixPolyCoeffsY),
                                   ((uint8_t *)CalibrationCurrentFixPolyCoeffsY));

        // Invoke callback.
        Lattice_CalibrationCompletedCb();

        CalibrationCompleted = FALSE;
    }

    // Search completed. Calculate parallel capacitor polynomial.
    if (SearchCompleted)
    {
        // Initialize resonance tracking algorithm variables.
        PowerTrackingIntegral = 0.0f;
        ResTrackingIntegral = 0.0f;
        PwmFrequency = AnchorFrequency;
        PwmDuty = getDuty(LATTICE_TRACKING_STARTUP_NORMALIZED_POWER);
        ResTrackingPiUpper = LATTICE_MAX_PWM_FREQ < (AnchorFrequency + RES_TRACKING_HALF_WINDOW)
                                 ? LATTICE_MAX_PWM_FREQ
                                 : (AnchorFrequency + RES_TRACKING_HALF_WINDOW);
        ResTrackingPiLower = LATTICE_MIN_PWM_FREQ > (AnchorFrequency - RES_TRACKING_HALF_WINDOW)
                                 ? LATTICE_MIN_PWM_FREQ
                                 : (AnchorFrequency - RES_TRACKING_HALF_WINDOW);

        ResTrackingPiUpper -= AnchorFrequency;
        ResTrackingPiLower -= AnchorFrequency;

        SearchCompleted = FALSE;
        Status.controlMode = LATTICE_CONTROL_MODE_RES_FREQ_TRACKING;
    }
}

void Lattice_Stop(void)
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

void Lattice_CmdPowerOutput(Bool_t status)
{
    Status.powerOutput = status;
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

/* Callbacks ---------------------------------------------------------------*/
__weak void Lattice_StatusChangeCb(Lattice_Status_t *status)
{
}

__weak void Lattice_CalibrationCompletedCb(void)
{
}

/* Private functions -------------------------------------------------------*/
inline void control(uint16_t *adcBuffer)
{
    float current_x, current_y, voltage_x, voltage_y;

    // Process data in the adc buffer and obtain current and voltage in phasor form.
    getPhasors(adcBuffer, &current_x, &current_y, &voltage_x, &voltage_y);

    if (Status.controlMode == LATTICE_CONTROL_MODE_CALIBRATING)
    {
        calibrator(current_x, current_y, voltage_x, voltage_y);
        updateHrtimRegisters();
    }
    else if (Status.controlMode == LATTICE_CONTROL_MODE_RES_FREQ_TRACKING)
    {
        tracker(current_x, current_y, voltage_x, voltage_y);
        updateHrtimRegisters();
    }
    else if (Status.controlMode == LATTICE_CONTROL_MODE_RES_FREQ_SEARCHING)
    {
        anchorSearcher(current_x, current_y, voltage_x, voltage_y);
        updateHrtimRegisters();
    }
}

inline void calibrator(float currentX, float currentY, float voltageX, float voltageY)
{
    if (CalibrationIteration < CALIBRATION_NUM_OF_SAMPLES)
    {
        float inorm, vnorm;
        inorm = sqrtf(currentX * currentX + currentY * currentY);
        vnorm = sqrtf(voltageX * voltageX + voltageY * voltageY);

        // Store calibration data to RAM.
        CalibrationCurrentSamplesX[CalibrationIteration] = currentX / inorm;
        CalibrationCurrentSamplesY[CalibrationIteration] = currentY / inorm;
        CalibrationVoltageSamplesX[CalibrationIteration] = voltageX / vnorm;
        CalibrationVoltageSamplesY[CalibrationIteration] = voltageY / vnorm;

        CalibrationIteration++;
        PwmFrequency = LATTICE_MIN_PWM_FREQ + ((LATTICE_MAX_PWM_FREQ - LATTICE_MIN_PWM_FREQ) * CalibrationIteration) /
                                                  CALIBRATION_NUM_OF_SAMPLES;
    }
    else
    {
        if (!CalibrationCompleted)
        {
            CalibrationCompleted = TRUE;
        }
    }
}

inline void anchorSearcher(float currentX, float currentY, float voltageX, float voltageY)
{
    if (StabilizationIteration < STABILIZATION_CYCLES)
    {
        StabilizationIteration++;
        return;
    }

    StabilizationIteration = 0;

    if (SearchIteration <= SEARCH_NUM_OF_MEASUREMENTS)
    {
        if (SearchIteration == SEARCH_NUM_OF_MEASUREMENTS)
        {
            PwmFrequency = AnchorFrequency;
        }
        else
        {
            float real_power, img_power;
            real_power = 0.5f * (voltageX * currentX + voltageY * currentY);
            img_power = 0.5f * (voltageY * currentX - voltageX * currentY);

            // Update the maxima.
            if (real_power > AnchorRealPower)
            {
                AnchorFrequency = PwmFrequency;
                AnchorRealPower = real_power;
                AnchorImgPower = img_power;
            }

            // Update PWM frequency.
            PwmFrequency = LATTICE_MIN_PWM_FREQ +
                           SearchIteration * ((LATTICE_MAX_PWM_FREQ - LATTICE_MIN_PWM_FREQ) / SEARCH_NUM_OF_MEASUREMENTS);
        }

        SearchIteration++;
    }
    else
    {
        SearchCompleted = TRUE;
    }
}

inline void tracker(float currentX, float currentY, float voltageX, float voltageY)
{
    float error;
    float power_real;
    float power_img;

    power_real = 0.5f * (voltageX * currentX + voltageY * currentY);
    power_img = 0.5f * (voltageY * currentX - voltageX * currentY);

    if (Status.powerOutput)
    {
        error = LATTICE_TRIGGERED_POWER - power_real;
    }
    else
    {
        error = LATTICE_NONTRIGGERED_POWER - power_real;
    }

    // Execute PID controller to control duty cycle.
    {
        float pi_out;
        pi_out = (error + PowerTrackingIntegral / POWER_TRACKING_PI_INTEGRAL_TC) * POWER_TRACKING_PI_GAIN;
        Bool_t windup_protect = FALSE;

        if (pi_out > 1.0f)
        {
            pi_out = 1.0f;
            windup_protect = (error >= 0.0f);
        }

        if (pi_out < 0.0f)
        {
            pi_out = 0.0f;
            windup_protect = (windup_protect || (error <= 0.0f));
        }

        // If no windup occurred; update the integral term.
        if (!windup_protect)
        {
            PowerTrackingIntegral += (PWM_CYCLES_PER_CONTROL_CYCLE / PwmFrequency) * error;
        }

        // Update PWM duty.
        PwmDuty = getDuty(pi_out);
    }

    // Use tracking measure as an input to frequency controlling PID controller.
    {
        float pi_in;
        float pi_out;

        pi_in = (power_img / power_real);
        pi_in = (pi_in > RES_TRACKING_MEASURE_MAX) ? RES_TRACKING_MEASURE_MAX : pi_in;
        pi_in = (pi_in < RES_TRACKING_MEASURE_MIN) ? RES_TRACKING_MEASURE_MIN : pi_in;

        // Determine if on track.
        Status.onTrack = (fabsf(pi_in) < RES_TRACKING_MEASURE_ON_TRACK_WINDOW);

        // Calculate PI controller output.
        pi_out = (pi_in + ResTrackingIntegral / RES_TRACKING_PI_INTEGRAL_TC) * RES_TRACKING_PI_GAIN;

        Bool_t windup_protect = FALSE;
        if (pi_out > ResTrackingPiUpper)
        {
            pi_out = ResTrackingPiUpper;
            windup_protect = (pi_in >= 0.0f);
        }

        if (pi_out < ResTrackingPiLower)
        {
            pi_out = ResTrackingPiLower;
            windup_protect = (windup_protect || (pi_in <= 0.0f));
        }

        // If no windup occurred; update the integral term.
        if (!windup_protect)
        {
            ResTrackingIntegral += (PWM_CYCLES_PER_CONTROL_CYCLE / PwmFrequency) * pi_in;
        }

        PwmFrequency = AnchorFrequency + pi_out;
    }
}

inline void getPhasors(uint16_t *bf, float *currentX, float *currentY,
                       float *voltageX, float *voltageY)
{
    float isx = 0.0f;
    float isy = 0.0f;
    float vsx = 0.0f;
    float vsy = 0.0f;
    float sumix, sumiy, sumvx, sumvy;
    uint16_t off = SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT * ADC_CHANNELS;

    // Process all the elements.
    while (off < (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT * ADC_CHANNELS))
    {
        sumix = bf[off];
        off++;
        sumvx = bf[off];
        off++;

        sumix += 0.8660254037844386f * bf[off];
        sumiy = 0.5f * bf[off];
        off++;
        sumvx += 0.8660254037844386f * bf[off];
        sumvy = 0.5f * bf[off];
        off++;

        sumix += 0.5f * bf[off];
        sumiy += 0.8660254037844386f * bf[off];
        off++;
        sumvx += 0.5f * bf[off];
        sumvy += 0.8660254037844386f * bf[off];
        off++;

        sumiy += bf[off];
        off++;
        sumvy += bf[off];
        off++;

        sumix -= 0.5f * bf[off];
        sumiy += 0.8660254037844386f * bf[off];
        off++;
        sumvx -= 0.5f * bf[off];
        sumvy += 0.8660254037844386f * bf[off];
        off++;

        sumix -= 0.8660254037844386f * bf[off];
        sumiy += 0.5f * bf[off];
        off++;
        sumvx -= 0.8660254037844386f * bf[off];
        sumvy += 0.5f * bf[off];
        off++;

        sumix -= bf[off];
        off++;
        sumvx -= bf[off];
        off++;

        sumix -= 0.8660254037844386f * bf[off];
        sumiy -= 0.5f * bf[off];
        off++;
        sumvx -= 0.8660254037844386f * bf[off];
        sumvy -= 0.5f * bf[off];
        off++;

        sumix -= 0.5f * bf[off];
        sumiy -= 0.8660254037844386f * bf[off];
        off++;
        sumvx -= 0.5f * bf[off];
        sumvy -= 0.8660254037844386f * bf[off];
        off++;

        sumiy -= bf[off];
        off++;
        sumvy -= bf[off];
        off++;

        sumix += 0.5f * bf[off];
        sumiy -= 0.8660254037844386f * bf[off];
        off++;
        sumvx += 0.5f * bf[off];
        sumvy -= 0.8660254037844386f * bf[off];
        off++;

        sumix += 0.8660254037844386f * bf[off];
        sumiy -= 0.5f * bf[off];
        off++;
        sumvx += 0.8660254037844386f * bf[off];
        sumvy -= 0.5f * bf[off];
        off++;

        isx += sumix;
        isy += sumiy;
        vsx += sumvx;
        vsy += sumvy;
    }

    float temp;

    // Calculate edge aligned PWM related phase error.
    float fixx, fixy, norm;
    fixx = fastsin(M_2PI * PwmDuty);
    fixy = fastcos(M_2PI * PwmDuty) - 1.0f;
    norm = sqrtf(fixx * fixx + fixy * fixy);
    fixx /= norm;
    fixy /= norm;

    // Fix edge aligned PWM related phase error.
    temp = isx * fixx - isy * fixy;
    isy = isx * fixy + isy * fixx;
    isx = temp;
    temp = vsx * fixx - vsy * fixy;
    vsy = vsx * fixy + vsy * fixx;
    vsx = temp;

    // Fix sampling time related phase error.
    temp = isx * COS_M_PI_12 - isy * SINE_M_PI_12;
    isy = isx * SINE_M_PI_12 + isy * COS_M_PI_12;
    isx = temp;
    temp = vsx * COS_M_PI_12 - vsy * SINE_M_PI_12;
    vsy = vsx * SINE_M_PI_12 + vsy * COS_M_PI_12;
    vsx = temp;

    // Parse analog circuitry related phase shift fixing phasors.
    float in, vfixx, vfixy, ifixx, ifixy;
    in = PwmFrequency / 1e4f;
    ifixx = polyCalc(in, CalibrationCurrentFixPolyCoeffsX);
    ifixy = -polyCalc(in, CalibrationCurrentFixPolyCoeffsY);
    vfixx = polyCalc(in, CalibrationVoltageFixPolyCoeffsX);
    vfixy = -polyCalc(in, CalibrationVoltageFixPolyCoeffsY);

    // Correct circuitry related phase shift.
    temp = isx * ifixx - isy * ifixy;
    isy = isx * ifixy + isy * ifixx;
    isx = temp;
    temp = vsx * vfixx - vsy * vfixy;
    vsy = vsx * vfixy + vsy * vfixx;
    vsx = temp;

    // Calculate voltage and current phasors.
    *currentX = isx / SIGNAL_PROCESSING_CURRENT_DIVISOR;
    *currentY = isy / SIGNAL_PROCESSING_CURRENT_DIVISOR;
    *voltageX = vsx / SIGNAL_PROCESSING_VOLTAGE_DIVISOR;
    *voltageY = vsy / SIGNAL_PROCESSING_VOLTAGE_DIVISOR;
}

inline float getDuty(float normalizedPower)
{
    return (fastacos(1.0f - 2.0f * normalizedPower) / M_2PI);
}

inline float polyCalc(float x, float *coeff)
{
    return (coeff[0] + x * (coeff[1] + x * (coeff[2])));
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

inline float fastsin(float x)
{
    x /= M_2PI;
    x -= (int)x;

    if (x >= 0.0f)
    {
        return (64.0f * x * (0.5f - x) / (5 - 16 * x * (0.5f - x)));
    }
    else
    {
        x = -x;
        return (-64.0f * x * (0.5f - x) / (5 - 16 * x * (0.5f - x)));
    }
}

inline float fastcos(float x)
{
    return fastsin(x + 0.5f * M_PI);
}

inline void updateHrtimRegisters(void)
{
    // Update TimerA registers.
    uint16_t period;
    uint16_t comp;
    period = (uint16_t)((HRTIM_CLOCK_FREQ / PwmFrequency) + 0.5f);
    comp = (uint16_t)((HRTIM_CLOCK_FREQ * PwmDuty / PwmFrequency) + 0.5f);

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