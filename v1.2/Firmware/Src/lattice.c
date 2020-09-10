#include "lattice.h"
#include "eeprom_emulator.h"
#include "eeprom_ids.h"
#include "polyreg.h"
#include "cli.h"

/* Private typedef ---------------------------------------------------------*/
typedef struct
{
    Complex_t calibrationPoly[(CALIBRATION_POLY_DEGREE + 1)];
} Calibration_t;

typedef struct
{
    float maxPower;
    float minLoading;
    float maxLoading;
} Constraints_t;

typedef struct
{
    uint32_t deviceId;
    uint16_t versionMajor;
    uint16_t versionMinor;
} DeviceInfo_t;

typedef struct
{
    float normalizedPower;
    uint32_t steps;
} SearchingParams_t;

typedef struct
{
    float minHornImpedance;
    float maxHornImpedance;
    float powerTrackingTolerance;
    float frequencyTrackingTolerance;
    float timeout;
} ErrorDetectionParams_t;

typedef struct
{
    Bool_t calibration : 1;
    Bool_t constraints : 1;
    Bool_t deviceInfo : 1;
    Bool_t searchingParams : 1;
    Bool_t errorDetectionParams : 1;
    Bool_t powerTrackingPidParams : 1;
    Bool_t frequencyTrackingPidParams : 1;
    Bool_t monitoringPeriod : 1;
    Bool_t trackingDestinationPower : 1;
} FactoryFlags_t;

enum
{
    EVENT_CALIBRATE = (1 << 0),
    EVENT_DESTINATION_POWER_SET = (1 << 1),
    EVENT_SCAN_COMPLETED = (1 << 2),
    EVENT_SEARCH_COMPLETED = (1 << 3),
    EVENT_FACTORY_MODE = (1 << 4),
    EVENT_ERROR_OCCURRED = (1 << 5)
};
typedef uint8_t Events_t;

/* Private variables -------------------------------------------------------*/
// State machine variables.
static Lattice_Status_t Status = LATTICE_STATUS_FACTORY_CONFIG;
static Lattice_Error_t Error;
static Events_t Events;
static FactoryFlags_t Flags;

// Factory configured data.
static Calibration_t Calibration;
static Constraints_t Constraints;
static DeviceInfo_t DeviceInfo;
static SearchingParams_t SearchingParams;
static ErrorDetectionParams_t ErrorDetectionParams;
static Pid_Params_t PowerTrackingPidParams;
static Pid_Params_t FrequencyTrackingPidParams;
static float MonitoringPeriod;
static float TrackingDestinationPower; // User can overwrite the destination power.

// Runtime variables.
static float ResonanceFrequency;
static float ResonanceImpedance;
static float FullPower;
static Bool_t ReportingEnabled;
static Bool_t PostponedDestPowerEepromWrite;
static uint8_t HornImpedanceOutofWindowsSuccessiveErrors;
static uint8_t PowerTrackingSuccessiveErrors;
static uint8_t FrequencyTrackingSuccessiveErrors;

// Impedance scanning store.
static float ScanCalibrationResistance;
static float ScanImpedanceReal[CALIBRATION_NUM_OF_SAMPLES];
static float ScanImpedanceImg[CALIBRATION_NUM_OF_SAMPLES];

/* Private function prototypes ---------------------------------------------*/
int32_t hash(uint32_t deviceId, uint16_t versionMajor, uint16_t versionMinor);
void calculateCalibrationPolynomials(float *scanImpReal, float *scanImpImg,
                                     float calibrationResistance, Complex_t *coeffs);
void startTracking(void);

/* Exported functions ------------------------------------------------------*/
void Lattice_Start(void)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return;
    }

    Error = LATTICE_ERROR_NONE;
    ReportingEnabled = FALSE;
    PostponedDestPowerEepromWrite = FALSE;

    // Load burned data.
    Flags.calibration = EepromEmulator_ReadObject(LATTICE_CALIBRATION_POLY_EEID, 0,
                                                  sizeof(Calibration),
                                                  NULL, &Calibration);
    Flags.constraints = EepromEmulator_ReadObject(LATTICE_CONSTRAINTS_EEID, 0,
                                                  sizeof(Constraints), NULL, &Constraints);
    Flags.deviceInfo = EepromEmulator_ReadObject(LATTICE_DEVICE_INFO_EEID, 0,
                                                 sizeof(DeviceInfo), NULL, &DeviceInfo);
    Flags.searchingParams = EepromEmulator_ReadObject(LATTICE_SEARCHING_PARAMS_EEID, 0,
                                                      sizeof(SearchingParams), NULL,
                                                      &SearchingParams);
    Flags.errorDetectionParams = EepromEmulator_ReadObject(LATTICE_ERROR_DETECTION_PARAMS_EEID, 0,
                                                           sizeof(ErrorDetectionParams), NULL,
                                                           &ErrorDetectionParams);
    Flags.powerTrackingPidParams = EepromEmulator_ReadObject(LATTICE_POWER_TRACKING_PID_PARAMS_EEID, 0,
                                                             sizeof(PowerTrackingPidParams), NULL,
                                                             &PowerTrackingPidParams);
    Flags.frequencyTrackingPidParams = EepromEmulator_ReadObject(LATTICE_FREQUENCY_TRACKING_PID_PARAMS_EEID, 0,
                                                                 sizeof(FrequencyTrackingPidParams), NULL,
                                                                 &FrequencyTrackingPidParams);
    Flags.monitoringPeriod = EepromEmulator_ReadObject(LATTICE_MONITORING_PERIOD_EEID, 0, sizeof(MonitoringPeriod),
                                                       NULL, &MonitoringPeriod);
    Flags.trackingDestinationPower = EepromEmulator_ReadObject(LATTICE_TRACKING_DESTINATION_POWER_EEID, 0,
                                                               sizeof(TrackingDestinationPower), NULL,
                                                               &TrackingDestinationPower);

    // Check if factory configuration completed. If it is, device can operate
    //as configured. If not configured, put device into the factory mode.
    if (Flags.calibration && Flags.constraints && Flags.deviceInfo && Flags.searchingParams &&
        Flags.errorDetectionParams && Flags.powerTrackingPidParams && Flags.frequencyTrackingPidParams &&
        Flags.monitoringPeriod && Flags.trackingDestinationPower)
    {
        Core_Init(&Calibration, LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY, Constraints.minLoading,
                  Constraints.maxLoading);

        // Start searching for the resonance frequency.
        Core_Search(SearchingParams.normalizedPower,
                    LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY, SearchingParams.steps);

        Status = LATTICE_STATUS_OPERATING_SEARCHING;
    }

    Events = 0;
}

void Lattice_Execute(void)
{
    Core_Execute();

    // If error occurred; core is stopped and device put
    //into error mode.
    if (Events & EVENT_ERROR_OCCURRED)
    {
        if ((Status == LATTICE_STATUS_OPERATING_SEARCHING) ||
            (Status == LATTICE_STATUS_OPERATING_TRACKING))
        {
            Core_Stop();

            Status = LATTICE_STATUS_ERROR;
        }

        Events ^= EVENT_ERROR_OCCURRED;
    }

    // If factory mode event occurred.
    if (Events & EVENT_FACTORY_MODE)
    {
        // If core is operating stop the core.
        if ((Status == LATTICE_STATUS_OPERATING_SEARCHING) ||
            (Status == LATTICE_STATUS_OPERATING_TRACKING) ||
            (Status == LATTICE_STATUS_FACTORY_CONFIG_CALIBRATING))
        {
            Core_Stop();
        }

        // Put device to pending factory initialization.
        Status = LATTICE_STATUS_FACTORY_CONFIG;
        Events ^= EVENT_FACTORY_MODE;
    }

    // If destination power is set.
    if (Events & EVENT_DESTINATION_POWER_SET)
    {
        // If the device is at tracking state; writing to the
        //eeprom emulator should interrupt the process.
        if (Status == LATTICE_STATUS_OPERATING_TRACKING)
        {
            // Stop tracking.
            Core_Stop();

            EepromEmulator_WriteObject(LATTICE_TRACKING_DESTINATION_POWER_EEID,
                                       sizeof(TrackingDestinationPower),
                                       &TrackingDestinationPower);

            // Start tracking.
            startTracking();
        }

        // Postpone tracking power eeprom write operation.
        if (Status == LATTICE_STATUS_OPERATING_SEARCHING)
        {
            PostponedDestPowerEepromWrite = TRUE;
        }

        Events ^= EVENT_DESTINATION_POWER_SET;
    }

    // Handle search completed event.
    if (Events & EVENT_SEARCH_COMPLETED)
    {
        if (Status == LATTICE_STATUS_OPERATING_SEARCHING)
        {
            // Update tracking destination power if postponed.
            if (PostponedDestPowerEepromWrite)
            {
                EepromEmulator_WriteObject(LATTICE_TRACKING_DESTINATION_POWER_EEID,
                                           sizeof(TrackingDestinationPower),
                                           &TrackingDestinationPower);

                PostponedDestPowerEepromWrite = FALSE;
            }

            // Start tracking.
            startTracking();

            Status = LATTICE_STATUS_OPERATING_TRACKING;
        }

        Events ^= EVENT_SEARCH_COMPLETED;
    }

    // Handle calibration command event.
    if (Events & EVENT_CALIBRATE)
    {
        if (Status == LATTICE_STATUS_FACTORY_CONFIG)
        {
            Core_Init(NULL, LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY, 0.0f,
                      1.0f);

            // Start scanning.
            Core_Scan(LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY,
                      CALIBRATION_NUM_OF_SAMPLES, ScanImpedanceReal, ScanImpedanceImg);

            Status = LATTICE_STATUS_FACTORY_CONFIG_CALIBRATING;
        }

        Events ^= EVENT_CALIBRATE;
    }

    // Handle scan completed event.
    if (Events & EVENT_SCAN_COMPLETED)
    {
        if (Status == LATTICE_STATUS_FACTORY_CONFIG_CALIBRATING)
        {
            float coeffs_real[CALIBRATION_POLY_DEGREE + 1];
            float coeffs_img[CALIBRATION_POLY_DEGREE + 1];
            Complex_t coeffs[CALIBRATION_POLY_DEGREE + 1];

            calculateCalibrationPolynomials(ScanImpedanceReal, ScanImpedanceImg,
                                            ScanCalibrationResistance, coeffs);

           // Record object.
            EepromEmulator_WriteObject(LATTICE_CALIBRATION_POLY_EEID, sizeof(Calibration_t),
                                       (uint8_t *)&Calibration);

            Flags.calibration = TRUE;

            // Return to factory parameter setup.
            Status = LATTICE_STATUS_FACTORY_CONFIG;
        }

        // Clear scan completed event.
        Events ^= EVENT_SCAN_COMPLETED;
    }
}

// Functions to manipulate factory data.
Bool_t Lattice_SetConstraints(float maxPower, float minLoading, float maxLoading)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    if ((minLoading >= 0.0f) && (maxLoading <= 1.0f))
    {
        Constraints.maxPower = maxPower;
        Constraints.minLoading = minLoading;
        Constraints.maxLoading = maxLoading;

        EepromEmulator_WriteObject(LATTICE_CONSTRAINTS_EEID, sizeof(Constraints),
                                   &Constraints);
        Flags.constraints = TRUE;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_GetConstraints(float *maxPower, float *minLoading, float *maxLoading)
{
    if (Flags.constraints)
    {
        *maxPower = Constraints.maxPower;
        *minLoading = Constraints.minLoading;
        *maxLoading = Constraints.maxLoading;

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_SetDeviceInfo(uint32_t deviceId, uint16_t versionMajor, uint16_t versionMinor)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    DeviceInfo.deviceId = deviceId;
    DeviceInfo.versionMajor = versionMajor;
    DeviceInfo.versionMinor = versionMinor;

    EepromEmulator_WriteObject(LATTICE_DEVICE_INFO_EEID, sizeof(DeviceInfo_t),
                               &DeviceInfo);
    Flags.deviceInfo = TRUE;

    return TRUE;
}

Bool_t Lattice_GetDeviceInfo(uint32_t *deviceId, uint16_t *versionMajor, uint16_t *versionMinor)
{
    if (Flags.deviceInfo)
    {
        *deviceId = DeviceInfo.deviceId;
        *versionMajor = DeviceInfo.versionMajor;
        *versionMinor = DeviceInfo.versionMinor;

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_SetSearchingParams(float normalizedPower, uint16_t steps)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    if ((0.0f <= normalizedPower) && (normalizedPower <= 1.0f))
    {
        SearchingParams.normalizedPower = normalizedPower;
        SearchingParams.steps = steps;

        EepromEmulator_WriteObject(LATTICE_SEARCHING_PARAMS_EEID,
                                   sizeof(SearchingParams_t), &SearchingParams);

        Flags.searchingParams = TRUE;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_GetSearchingParams(float *normalizedPower, uint16_t *steps)
{
    if (Flags.searchingParams)
    {
        *normalizedPower = SearchingParams.normalizedPower;
        *steps = SearchingParams.steps;

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_SetErrorDetectionParams(float minHornImpedance, float maxHornImpedance,
                                       float powerTrackingTolerance, float frequencyTrackingTolerance,
                                       float timeout)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    // Update error detection parameters.
    if (((0.0f < minHornImpedance) && (minHornImpedance < maxHornImpedance)) &&
        (0.0f < powerTrackingTolerance) && (0.0f < frequencyTrackingTolerance) && (0.0f < timeout))
    {
        ErrorDetectionParams.minHornImpedance = minHornImpedance;
        ErrorDetectionParams.maxHornImpedance = maxHornImpedance;
        ErrorDetectionParams.powerTrackingTolerance = powerTrackingTolerance;
        ErrorDetectionParams.frequencyTrackingTolerance = frequencyTrackingTolerance;
        ErrorDetectionParams.timeout = timeout;

        Flags.errorDetectionParams = TRUE;

        EepromEmulator_WriteObject(LATTICE_ERROR_DETECTION_PARAMS_EEID, sizeof(ErrorDetectionParams),
                                   &ErrorDetectionParams);

        return TRUE;
    }
    else
    {
        return FALSE;
    }

    return TRUE;
}

Bool_t Lattice_GetErrorDetectionParams(float *minHornImpedance, float *maxHornImpedance,
                                       float *powerTrackingTolerance, float *frequencyTrackingTolerance,
                                       float *timeout)
{
    if (!Flags.errorDetectionParams)
    {
        return FALSE;
    }

    *minHornImpedance = ErrorDetectionParams.minHornImpedance;
    *maxHornImpedance = ErrorDetectionParams.maxHornImpedance;
    *powerTrackingTolerance = ErrorDetectionParams.powerTrackingTolerance;
    *frequencyTrackingTolerance = ErrorDetectionParams.frequencyTrackingTolerance;
    *timeout = ErrorDetectionParams.timeout;

    return TRUE;
}

Bool_t Lattice_SetMonitoringPeriod(float period)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    if (period > 0.0f)
    {
        MonitoringPeriod = period;
        Flags.monitoringPeriod = TRUE;

        EepromEmulator_WriteObject(LATTICE_MONITORING_PERIOD_EEID, sizeof(MonitoringPeriod),
                                   &MonitoringPeriod);
        return TRUE;
    }

    return FALSE;
}

Bool_t Lattice_GetMonitoringPeriod(float *period)
{
    if (!Flags.monitoringPeriod)
    {
        return FALSE;
    }

    *period = MonitoringPeriod;

    return TRUE;
}

Bool_t Lattice_SetPowerTrackingPidCoeffs(float kp, float ki, float kd, float tf)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    if ((kp >= 0.0f) && (ki >= 0.0f) && (kd >= 0.0f) && (tf > 0.0f))
    {
        PowerTrackingPidParams.kp = kp;
        PowerTrackingPidParams.ki = ki;
        PowerTrackingPidParams.kd = kd;
        PowerTrackingPidParams.tfilt = tf;

        Flags.powerTrackingPidParams = TRUE;

        EepromEmulator_WriteObject(LATTICE_POWER_TRACKING_PID_PARAMS_EEID,
                                   sizeof(Pid_Params_t),
                                   &PowerTrackingPidParams);

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_GetPowerTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf)
{
    if (Flags.powerTrackingPidParams)
    {
        *kp = PowerTrackingPidParams.kp;
        *ki = PowerTrackingPidParams.ki;
        *kd = PowerTrackingPidParams.kd;
        *tf = PowerTrackingPidParams.tfilt;

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_SetFrequencyTrackingPidCoeffs(float kp, float ki, float kd, float tf)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    if ((kp >= 0.0f) && (ki >= 0.0f) && (kd >= 0.0f) && (tf > 0.0f))
    {
        FrequencyTrackingPidParams.kp = kp;
        FrequencyTrackingPidParams.ki = ki;
        FrequencyTrackingPidParams.kd = kd;
        FrequencyTrackingPidParams.tfilt = tf;

        Flags.frequencyTrackingPidParams = TRUE;

        EepromEmulator_WriteObject(LATTICE_FREQUENCY_TRACKING_PID_PARAMS_EEID,
                                   sizeof(Pid_Params_t),
                                   &FrequencyTrackingPidParams);

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_GetFrequencyTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf)
{
    if (Flags.frequencyTrackingPidParams)
    {
        *kp = FrequencyTrackingPidParams.kp;
        *ki = FrequencyTrackingPidParams.ki;
        *kd = FrequencyTrackingPidParams.kd;
        *tf = FrequencyTrackingPidParams.tfilt;

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

Bool_t Lattice_FactoryMode(int32_t password)
{
    // If device info is not set; the device is already authorized.
    if (!Flags.deviceInfo)
    {
        return TRUE;
    }

    // Check password.
    if (password != hash(DeviceInfo.deviceId, DeviceInfo.versionMajor,
                         DeviceInfo.versionMinor))
    {
        return FALSE;
    }

    // If not in factory mode, trigger event.
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        Events |= EVENT_FACTORY_MODE;
    }

    return TRUE;
}

Bool_t Lattice_Calibrate(void)
{
    if (Status != LATTICE_STATUS_FACTORY_CONFIG)
    {
        return FALSE;
    }

    Events |= EVENT_CALIBRATE;

    return TRUE;
}

void Lattice_Reset(void)
{
    // Reset the system.
    HAL_NVIC_SystemReset();
}

Bool_t Lattice_SetDestinationPower(float destinationPower)
{
    if (!((Status == LATTICE_STATUS_FACTORY_CONFIG) ||
          (Status == LATTICE_STATUS_OPERATING_SEARCHING) ||
          (Status == LATTICE_STATUS_OPERATING_TRACKING)))
    {
        return FALSE;
    }

    if (0.0f < destinationPower)
    {
        TrackingDestinationPower = destinationPower;
        Flags.trackingDestinationPower = TRUE;

        // If the device is at factory mode, there aren't any
        //problem of writing to the eeprom emulator.
        if (Status == LATTICE_STATUS_FACTORY_CONFIG)
        {
            EepromEmulator_WriteObject(LATTICE_TRACKING_DESTINATION_POWER_EEID,
                                       sizeof(TrackingDestinationPower),
                                       &TrackingDestinationPower);
        }
        else
        {
            Events |= EVENT_DESTINATION_POWER_SET;    
        }
        
        return TRUE;
    }
    else
    {
        return FALSE;
    }

    return TRUE;
}

Bool_t Lattice_Report(Bool_t isOn)
{
    if ((Status != LATTICE_STATUS_OPERATING_SEARCHING) &&
        (Status != LATTICE_STATUS_OPERATING_TRACKING))
    {
        return FALSE;
    }

    ReportingEnabled = isOn;
    return TRUE;
}

Lattice_Status_t Lattice_GetStatus(void)
{
    return Status;
}

Lattice_Error_t Lattice_GetError(void)
{
    return Error;
}

/* Private functions -------------------------------------------------------*/
void startTracking(void)
{
    // Start tracking.
    Core_Track(&PowerTrackingPidParams, &FrequencyTrackingPidParams, ResonanceFrequency,
               FullPower, TrackingDestinationPower);

    // Clear error counters.
    HornImpedanceOutofWindowsSuccessiveErrors = 0;
    PowerTrackingSuccessiveErrors = 0;
    FrequencyTrackingSuccessiveErrors = 0;

    // Enable periodic measurements if set.
    Core_PeriodicMeasurements(TRUE, MonitoringPeriod);
}

int32_t hash(uint32_t deviceId, uint16_t versionMajor, uint16_t versionMinor)
{
    uint64_t tmp;
    tmp = (deviceId << 32) | (versionMajor << 16) | versionMinor;

    return ((tmp + LATTICE_HASH_OFFSET) % LATTICE_HASH_DIVISOR);
}

void calculateCalibrationPolynomials(float *scanImpReal, float *scanImpImg,
                                     float calibrationResistance, Complex_t *coeffs)
{
    // Obtain voltage fixing complex phasor data. Since the analog circuitry
    //is weak on the voltage measurement side; it's assumed that all the error
    //is due to voltage measuring circuitry.
    for (uint16_t i = 0; i < CALIBRATION_NUM_OF_SAMPLES; i++)
    {
        float abs_sqr;
        abs_sqr = scanImpReal[i] * scanImpReal[i] +
                  scanImpImg[i] * scanImpImg[i];

        scanImpReal[i] = calibrationResistance * scanImpReal[i] /
                         abs_sqr;
        scanImpImg[i] = -calibrationResistance * scanImpImg[i] /
                        abs_sqr;
    }

    float coeffs_real[3], coeffs_img[3];

    // Obtain calibration polynomials(for real and imaginary parts).
    Polyreg_Fit(scanImpReal, coeffs_real);
    Polyreg_Fit(scanImpImg, coeffs_img);

    // Set calibration polynomial data.
    for (uint8_t i = 0; i < (CALIBRATION_POLY_DEGREE + 1); i++)
    {
        coeffs[i].real = coeffs_real[i];
        coeffs[i].img = coeffs_img[i];
    }
}

/* Callback functions ------------------------------------------------------*/
void Core_ScanCompletedCallback(void)
{
    Events |= EVENT_SCAN_COMPLETED;
}

void Core_SearchCompletedCallback(float resonanceFrequency, float resonanceImpedance,
                                  float fullPower)
{
    ResonanceFrequency = resonanceFrequency;
    ResonanceImpedance = resonanceImpedance;
    FullPower = fullPower;

    Events |= EVENT_SEARCH_COMPLETED;
}

void Core_MonitoringCallback(Bool_t triggered, float frequency, float duty,
                             Complex_t *power, Complex_t *impedance)
{
    char buff[128];
    uint8_t length = 0;

    // Check for errors only if triggered.
    if (triggered)
    {
        float tracking_measure, horn_imp;
        tracking_measure = fabsf(power->img / power->real);
        horn_imp = sqrtf(Complex_NormSqr(impedance));

        // Check if horn impedance is acceptable.
        //If not increase successive error counter. If so clear the error counter.
        if ((horn_imp < ErrorDetectionParams.minHornImpedance) ||
            (horn_imp > ErrorDetectionParams.maxHornImpedance))
        {
            HornImpedanceOutofWindowsSuccessiveErrors++;
        }
        else
        {
            HornImpedanceOutofWindowsSuccessiveErrors = 0;
        }

        // Check if frequency tracking keeps the tracking measure acceptable.
        //If not increase successive error counter. If so clear the error counter.
        if ((tracking_measure > ErrorDetectionParams.frequencyTrackingTolerance) ||
            (tracking_measure < -ErrorDetectionParams.frequencyTrackingTolerance))
        {
            FrequencyTrackingSuccessiveErrors++;
        }
        else
        {
            FrequencyTrackingSuccessiveErrors = 0;
        }

        // Check if power tracking keeps the power level acceptable.
        //If not increase successive error counter. If so clear the error counter.
        if (power->real > (TrackingDestinationPower * (1.0f + ErrorDetectionParams.powerTrackingTolerance)) ||
            power->real < (TrackingDestinationPower * (1.0f - ErrorDetectionParams.powerTrackingTolerance)))
        {
            PowerTrackingSuccessiveErrors++;
        }
        else
        {
            PowerTrackingSuccessiveErrors = 0;
        }

        // Calculate maximum number of successive errors.
        uint8_t max_successive_errors;
        max_successive_errors = (uint8_t)((ErrorDetectionParams.timeout / MonitoringPeriod) + 0.5f);

        // Check if any type of error counter has exceeded the maximum number of errors.
        if (HornImpedanceOutofWindowsSuccessiveErrors > max_successive_errors)
        {
            length = snprintf(buff, sizeof(buff), "errhimp I%.1f", horn_imp);

            Error = LATTICE_ERROR_HORN_IMPEDANCE_OUT_OF_WINDOW;
            Events |= EVENT_ERROR_OCCURRED;
        }
        else if (FrequencyTrackingSuccessiveErrors > max_successive_errors)
        {
            length = snprintf(buff, sizeof(buff), "errftra M%.2f", tracking_measure);
            Error = LATTICE_ERROR_FREQUENCY_TRACKING_FAILURE;
            Events |= EVENT_ERROR_OCCURRED;
        }
        else if (PowerTrackingSuccessiveErrors > max_successive_errors)
        {
            length = snprintf(buff, sizeof(buff), "errptra P%.1f", power->real);
            Error = LATTICE_ERROR_POWER_TRACKING_FAILURE;
            Events |= EVENT_ERROR_OCCURRED;
        }
    }

    // If reporting is enabled and error didn't occur send report message.
    if (ReportingEnabled && !length)
    {
        uint8_t trg;
        trg = triggered ? 1 : 0;
        length = snprintf(buff, sizeof(buff), "report T%d F%.0f D%.2f P%.1f I%.1f M%.1f J%.1f",
                          trg, frequency, duty, power->real, power->img,
                          impedance->real, impedance->img);
    }

    Cli_SendMsg(buff, length);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TRIGIN_RISING_Pin)
    {
        Core_TriggerStatusUpdate(TRUE);
    }

    if (GPIO_Pin == TRIGIN_FALLING_Pin)
    {
        Core_TriggerStatusUpdate(FALSE);
    }
}