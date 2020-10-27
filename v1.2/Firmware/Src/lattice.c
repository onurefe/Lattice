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
    float minFrequency;
    float maxFrequency;
} Constraints_t;

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
    float monitoringPeriod;
    float timeout;
} ErrorDetectionParams_t;

typedef struct
{
    Bool_t calibration : 1;
    Bool_t constraints : 1;
    Bool_t searchingParams : 1;
    Bool_t errorDetectionParams : 1;
    Bool_t powerTrackingPidParams : 1;
    Bool_t frequencyTrackingPidParams : 1;
    Bool_t trackingDestinationPower : 1;
} FactoryFlags_t;

enum
{
    EVENT_CALIBRATE = (1 << 0),
    EVENT_SCAN_COMPLETED = (1 << 1),
    EVENT_SEARCH_COMPLETED = (1 << 2),
    EVENT_FACTORY_MODE = (1 << 3),
    EVENT_ERROR_OCCURRED = (1 << 4)
};
typedef uint8_t Events_t;

/* Private variables -------------------------------------------------------*/
// State machine variables.
static Lattice_Status_t Status = LATTICE_STATUS_CONFIG;
static Lattice_Error_t Error;
static Events_t Events;

// Factory configured data.
static Calibration_t Calibration;
static Constraints_t Constraints;
static SearchingParams_t SearchingParams;
static ErrorDetectionParams_t ErrorDetectionParams;
static Pid_Params_t PowerTrackingPidParams;
static Pid_Params_t FrequencyTrackingPidParams;
static float TrackingDestinationPower; // User can overwrite the destination power.

// Runtime variables.
static float ResonanceFrequency;
static float ResonanceImpedance;
static float FullPower;
static uint8_t HornImpedanceOutofWindowsSuccessiveErrors;
static uint8_t PowerTrackingSuccessiveErrors;
static uint8_t FrequencyTrackingSuccessiveErrors;

// Impedance scanning store.
static float ScanCalibrationResistance;
static float ScanImpedanceReal[CALIBRATION_NUM_OF_SAMPLES];
static float ScanImpedanceImg[CALIBRATION_NUM_OF_SAMPLES];

/* Private function prototypes ---------------------------------------------*/
void calculateCalibrationPolynomials(float *scanImpReal, float *scanImpImg,
                                     float calibrationResistance, Complex_t *coeffs);
void startTracking(void);

/* Exported functions ------------------------------------------------------*/
/***
 * @brief Starts the lattice module. Starting includes loading ROM data
 * and initializing the state variables.
 */
void Lattice_Start(void)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return;
    }

    FactoryFlags_t flags;

    // Load burned data.
    flags.calibration = EepromEmulator_ReadObject(LATTICE_CALIBRATION_POLY_EEID, 0,
                                                  sizeof(Calibration),
                                                  NULL, &Calibration);
    flags.constraints = EepromEmulator_ReadObject(LATTICE_CONSTRAINTS_EEID, 0,
                                                  sizeof(Constraints), NULL, &Constraints);
    flags.searchingParams = EepromEmulator_ReadObject(LATTICE_SEARCHING_PARAMS_EEID, 0,
                                                      sizeof(SearchingParams), NULL,
                                                      &SearchingParams);
    flags.errorDetectionParams = EepromEmulator_ReadObject(LATTICE_ERROR_DETECTION_PARAMS_EEID, 0,
                                                           sizeof(ErrorDetectionParams), NULL,
                                                           &ErrorDetectionParams);
    flags.powerTrackingPidParams = EepromEmulator_ReadObject(LATTICE_POWER_TRACKING_PID_PARAMS_EEID, 0,
                                                             sizeof(PowerTrackingPidParams), NULL,
                                                             &PowerTrackingPidParams);
    flags.frequencyTrackingPidParams = EepromEmulator_ReadObject(LATTICE_FREQUENCY_TRACKING_PID_PARAMS_EEID, 0,
                                                                 sizeof(FrequencyTrackingPidParams), NULL,
                                                                 &FrequencyTrackingPidParams);
    flags.trackingDestinationPower = EepromEmulator_ReadObject(LATTICE_TRACKING_DESTINATION_POWER_EEID, 0,
                                                               sizeof(TrackingDestinationPower), NULL,
                                                               &TrackingDestinationPower);
    /* Load default value if not set. */
    if (!flags.calibration)
    {
        float a[3] = DEFAULT_CALIBRATION_POLY_REAL;
        float b[3] = DEFAULT_CALIBRATION_POLY_IMG;

        Calibration.calibrationPoly[0].real = a[0];
        Calibration.calibrationPoly[0].img = b[0];
        Calibration.calibrationPoly[1].real = a[1];
        Calibration.calibrationPoly[1].img = b[1];
        Calibration.calibrationPoly[2].real = a[2];
        Calibration.calibrationPoly[2].img = b[2];
    }

    if (!flags.constraints)
    {
        Constraints.maxPower = DEFAULT_CONSTR_MAX_POWER;
        Constraints.minFrequency = DEFAULT_CONSTR_MIN_FREQ;
        Constraints.maxFrequency = DEFAULT_CONSTR_MAX_FREQ;
    }

    if (!flags.searchingParams)
    {
        SearchingParams.normalizedPower = DEFAULT_SEARCHING_NORMALIZED_POWER;
        SearchingParams.steps = DEFAULT_SEARCHING_STEPS;
    }

    if (!flags.errorDetectionParams)
    {
        ErrorDetectionParams.minHornImpedance = DEFAULT_ERRDET_MIN_HORN_IMPEDANCE;
        ErrorDetectionParams.maxHornImpedance = DEFAULT_ERRDET_MAX_HORN_IMPEDANCE;
        ErrorDetectionParams.frequencyTrackingTolerance = DEFAULT_ERRDET_FREQ_TRACKING_TOLERANCE;
        ErrorDetectionParams.powerTrackingTolerance = DEFAULT_ERRDET_POWER_TRACKING_TOLERANCE;
        ErrorDetectionParams.monitoringPeriod = DEFAULT_ERRDET_MONITORING_PERIOD;
        ErrorDetectionParams.timeout = DEFAULT_ERRDET_TIMEOUT;
    }

    if (!flags.powerTrackingPidParams)
    {
        PowerTrackingPidParams.tfilt = DEFAULT_PTRACK_PID_TF;
        PowerTrackingPidParams.kp = DEFAULT_PTRACK_PID_KP;
        PowerTrackingPidParams.ki = DEFAULT_PTRACK_PID_KI;
        PowerTrackingPidParams.kd = DEFAULT_PTRACK_PID_KD;
    }

    if (!flags.frequencyTrackingPidParams)
    {
        FrequencyTrackingPidParams.tfilt = DEFAULT_FTRACK_PID_TF;
        FrequencyTrackingPidParams.kp = DEFAULT_FTRACK_PID_KP;
        FrequencyTrackingPidParams.ki = DEFAULT_FTRACK_PID_KI;
        FrequencyTrackingPidParams.kd = DEFAULT_FTRACK_PID_KD;
    }

    if (!flags.trackingDestinationPower)
    {
        TrackingDestinationPower = DEFAULT_DESTINATION_POWER;
    }

    Core_Init(Calibration.calibrationPoly, Constraints.minFrequency,
              Constraints.maxFrequency, MIN_NORMALIZED_POWER, MAX_NORMALIZED_POWER);

    // Start searching for the resonance frequency.
    Core_Search(SearchingParams.normalizedPower,
                Constraints.minFrequency, Constraints.maxFrequency, SearchingParams.steps);

    Status = LATTICE_STATUS_SEARCHING;
    Error = LATTICE_ERROR_NONE;
    Events = 0;
}

/***
 * @brief Handles pending events and executes submodules.
 */
void Lattice_Execute(void)
{
    Core_Execute();

    // If error occurred; core is stopped and device put
    //into error mode.
    if (Events & EVENT_ERROR_OCCURRED)
    {
        if ((Status == LATTICE_STATUS_SEARCHING) ||
            (Status == LATTICE_STATUS_TRACKING))
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
        if ((Status == LATTICE_STATUS_SEARCHING) ||
            (Status == LATTICE_STATUS_TRACKING) ||
            (Status == LATTICE_STATUS_CALIBRATING))
        {
            Core_Stop();
        }

        // Put device to pending factory initialization.
        Status = LATTICE_STATUS_CONFIG;
        Events ^= EVENT_FACTORY_MODE;
    }

    // Handle search completed event.
    if (Events & EVENT_SEARCH_COMPLETED)
    {
        if (Status == LATTICE_STATUS_SEARCHING)
        {
            // Start tracking.
            startTracking();

            Status = LATTICE_STATUS_TRACKING;
        }

        Events ^= EVENT_SEARCH_COMPLETED;
    }

    // Handle calibration command event.
    if (Events & EVENT_CALIBRATE)
    {
        if (Status == LATTICE_STATUS_CONFIG)
        {
            // Start scanning.
            Core_Scan(MIN_FREQUENCY, MAX_FREQUENCY, CALIBRATION_NUM_OF_SAMPLES,
                      ScanImpedanceReal, ScanImpedanceImg);

            Status = LATTICE_STATUS_CALIBRATING;
        }

        Events ^= EVENT_CALIBRATE;
    }

    // Handle scan completed event.
    if (Events & EVENT_SCAN_COMPLETED)
    {
        if (Status == LATTICE_STATUS_CALIBRATING)
        {
            Complex_t coeffs[CALIBRATION_POLY_DEGREE + 1];

            calculateCalibrationPolynomials(ScanImpedanceReal, ScanImpedanceImg,
                                            ScanCalibrationResistance, coeffs);

            // Record object.
            EepromEmulator_WriteObject(LATTICE_CALIBRATION_POLY_EEID, sizeof(Calibration_t),
                                       (uint8_t *)&Calibration);

            // Return to factory parameter setup.
            Status = LATTICE_STATUS_CONFIG;
        }

        // Clear scan completed event.
        Events ^= EVENT_SCAN_COMPLETED;
    }
}

/***
 * @brief Puts the device into the factory mode.
 * 
 * @param password: Password generated by the hash function with device 
 * information.
 * 
 * @retval Operation result: either TRUE or FALSE.
 */
Bool_t Lattice_ConfigMode(int32_t password)
{
    // If device info is not set; the device is already authorized.
    if (Status == LATTICE_STATUS_CONFIG)
    {
        return TRUE;
    }

    // Check password.
    if (password != CONFIG_PASSWORD)
    {
        return FALSE;
    }

    Events |= EVENT_FACTORY_MODE;

    return TRUE;
}

/***
 * @brief Loads default values to configurable parameters.
 * 
 * @retval Operation result(TRUE or FALSE).
 */
Bool_t Lattice_LoadDefaults(void)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    {
        float a[3] = DEFAULT_CALIBRATION_POLY_REAL;
        float b[3] = DEFAULT_CALIBRATION_POLY_IMG;

        Calibration.calibrationPoly[0].real = a[0];
        Calibration.calibrationPoly[0].img = b[0];
        Calibration.calibrationPoly[1].real = a[1];
        Calibration.calibrationPoly[1].img = b[1];
        Calibration.calibrationPoly[2].real = a[2];
        Calibration.calibrationPoly[2].img = b[2];
    }
    EepromEmulator_DeleteObject(LATTICE_CALIBRATION_POLY_EEID);

    {
        Constraints.maxPower = DEFAULT_CONSTR_MAX_POWER;
        Constraints.minFrequency = DEFAULT_CONSTR_MIN_FREQ;
        Constraints.maxFrequency = DEFAULT_CONSTR_MAX_FREQ;
    }
    EepromEmulator_DeleteObject(LATTICE_CONSTRAINTS_EEID);

    {
        SearchingParams.normalizedPower = DEFAULT_SEARCHING_NORMALIZED_POWER;
        SearchingParams.steps = DEFAULT_SEARCHING_STEPS;
    }
    EepromEmulator_DeleteObject(LATTICE_SEARCHING_PARAMS_EEID);

    {
        ErrorDetectionParams.minHornImpedance = DEFAULT_ERRDET_MIN_HORN_IMPEDANCE;
        ErrorDetectionParams.maxHornImpedance = DEFAULT_ERRDET_MAX_HORN_IMPEDANCE;
        ErrorDetectionParams.frequencyTrackingTolerance = DEFAULT_ERRDET_FREQ_TRACKING_TOLERANCE;
        ErrorDetectionParams.powerTrackingTolerance = DEFAULT_ERRDET_POWER_TRACKING_TOLERANCE;
        ErrorDetectionParams.monitoringPeriod = DEFAULT_ERRDET_MONITORING_PERIOD;
        ErrorDetectionParams.timeout = DEFAULT_ERRDET_TIMEOUT;
    }
    EepromEmulator_DeleteObject(LATTICE_ERROR_DETECTION_PARAMS_EEID);

    {
        PowerTrackingPidParams.tfilt = DEFAULT_PTRACK_PID_TF;
        PowerTrackingPidParams.kp = DEFAULT_PTRACK_PID_KP;
        PowerTrackingPidParams.ki = DEFAULT_PTRACK_PID_KI;
        PowerTrackingPidParams.kd = DEFAULT_PTRACK_PID_KD;
    }
    EepromEmulator_DeleteObject(LATTICE_POWER_TRACKING_PID_PARAMS_EEID);

    {
        FrequencyTrackingPidParams.tfilt = DEFAULT_FTRACK_PID_TF;
        FrequencyTrackingPidParams.kp = DEFAULT_FTRACK_PID_KP;
        FrequencyTrackingPidParams.ki = DEFAULT_FTRACK_PID_KI;
        FrequencyTrackingPidParams.kd = DEFAULT_FTRACK_PID_KD;
    }
    EepromEmulator_DeleteObject(LATTICE_FREQUENCY_TRACKING_PID_PARAMS_EEID);

    {
        TrackingDestinationPower = DEFAULT_DESTINATION_POWER;
    }
    EepromEmulator_DeleteObject(LATTICE_TRACKING_DESTINATION_POWER_EEID);

    return TRUE;
}

/***
 * @brief Invokes calibration procedure.
 * 
 * @retval Operation result: either TRUE or FALSE.
 */
Bool_t Lattice_Calibrate(void)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    Events |= EVENT_CALIBRATE;

    return TRUE;
}

/***
 * @brief Soft resets the system.
 */
void Lattice_Reset(void)
{
    // Reset the system.
    HAL_NVIC_SystemReset();
}

/***
 * @brief Sets calibration polynomial. 
 *
 * @param a0: Real part zeroth order term coefficient.
 * @param a1: Real part first order term coefficient.
 * @param a2: Real part second order term coefficient.
 * @param b0: Imaginary part zeroth order term coefficient.
 * @param b1: Imaginary part first order term coefficient.
 * @param b2: Imaginary part second order term coefficient.
 *
 * @retval Operation result(TRUE or FALSE).
 */
Bool_t Lattice_SetCalibrationPolynomial(float a0, float a1, float a2, float b0, float b1,
                                        float b2)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    Calibration.calibrationPoly[0].real = a0;
    Calibration.calibrationPoly[0].img = b0;
    Calibration.calibrationPoly[1].real = a1;
    Calibration.calibrationPoly[1].img = b1;
    Calibration.calibrationPoly[2].real = a2;
    Calibration.calibrationPoly[2].img = b2;

    EepromEmulator_WriteObject(LATTICE_CALIBRATION_POLY_EEID, sizeof(Calibration_t), &Calibration);

    return TRUE;
}

/***
 * @brief Sets constraint variables which the device should obey.
 * 
 * @param maxPower: Maximum power which device can deliver.
 * @param minFrequency: Minimum frequency value.
 * @param maxFrequency: Maximum frequency value.
 * 
 * @retval Operation result; either TRUE or FALSE.
 */
Bool_t Lattice_SetConstraints(float maxPower, float minFrequency, float maxFrequency)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    if ((minFrequency >= MIN_FREQUENCY) && (maxFrequency <= MAX_FREQUENCY))
    {
        Constraints.maxPower = maxPower;
        Constraints.minFrequency = minFrequency;
        Constraints.maxFrequency = maxFrequency;

        EepromEmulator_WriteObject(LATTICE_CONSTRAINTS_EEID, sizeof(Constraints),
                                   &Constraints);

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/***
 * @brief Sets the searching task parameters.
 * 
 * @param normalizedPower: Normalized power. Between 0 to 1. This value
 * represents an arbitrary power unit which 1.0 meaning all the power 
 * device can deliver to the current load.
 * @param steps: Determines the searching resolution.
 * 
 * @retval Operation result: either TRUE or FALSE.
 */
Bool_t Lattice_SetSearchingParams(float normalizedPower, uint16_t steps)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    if ((0.0f <= normalizedPower) && (normalizedPower <= 1.0f))
    {
        SearchingParams.normalizedPower = normalizedPower;
        SearchingParams.steps = steps;

        EepromEmulator_WriteObject(LATTICE_SEARCHING_PARAMS_EEID,
                                   sizeof(SearchingParams_t), &SearchingParams);

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/***
 * @brief Sets error detection parameters.
 * 
 * @param minHornImpedance: Acceptable minimum impedance of the horn.
 * @param maxHornImpedance: Acceptable maximum impedance of the horn.
 * @param powerTrackingTolerance: Tolerance of the power tracking algorithm. 
 * @param frequencyTrackingTolerance: Tolerance of the frequency tracking algorithm.
 * @param monitoringPeriod: Interval which controls are applied.
 * @param timeout: Timeout value in seconds. This some value is out of the acceptable 
 * boundaries for <timeout> number of seconds; error will be invoked.
 * 
 * @retval Operation result: either TRUE or FALSE.
 */
Bool_t Lattice_SetErrorDetectionParams(float minHornImpedance, float maxHornImpedance,
                                       float powerTrackingTolerance, float frequencyTrackingTolerance,
                                       float monitoringPeriod, float timeout)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    // Update error detection parameters.
    if (((0.0f < minHornImpedance) && (minHornImpedance < maxHornImpedance)) && (0.0f < monitoringPeriod) &&
        (0.0f < powerTrackingTolerance) && (0.0f < frequencyTrackingTolerance) && (0.0f < timeout))
    {
        ErrorDetectionParams.minHornImpedance = minHornImpedance;
        ErrorDetectionParams.maxHornImpedance = maxHornImpedance;
        ErrorDetectionParams.powerTrackingTolerance = powerTrackingTolerance;
        ErrorDetectionParams.frequencyTrackingTolerance = frequencyTrackingTolerance;
        ErrorDetectionParams.monitoringPeriod = monitoringPeriod;
        ErrorDetectionParams.timeout = timeout;

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

/***
 * @brief Gets power tracking PID coefficients.
 * 
 * @param kp: Proportional value.
 * @param ki: Integral operator coefficient.
 * @param kd: Derivative operator coefficient.
 * @param tf: Input filter time constant.
 * 
 * @retval Operation result: either TRUE or FALSE.
 */
Bool_t Lattice_SetPowerTrackingPidCoeffs(float kp, float ki, float kd, float tf)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    if ((kp >= 0.0f) && (ki >= 0.0f) && (kd >= 0.0f) && (tf > 0.0f))
    {
        PowerTrackingPidParams.kp = kp;
        PowerTrackingPidParams.ki = ki;
        PowerTrackingPidParams.kd = kd;
        PowerTrackingPidParams.tfilt = tf;

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

/***
 * @brief Sets frequency tracking PID coefficients.
 * 
 * @param kp: Proportional value.
 * @param ki: Integral operator coefficient.
 * @param kd: Derivative operator coefficient.
 * @param tf: Input filter time constant.
 * 
 * @retval Operation result: either TRUE or FALSE.
 */
Bool_t Lattice_SetFrequencyTrackingPidCoeffs(float kp, float ki, float kd, float tf)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    if ((kp >= 0.0f) && (ki >= 0.0f) && (kd >= 0.0f) && (tf > 0.0f))
    {
        FrequencyTrackingPidParams.kp = kp;
        FrequencyTrackingPidParams.ki = ki;
        FrequencyTrackingPidParams.kd = kd;
        FrequencyTrackingPidParams.tfilt = tf;

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

/***
 * @brief Sets the destination power(used in tracking mode).
 * 
 * @param destinationPower: Destination power.
 * 
 * @retval Operation result: either TRUE or FALSE.
 */
Bool_t Lattice_SetDestinationPower(float destinationPower)
{
    if (Status != LATTICE_STATUS_CONFIG)
    {
        return FALSE;
    }

    if (0.0f < destinationPower)
    {
        TrackingDestinationPower = destinationPower;

        EepromEmulator_WriteObject(LATTICE_TRACKING_DESTINATION_POWER_EEID,
                                   sizeof(TrackingDestinationPower),
                                   &TrackingDestinationPower);

        return TRUE;
    }
    else
    {
        return FALSE;
    }

    return TRUE;
}

/***
 * @brief Gets calibration polynomial. 
 *
 * @param a0: Pointer to real part zeroth order term coefficient.
 * @param a1: Pointer to real part first order term coefficient.
 * @param a2: Pointer to real part second order term coefficient.
 * @param b0: Pointer to imaginary part zeroth order term coefficient.
 * @param b1: Pointer to imaginary part first order term coefficient.
 * @param b2: Pointer to imaginary part second order term coefficient.
 *
 * @retval Operation result(TRUE or FALSE).
 */
void Lattice_GetCalibrationPolynomial(float *a0, float *a1, float *a2, float *b0, float *b1,
                                      float *b2)
{
    *a0 = Calibration.calibrationPoly[0].real;
    *b0 = Calibration.calibrationPoly[0].img;
    *a1 = Calibration.calibrationPoly[1].real;
    *b1 = Calibration.calibrationPoly[1].img;
    *a2 = Calibration.calibrationPoly[2].real;
    *b2 = Calibration.calibrationPoly[2].img;
}

/***
 * @brief Gets constraint variables.
 * 
 * @param maxPower: Pointer to return maximum power.
 * @param minFrequency: Pointer to return minimum frequency.
 * @param maxFrequency: Pointer to return maximum frequency.
 */
void Lattice_GetConstraints(float *maxPower, float *minFrequency, float *maxFrequency)
{
    *maxPower = Constraints.maxPower;
    *minFrequency = Constraints.minFrequency;
    *maxFrequency = Constraints.maxFrequency;
}

/***
 * @brief Gets searching parameters.
 * 
 * @param normalizedPower: Pointer to return the normalized searching power.
 * @param steps: Point to return the number of steps.
 */
void Lattice_GetSearchingParams(float *normalizedPower, uint16_t *steps)
{
    *normalizedPower = SearchingParams.normalizedPower;
    *steps = SearchingParams.steps;
}

/***
 * @brief Gets error detection parameters.
 * 
 * @param minHornImpedance: Pointer to return minimum horn impedance.
 * @param maxHornImpedance: Pointer to return maximum horn impedance.
 * @param powerTrackingTolerance: Pointer to return power tracking tolerance.
 * @param frequencyTrackingTolerance: Pointer to return frequency tracking tolerance.
 * @param monitoringPeriod: Pointer to return monitoring period value.
 * @param timeout: Pointer to return timeout value.
 */
void Lattice_GetErrorDetectionParams(float *minHornImpedance, float *maxHornImpedance,
                                     float *powerTrackingTolerance, float *frequencyTrackingTolerance,
                                     float *monitoringPeriod, float *timeout)
{
    *minHornImpedance = ErrorDetectionParams.minHornImpedance;
    *maxHornImpedance = ErrorDetectionParams.maxHornImpedance;
    *powerTrackingTolerance = ErrorDetectionParams.powerTrackingTolerance;
    *frequencyTrackingTolerance = ErrorDetectionParams.frequencyTrackingTolerance;
    *monitoringPeriod = ErrorDetectionParams.monitoringPeriod;
    *timeout = ErrorDetectionParams.timeout;
}

/***
 * @brief Gets power tracking PID coefficients.
 * 
 * @param kp: Pointer to return the proportional value.
 * @param ki: Pointer to return the integral operator coefficient.
 * @param kd: Pointer to return the derivative operator coefficient.
 * @param tf: Pointer to return the input filter time constant.
 */
void Lattice_GetPowerTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf)
{
    *kp = PowerTrackingPidParams.kp;
    *ki = PowerTrackingPidParams.ki;
    *kd = PowerTrackingPidParams.kd;
    *tf = PowerTrackingPidParams.tfilt;
}

/***
 * @brief Gets frequency tracking PID coefficients.
 * 
 * @param kp: Pointer to return proportional value.
 * @param ki: Pointer to return integral operator's coefficient.
 * @param kd: Pointer to return derivative operator's coefficient.
 * @param tf: Pointer to return input filter's time constant.
 */
void Lattice_GetFrequencyTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf)
{
    *kp = FrequencyTrackingPidParams.kp;
    *ki = FrequencyTrackingPidParams.ki;
    *kd = FrequencyTrackingPidParams.kd;
    *tf = FrequencyTrackingPidParams.tfilt;
}

/***
 * @brief Gets the destination power(used in tracking mode).
 * 
 * @param destinationPower: Pointer to return destination power.
 */
void Lattice_GetDestinationPower(float *destinationPower)
{
    *destinationPower = TrackingDestinationPower;
}

/***
 * @brief Gets monitoring variables.
 * 
 * @param status: Pointer to return status of the device.
 * @param triggered: Pointer to return trigger state of the device.
 * @param frequency: Pointer to return output frequency.
 * @param duty: Pointer to return output duty.
 * @param powerReal: Pointer to return power phasor real component.
 * @param powerImg: Pointer to return power phasor imaginary component.
 * @param impReal: Pointer to return impendace phasor real component.
 * @param impImg: Pointer to return impedance phasor imaginary component.
 */
void Lattice_GetReport(Lattice_Status_t *status, Bool_t *triggered, float *frequency, float *duty,
                       float *powerReal, float *powerImg, float *impReal, float *impImg)
{
    Complex_t power;
    Complex_t impedance;

    *status = Status;
    Core_GetMonitoringParams(triggered, frequency, duty, &power, &impedance);
    *powerReal = power.real;
    *powerImg = power.img;
    *impReal = impedance.real;
    *impImg = impedance.img;
}

/***
 * @brief Returns the status of the module.
 * 
 * @retval Status.
 */
Lattice_Status_t Lattice_GetStatus(void)
{
    return Status;
}

/***
 * @brief Returns the error code.
 * 
 * @retval Error.
 */
Lattice_Error_t Lattice_GetError(void)
{
    return Error;
}

/* Private functions -------------------------------------------------------*/
/***
 * @brief Starts tracking.
 */
void startTracking(void)
{
    // Start tracking.
    Core_Track(&PowerTrackingPidParams, &FrequencyTrackingPidParams, ResonanceFrequency,
               FullPower, TrackingDestinationPower, ErrorDetectionParams.monitoringPeriod);

    // Clear error counters.
    HornImpedanceOutofWindowsSuccessiveErrors = 0;
    PowerTrackingSuccessiveErrors = 0;
    FrequencyTrackingSuccessiveErrors = 0;
}

/***
 * @brief Calculates the calibration polynomial coefficients from the impedance
 * curve of the calibration element.
 * 
 * @param scanImpReal: Element's impedance phasor curve's real part.
 * @param scanImpImg: Element's impedance phasor curve's imaginary part.
 * @param calibrationResistance: Calibration element's resistance.
 * @param coeffs: Pointer to return the regression polynomial coefficients.
 */
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

/* Imported callbacks ------------------------------------------------------*/
/***
 * @brief Gets triggered when scan process is completed.
 */
void Core_ScanCompletedCallback(void)
{
    Events |= EVENT_SCAN_COMPLETED;
}

/***
 * @brief Gets triggered when search is completed.
 * 
 * @param resonanceFrequency: Detected resonance frequency.
 * @param resonanceImpedance: Impedance at the resonance frequency.
 * @param fullPower: Maximum power transmittable at resonance frequency.
 */
void Core_SearchCompletedCallback(float resonanceFrequency, float resonanceImpedance,
                                  float fullPower)
{
    ResonanceFrequency = resonanceFrequency;
    ResonanceImpedance = resonanceImpedance;
    FullPower = fullPower;

    Events |= EVENT_SEARCH_COMPLETED;
}

/***
 * @brief Gets triggered when monitoring event occurrs. 
 * 
 * @param triggered: Flag showing if triggered or not.
 * @param frequency: Tracking frequency.
 * @param duty: Tracking duty.
 * @param power: Power transmitted to the horn.
 * @param impedance: Impedance of the horn.
 */
void Core_MonitoringCallback(Bool_t triggered, float frequency, float duty,
                             Complex_t *power, Complex_t *impedance)
{
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
        max_successive_errors = (uint8_t)((ErrorDetectionParams.timeout /
                                           ErrorDetectionParams.monitoringPeriod) +
                                          0.5f);

        // Check if any type of error counter has exceeded the maximum number of errors.
        if (HornImpedanceOutofWindowsSuccessiveErrors > max_successive_errors)
        {
            Error = LATTICE_ERROR_HORN_IMPEDANCE_OUT_OF_WINDOW;
            Events |= EVENT_ERROR_OCCURRED;
        }
        else if (FrequencyTrackingSuccessiveErrors > max_successive_errors)
        {
            Error = LATTICE_ERROR_FREQUENCY_TRACKING_FAILURE;
            Events |= EVENT_ERROR_OCCURRED;
        }
        else if (PowerTrackingSuccessiveErrors > max_successive_errors)
        {
            Error = LATTICE_ERROR_POWER_TRACKING_FAILURE;
            Events |= EVENT_ERROR_OCCURRED;
        }
    }
}

/***
 * @brief EXTI callback implementation.
 * 
 * @param GPIO_Pin: Pin number which EXTI event occurred.
 */
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