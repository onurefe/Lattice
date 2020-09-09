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
    Bool_t calibration : 1;
    Bool_t constraints : 1;
    Bool_t deviceInfo : 1;
    Bool_t searchingParams : 1;
    Bool_t powerTrackingPidParams : 1;
    Bool_t frequencyTrackingPidParams : 1;
    Bool_t trackingDestinationPower : 1;
} FactoryFlags_t;

enum
{
    EVENT_SCAN_COMMAND = (1 << 0),
    EVENT_SCAN_COMPLETED = (1 << 1),
    EVENT_SEARCH_COMPLETED = (1 << 2),
};
typedef uint8_t Events_t;

/* Private variables -------------------------------------------------------*/
// State machine variables.
static Lattice_Status_t Status = LATTICE_STATUS_READY;
static Events_t Events;
static FactoryFlags_t Flags;
static Bool_t Authorized;

// Factory configured data.
static Calibration_t Calibration;
static Constraints_t Constraints;
static DeviceInfo_t DeviceInfo;
static SearchingParams_t SearchingParams;
static Pid_Params_t PowerTrackingPidParams;
static Pid_Params_t FrequencyTrackingPidParams;
static float TrackingDestinationPower; // User can overwrite the destination power.

// Runtime variables.
static float ResonanceFrequency;
static float ResonanceImpedance;
static float FullPower;

// Impedance scanning store.
static float ScanCalibrationResistance;
static float ScanCalibrationDataReal[CALIBRATION_NUM_OF_SAMPLES];
static float ScanCalibrationDataImg[CALIBRATION_NUM_OF_SAMPLES];

/* Private function prototypes ---------------------------------------------*/
void Lattice_Start(void)
{
    if (Status != LATTICE_STATUS_READY)
    {
        return;
    }

    // Load burned data.
    Flags.calibration = EepromEmulator_ReadObject(LATTICE_CALIBRATION_POLY_EEID, 0,
                                                  sizeof(Calibration_t),
                                                  NULL, &Calibration);
    Flags.constraints = EepromEmulator_ReadObject(LATTICE_CONSTRAINTS_EEID, 0,
                                                  sizeof(Constraints_t), NULL, &Constraints);
    Flags.deviceInfo = EepromEmulator_ReadObject(LATTICE_DEVICE_INFO_EEID, 0,
                                                 sizeof(DeviceInfo_t), NULL, &DeviceInfo);
    Flags.powerTrackingPidParams = EepromEmulator_ReadObject(LATTICE_POWER_TRACKING_PID_PARAMS_EEID, 0,
                                                             sizeof(Pid_Params_t), NULL,
                                                             &PowerTrackingPidParams);
    Flags.frequencyTrackingPidParams = EepromEmulator_ReadObject(LATTICE_FREQUENCY_TRACKING_PID_PARAMS_EEID, 0,
                                                                 sizeof(Pid_Params_t), NULL,
                                                                 &FrequencyTrackingPidParams);
    Flags.searchingParams = EepromEmulator_ReadObject(LATTICE_SEARCHING_PARAMS_EEID, 0,
                                                      sizeof(SearchingParams_t), NULL,
                                                      &SearchingParams);
    Flags.trackingDestinationPower = EepromEmulator_ReadObject(LATTICE_TRACKING_DESTINATION_POWER_EEID, 0,
                                                               sizeof(float), NULL,
                                                               &TrackingDestinationPower);

    // Determine the state of device.
    if (Flags.calibration && Flags.constraints && Flags.deviceInfo && Flags.powerTrackingPidParams &&
        Flags.frequencyTrackingPidParams && Flags.searchingParams && Flags.trackingDestinationPower)
    {
        Core_Init(&Calibration, LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY, Constraints.minLoading,
                  Constraints.maxLoading);

        // Start searching for the resonance frequency.
        Core_Search(SearchingParams.normalizedPower,
                    LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY, SearchingParams.steps);

        Status = LATTICE_STATUS_OPERATING;
    }
    else
    {
        Status = LATTICE_STATUS_PENDING_FACTORY_INITIALIZATION;
    }

    Events = 0;
}

void Lattice_Execute(void)
{
    if ((Status != LATTICE_STATUS_PENDING_FACTORY_INITIALIZATION) &&
        Status != LATTICE_STATUS_OPERATING)
    {
        return;
    }

    Core_Execute();

    // Handle search completed event.
    if (Events & EVENT_SEARCH_COMPLETED)
    {
        if (Status == LATTICE_STATUS_OPERATING)
        {
            // Start tracking.
            Core_Track(&PowerTrackingPidParams, &FrequencyTrackingPidParams, ResonanceFrequency,
                       FullPower, TrackingDestinationPower);
        }

        Events ^= EVENT_SEARCH_COMPLETED;
    }

    // Handle scan command event.
    if (Events & EVENT_SCAN_COMMAND)
    {
        if (Status == LATTICE_STATUS_PENDING_FACTORY_INITIALIZATION)
        {
            Core_Init(NULL, LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY, 0.0f,
                      1.0f);

            // Start scanning.
            Core_Scan(LATTICE_MIN_FREQUENCY, LATTICE_MAX_FREQUENCY,
                      CALIBRATION_NUM_OF_SAMPLES, ScanCalibrationDataReal, ScanCalibrationDataImg);
        }

        Events ^= EVENT_SCAN_COMMAND;
    }

    // Handle scan completed event.
    if (Events & EVENT_SCAN_COMPLETED)
    {
        if (Status == LATTICE_STATUS_PENDING_FACTORY_INITIALIZATION)
        {
            float coeffs_real[CALIBRATION_POLY_DEGREE + 1];
            float coeffs_img[CALIBRATION_POLY_DEGREE + 1];

            // Obtain voltage fixing complex phasor data. Since the analog circuitry
            //is weak on the voltage measurement side; it's assumed that all the error
            //is due to voltage measuring circuitry.
            for (uint16_t i = 0; i < CALIBRATION_NUM_OF_SAMPLES; i++)
            {
                float abs_sqr;
                abs_sqr = ScanCalibrationDataReal[i] * ScanCalibrationDataReal[i] +
                          ScanCalibrationDataImg[i] * ScanCalibrationDataImg[i];

                ScanCalibrationDataReal[i] = ScanCalibrationResistance * ScanCalibrationDataReal[i] /
                                             abs_sqr;
                ScanCalibrationDataImg[i] = -ScanCalibrationResistance * ScanCalibrationDataImg[i] /
                                            abs_sqr;
            }

            // Obtain calibration polynomials(for real and imaginary parts).
            Polyreg_Fit(ScanCalibrationDataReal, coeffs_real);
            Polyreg_Fit(ScanCalibrationDataImg, coeffs_img);

            // Set calibration polynomial data.
            for (uint8_t i = 0; i < (CALIBRATION_POLY_DEGREE + 1); i++)
            {
                Calibration.calibrationPoly[i].real = coeffs_real[i];
                Calibration.calibrationPoly[i].img = coeffs_img[i];
            }

            // Record object.
            EepromEmulator_WriteObject(LATTICE_CALIBRATION_POLY_EEID, sizeof(Calibration_t),
                                       (uint8_t *)&Calibration);
        }

        // Clear scan completed event.
        Events ^= EVENT_SCAN_COMPLETED;
    }
}

void Lattice_Stop(void)
{
    if ((Status != LATTICE_STATUS_PENDING_FACTORY_INITIALIZATION) &&
        Status != LATTICE_STATUS_OPERATING)
    {
        return;
    }

    Status = LATTICE_STATUS_READY;
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

void Core_PeriodicMeasurementCallback(float frequency, float duty,
                                      Complex_t *power, Complex_t *impedance)
{
    char buff[64];
    uint8_t length;
    length = snprintf(buff, sizeof(buff), "meas F%f D%f P%f I%f M%f J%f",
                      frequency, duty, power->real, power->img,
                      impedance->real, impedance->img);

    // Transmit measurement.
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