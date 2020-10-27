#ifndef __CONFIG_H
#define __CONFIG_H

/* Exported constants ------------------------------------------------------*/
#define MIN_FREQUENCY 18E3
#define MAX_FREQUENCY 22E3
#define MIN_NORMALIZED_POWER 0.0f
#define MAX_NORMALIZED_POWER 1.0f
#define MAX_OUTPUT_POWER 2E3

#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 0
#define CONFIG_PASSWORD 111995

// Defaults.
#define DEFAULT_SEARCHING_NORMALIZED_POWER 1.0f
#define DEFAULT_SEARCHING_STEPS 2000
#define DEFAULT_CALIBRATION_POLY_REAL {1.0, 0.0, 0.0} 
#define DEFAULT_CALIBRATION_POLY_IMG {0.0, 0.0, 0.0}
#define DEFAULT_CONSTR_MAX_POWER MAX_OUTPUT_POWER
#define DEFAULT_CONSTR_MIN_FREQ MIN_FREQUENCY
#define DEFAULT_CONSTR_MAX_FREQ MAX_FREQUENCY
#define DEFAULT_ERRDET_MIN_HORN_IMPEDANCE 10.0f
#define DEFAULT_ERRDET_MAX_HORN_IMPEDANCE 1000.0f
#define DEFAULT_ERRDET_FREQ_TRACKING_TOLERANCE 0.25f
#define DEFAULT_ERRDET_POWER_TRACKING_TOLERANCE 0.4f
#define DEFAULT_ERRDET_MONITORING_PERIOD 0.5f
#define DEFAULT_ERRDET_TIMEOUT 5.0f
#define DEFAULT_PTRACK_PID_TF 0.1f
#define DEFAULT_PTRACK_PID_KP 1.0f
#define DEFAULT_PTRACK_PID_KI 0.1f
#define DEFAULT_PTRACK_PID_KD 0.0f
#define DEFAULT_FTRACK_PID_TF 0.1f
#define DEFAULT_FTRACK_PID_KP 1.0f
#define DEFAULT_FTRACK_PID_KI 0.1f
#define DEFAULT_FTRACK_PID_KD 0.0f
#define DEFAULT_DESTINATION_POWER 400.0f

// Contactor trigger time.
#define FBR_BULK_CAPACITOR 1880e-6
#define FBR_CHARGEUP_RESISTOR 120.0

// Calibration function constants.
#define CALIBRATION_POLY_DEGREE 2
#define CALIBRATION_NUM_OF_SAMPLES 100

// RS485 module configuration.
#define RS485_RX_BUFFER_SIZE 256
#define RS485_TX_BUFFER_SIZE 256

// CLI module configuration.
#define CLI_MAX_COMMAND_LINE_LENGTH 128
#define CLI_LINE_TERMINATOR \
    {                       \
        '\r', '\n'          \
    }

#endif