#ifndef __CONFIG_H
#define __CONFIG_H

/* Exported constants ------------------------------------------------------*/
#define FBR_BULK_CAPACITOR 1880e-6
#define FBR_CHARGEUP_RESISTOR 120.0

// Lattice module configuration.
#define LATTICE_MIN_FREQUENCY 18E3
#define LATTICE_MAX_FREQUENCY 22E3

// Calibration function constants.
#define CALIBRATION_POLY_DEGREE 2
#define CALIBRATION_NUM_OF_SAMPLES 100

// RS485 module configuration.
#define RS485_RX_BUFFER_SIZE 128
#define RS485_TX_BUFFER_SIZE 128

// CLI module configuration.
#define CLI_MAX_COMMAND_LINE_LENGTH 64
#define CLI_LINE_TERMINATOR \
    {                       \
        '\n', '\r'          \
    }

#endif