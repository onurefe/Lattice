#ifndef __LATTICE_H
#define __LATTICE_H

#include "global.h"
#include "core.h"
#include "complex.h"

/* Exported types ----------------------------------------------------------*/
enum
{
    LATTICE_STATUS_CONFIG = 0,
    LATTICE_STATUS_CALIBRATING,
    LATTICE_STATUS_SEARCHING,
    LATTICE_STATUS_TRACKING,
    LATTICE_STATUS_ERROR
};
typedef uint8_t Lattice_Status_t;

enum
{
    LATTICE_ERROR_NONE = 0,
    LATTICE_ERROR_POWER_TRACKING_FAILURE,
    LATTICE_ERROR_FREQUENCY_TRACKING_FAILURE,
    LATTICE_ERROR_HORN_IMPEDANCE_OUT_OF_WINDOW
};
typedef uint8_t Lattice_Error_t;

/* Exported functions ------------------------------------------------------*/
// Cli control functions.
Bool_t Lattice_ConfigMode(int32_t password);
Bool_t Lattice_LoadDefaults(void);
void Lattice_Reset(void);
Lattice_Status_t Lattice_GetStatus(void);
Lattice_Error_t Lattice_GetError(void);
Bool_t Lattice_Calibrate(void);
void Lattice_GetReport(Lattice_Status_t *status, Bool_t *triggerStatus, float *frequency,
                         float *duty, float *powerReal, float *powerImg, float *impedanceReal,
                         float *impendaceImg);
Bool_t Lattice_SetCalibrationPolynomial(float a0, float a1, float a2, float b0, float b1,
                                        float b2);
Bool_t Lattice_SetConstraints(float maxPower, float minFrequency, float maxFrequency);
Bool_t Lattice_SetSearchingParams(float normalizedPower, uint16_t steps);
Bool_t Lattice_SetErrorDetectionParams(float minHornImpedance, float maxHornImpedance,
                                       float powerTrackingTolerance, float frequencyTrackingTolerance,
                                       float monitoringPeriod, float timeout);
Bool_t Lattice_SetPowerTrackingPidCoeffs(float kp, float ki, float kd, float tf);
Bool_t Lattice_SetFrequencyTrackingPidCoeffs(float kp, float ki, float kd, float tf);
Bool_t Lattice_SetDestinationPower(float destinationPower);
void Lattice_GetCalibrationPolynomial(float *a0, float *a1, float *a2, float *b0, float *b1,
                                      float *b2);
void Lattice_GetConstraints(float *maxPower, float *minFrequency, float *maxFrequency);
void Lattice_GetSearchingParams(float *normalizedPower, uint16_t *steps);
void Lattice_GetErrorDetectionParams(float *minHornImpedance, float *maxHornImpedance,
                                     float *powerTrackingTolerance, float *frequencyTrackingTolerance,
                                     float *monitoringPeriod, float *timeout);
void Lattice_GetFrequencyTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf);
void Lattice_GetPowerTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf);
void Lattice_GetDestinationPower(float *destinationPower);

// Module control functions.
void Lattice_Start(void);
void Lattice_Execute(void);
void Lattice_Stop(void);

#endif