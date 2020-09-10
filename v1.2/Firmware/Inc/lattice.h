#ifndef __LATTICE_H
#define __LATTICE_H

#include "global.h"
#include "core.h"

/* Exported types ----------------------------------------------------------*/
enum
{
    LATTICE_STATUS_FACTORY_CONFIG = 0,
    LATTICE_STATUS_FACTORY_CONFIG_CALIBRATING,
    LATTICE_STATUS_OPERATING_SEARCHING,
    LATTICE_STATUS_OPERATING_TRACKING,
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
// Functions to manipulate factory data.
Bool_t Lattice_SetConstraints(float maxPower, float minLoading, float maxLoading);
Bool_t Lattice_GetConstraints(float *maxPower, float *minLoading, float *maxLoading);
Bool_t Lattice_SetDeviceInfo(uint32_t deviceId, uint16_t versionMajor, uint16_t versionMinor);
Bool_t Lattice_GetDeviceInfo(uint32_t *deviceId, uint16_t *versionMajor, uint16_t *versionMinor);
Bool_t Lattice_SetSearchingParams(float normalizedPower, uint16_t steps);
Bool_t Lattice_GetSearchingParams(float *normalizedPower, uint16_t *steps);
Bool_t Lattice_SetErrorDetectionParams(float minHornImpedance, float maxHornImpedance,
                                       float powerTrackingTolerance, float frequencyTrackingTolerance,
                                       float timeout);
Bool_t Lattice_GetErrorDetectionParams(float *minHornImpedance, float *maxHornImpedance,
                                       float *powerTrackingTolerance, float *frequencyTrackingTolerance,
                                       float *timeout);
Bool_t Lattice_SetMonitoringPeriod(float period);
Bool_t Lattice_GetMonitoringPeriod(float *period);
Bool_t Lattice_SetPowerTrackingPidCoeffs(float kp, float ki, float kd, float tf);
Bool_t Lattice_GetPowerTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf);
Bool_t Lattice_SetFrequencyTrackingPidCoeffs(float kp, float ki, float kd, float tf);
Bool_t Lattice_GetFrequencyTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf);

// Cli control functions.
Bool_t Lattice_FactoryMode(int32_t password);
Bool_t Lattice_Calibrate(void);
void Lattice_Reset(void);
Bool_t Lattice_SetDestinationPower(float destionationPower);
Bool_t Lattice_Report(Bool_t isOn);
Lattice_Status_t Lattice_GetStatus(void);
Lattice_Error_t Lattice_GetError(void);

// Module control functions.
void Lattice_Start(void);
void Lattice_Execute(void);
void Lattice_Stop(void);

#endif