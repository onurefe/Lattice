#ifndef __LATTICE_H
#define __LATTICE_H

#include "global.h"
#include "core.h"

/* Exported types ----------------------------------------------------------*/
enum
{
    LATTICE_STATUS_READY = 0,
    LATTICE_STATUS_PENDING_FACTORY_INITIALIZATION,
    LATTICE_STATUS_OPERATING,
    LATTICE_STATUS_ERROR
}; 
typedef uint8_t Lattice_Status_t;

enum
{
    LATTICE_ERROR_HALF_BRIDGE_FAILURE = 0,
    LATTICE_ERROR_HORN_FAILURE,
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
Bool_t Lattice_SetPowerTrackingPidCoeffs(float kp, float ki, float kd, float tf);
Bool_t Lattice_GetPowerTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf);
Bool_t Lattice_SetFrequencyTrackingPidCoeffs(float kp, float ki, float kd, float tf);
Bool_t Lattice_GetFrequencyTrackingPidCoeffs(float *kp, float *ki, float *kd, float *tf);

// Cli control functions.
Bool_t Lattice_Authorize(int32_t password);
void Lattice_Reset(void);
void Lattice_Calibrate(void);
void Lattice_Measure(Bool_t isOn, float measurementPeriod);
Lattice_Status_t Lattice_GetStatus(void);
Lattice_Error_t Lattice_GetError(void);

// Module control functions.
void Lattice_Start(void);
void Lattice_Execute(void);
void Lattice_Stop(void);

#endif