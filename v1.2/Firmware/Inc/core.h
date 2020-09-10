#ifndef __CORE_H
#define __CORE_H

#include "global.h"
#include "pid.h"
#include "complex.h"

/* Exported types ----------------------------------------------------------*/
enum
{
    CORE_STATUS_UNINIT = 0,
    CORE_STATUS_READY,
    CORE_STATUS_SCANNING,
    CORE_STATUS_SEARCHING,
    CORE_STATUS_TRACKING,
    CORE_STATUS_ERROR
};
typedef uint8_t Core_Status_t;

/* Exported functions ------------------------------------------------------*/
void Core_Init(Complex_t *calibrationCoeffs, float minFrequency, float maxFrequency,
               float minNormPower, float maxNormPower);
void Core_Scan(float startFrequency, float stopFrequency,
               uint16_t steps, float *impedanceReal, float *impedanceImg);
void Core_Search(float normalizedPower, float startFrequency, float stopFrequency,
                 uint16_t steps);
void Core_Track(Pid_Params_t *powerTrackingPidParams,
                Pid_Params_t *frequencyTrackingPidParams,
                float anchorFrequency, float fullPower, float destinationPower);
void Core_Execute(void);
void Core_Stop(void);
void Core_TriggerStatusUpdate(Bool_t triggered);
void Core_Monitor(Bool_t enabled, float period);
Core_Status_t Core_GetStatus(void);

/* Callbacks ---------------------------------------------------------------*/
void Core_ScanCompletedCallback(void);
void Core_SearchCompletedCallback(float resonanceFrequency, float resonanceImpedance,
                                  float fullPower);
void Core_MonitoringCallback(Bool_t triggered, float frequency, float duty,
                             Complex_t *power, Complex_t *impedance);

#endif