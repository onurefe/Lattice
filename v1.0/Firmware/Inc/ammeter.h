#ifndef __AMMETER_H
#define __AMMETER_H

#include "global.h"

/* Exported definitions ----------------------------------------------------*/
#define AMMETER_MAX_BREAK_OCCURRANCE 0.01f

/* Exported types ----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------*/
extern void Ammeter_Start(void);
extern void Ammeter_Execute(void);
extern void Ammeter_Stop(void);
extern void Ammeter_Measure(uint32_t driveSignalPeriod);
extern Bool_t Ammeter_IsBusy(void);

/* Callbacks ---------------------------------------------------------------*/
void Ammeter_MeasurementCompletedCb(float rmsCurrent);
void Ammeter_PitfallDetectedCb(void);

#endif