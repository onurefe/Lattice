#ifndef __PIEZO_DRIVER_H
#define __PIEZO_DRIVER_H

#include "global.h"

/* Exported definitions ----------------------------------------------------*/
#define PIEZO_DRIVER_MAX_BREAK_COUNT 

/* Exported functions ------------------------------------------------------*/
extern void Pd_CmdSignalGeneration(Bool_t enabled);
extern void Pd_SetFrequency(float frequency);
extern void Pd_SetPeriod(uint32_t period);

#endif