#ifndef __SYNCHRONOUS_RECTIFIER
#define __SYNCHRONOUS_RECTIFIER

#include "global.h"

/* Exported functions ------------------------------------------------------*/
extern void Sr_Start(void);
extern void Sr_Stop(void);
extern void Sr_SetVoltage(float normalizedVoltage);

#endif