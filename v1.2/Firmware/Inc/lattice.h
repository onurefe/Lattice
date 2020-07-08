#ifndef __LATTICE_H
#define __LATTICE_H

#include "global.h"

/* Exported types ----------------------------------------------------------*/
enum
{
    LATTICE_CONTROL_MODE_RES_FREQ_SEARCHING = 0,
    LATTICE_CONTROL_MODE_RES_FREQ_TRACKING = 1
};
typedef uint8_t Lattice_ControlMode_t;

typedef struct 
{
    Lattice_ControlMode_t controlMode:1;
    Bool_t resonanceTrackingOnTrack:1;
    Bool_t powerTrackingOnTrack:1;
    Bool_t loaded:1;
} Lattice_Status_t;

/* Exported functions ------------------------------------------------------*/
extern void Lattice_Start(void);
extern void Lattice_Execute(void);
extern void Lattice_Stop(void);

/* Callbacks ---------------------------------------------------------------*/
void Lattice_StatusChangeCb(Lattice_Status_t *status);

#endif