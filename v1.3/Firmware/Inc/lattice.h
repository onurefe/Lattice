#ifndef __LATTICE_H
#define __LATTICE_H

#include "global.h"

/* Exported types ----------------------------------------------------------*/
enum
{
    LATTICE_CONTROL_MODE_CALIBRATING = 0,
    LATTICE_CONTROL_MODE_RES_FREQ_SEARCHING = 1,
    LATTICE_CONTROL_MODE_RES_FREQ_TRACKING = 2
};
typedef uint8_t Lattice_ControlMode_t;

typedef struct
{
    Lattice_ControlMode_t controlMode : 2;
    Bool_t onTrack : 1;
    Bool_t powerOutput : 1;
} Lattice_Status_t;

/* Exported functions ------------------------------------------------------*/
extern void Lattice_Start(void);
extern void Lattice_Execute(void);
extern void Lattice_Stop(void);
extern void Lattice_CmdPowerOutput(Bool_t status);

/* Callbacks ---------------------------------------------------------------*/
void Lattice_StatusChangeCb(Lattice_Status_t *status);
void Lattice_CalibrationCompletedCb(void);

#endif