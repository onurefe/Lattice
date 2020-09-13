#ifndef __PID_H
#define __PID_H

#include "global.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* Exported types ----------------------------------------------------------*/
    typedef struct
    {
        float kp;
        float ki;
        float kd;
        float tfilt;
    } Pid_Params_t;

    typedef struct
    {
        float kp;
        float ki;
        float kd;
        float tf;
        float lastInput;
        float integral;
        float minVal;
        float maxVal;
    } Pid_t;

    /* Exported functions ------------------------------------------------------*/
    extern void Pid_Setup(Pid_t *pid, Pid_Params_t *params, float minVal, float maxVal);
    extern float Pid_Exe(Pid_t *pid, float input, float dt);
    extern void Pid_Reset(Pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif