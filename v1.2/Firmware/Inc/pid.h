#ifndef __PID_H
#define __PID_H

#include "global.h"

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

/* Inline functions --------------------------------------------------------*/
inline void Pid_Setup(Pid_t *pid, Pid_Params_t *params, float minVal, float maxVal)
{
    pid->kp = params->kp;
    pid->ki = params->ki;
    pid->kd = params->kd;
    pid->tf = params->tfilt;
    pid->minVal = minVal;
    pid->maxVal = maxVal;
    pid->integral = 0.0f;
    pid->lastInput = 0.0f;
}

inline float Pid_Exe(Pid_t *pid, float input, float dt)
{
    float cfilt;
    float in;
    float out;

    // Calculate input filter coefficient.
    cfilt = dt / pid->tf;

    // Calculate filtered input.
    in = (1.0f - cfilt) * pid->lastInput + cfilt * input;

    // Filter input value.
    out = in * pid->kp;
    out += pid->integral * pid->ki;
    out += ((in - pid->lastInput) / dt) * pid->kd;

    Bool_t windup_protect = FALSE;

    if (out > pid->maxVal)
    {
        out = pid->maxVal;
        windup_protect = (in >= 0.0f);
    }

    if (out < pid->minVal)
    {
        out = pid->minVal;
        windup_protect = (windup_protect || (in <= 0.0f));
    }

    // If no windup occurred; update the integral term.
    if (!windup_protect)
    {
        pid->integral += (dt * in);
    }

    pid->lastInput = in;

    return out;
}

#endif