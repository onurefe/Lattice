#include "../inc/pid.h"

/* Exported functions ------------------------------------------------------*/
/***
 * @brief Sets up the PID block.
 * 
 * @param pid: Pointer to the PID module.
 * @param params: Pointer to the PID parameter struct.
 * @param minVal: Minimum output value.
 * @param maxVal: Maximum output value.
 */
void Pid_Setup(Pid_t *pid, Pid_Params_t *params, float minVal, float maxVal)
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

/***
 * @brief Updates PID block internals and output.
 * 
 * @param pid: Pointer to the PID module.
 * @param input: Input signal.
 * @param dt: Time change since last call.
 * 
 * @retval Output signal.
 */
float Pid_Exe(Pid_t *pid, float input, float dt)
{
    float cfilt;
    float in;
    float out;

    // Calculate input filter coefficient.
    cfilt = dt / (dt + pid->tf);

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

/***
 * @brief Resets the PID module.
 * 
 * @param pid: Pointer to the PID module.
 */
void Pid_Reset(Pid_t *pid)
{
    pid->integral = 0.0f;
    pid->lastInput = 0.0f;
}