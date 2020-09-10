#include "cli.h"
#include "rs485.h"
#include "cparser.h"
#include "lattice.h"

/* Private typedefs --------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------*/
static void authorizeCallback(Dictionary_t *params);
static void setConstraintsCallback(Dictionary_t *params);
static void getConstraintsCallback(Dictionary_t *params);
static void setDeviceInfoCallback(Dictionary_t *params);
static void getDeviceInfoCallback(Dictionary_t *params);
static void setSearchParametersCallback(Dictionary_t *params);
static void getSearchParametersCallback(Dictionary_t *params);
static void setErrorDetectionParametersCallback(Dictionary_t *params);
static void getErrorDetectionParametersCallback(Dictionary_t *params);
static void setPowerTrackingPidCoeffsCallback(Dictionary_t *params);
static void getPowerTrackingPidCoeffsCallback(Dictionary_t *params);
static void setFrequencyTrackingPidCoeffsCallback(Dictionary_t *params);
static void getFrequencyTrackingPidCoeffsCallback(Dictionary_t *params);
static void setDestinationPowerCallback(Dictionary_t *params);
static void measureCallback(Dictionary_t *params);
static void resetCallback(Dictionary_t *params);
static void calibrateCallback(Dictionary_t *params);
static void getStatusCallback(Dictionary_t *params);
static void getErrorCallback(Dictionary_t *params);

/* Imported variables ------------------------------------------------------*/
extern UART_HandleTypeDef huart3;

/* Private variables -------------------------------------------------------*/
static Cp_Trigger_t TriggerTable[] =
    {
        // Factory mode.
        {
            .name = "factory",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_INTEGER}},
            .callback = factoryCallback,
            .numOfParams = 1},

        // Set constraints.
        {
            .name = "cstr-s",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},  // Maximum power.
                       {.letter = 'N', .type = CP_PARAM_TYPE_REAL},  // Minimum loading.
                       {.letter = 'X', .type = CP_PARAM_TYPE_REAL}}, // Maximum loading.
            .callback = setConstraintsCallback,
            .numOfParams = 3},

        // Get constraints.
        {
            .name = "cstr-g",
            .params = {},
            .callback = getConstraintsCallback,
            .numOfParams = 0},

        // Set device info.
        {
            .name = "devinf-s",
            .params = {{.letter = 'I', .type = CP_PARAM_TYPE_INTEGER},
                       {.letter = 'J', .type = CP_PARAM_TYPE_INTEGER},
                       {.letter = 'N', .type = CP_PARAM_TYPE_INTEGER}},
            .callback = setDeviceInfoCallback,
            .numOfParams = 3},

        // Get device info.
        {
            .name = "devinf-g",
            .params = {},
            .callback = getDeviceInfoCallback,
            .numOfParams = 0},

        // Set search parameters.
        {
            .name = "srcprams-s",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'S', .type = CP_PARAM_TYPE_INTEGER}},
            .callback = setSearchParametersCallback,
            .numOfParams = 2},

        // Get search parameters.
        {
            .name = "srcprams-g",
            .params = {},
            .callback = getSearchParametersCallback,
            .numOfParams = 0},

        // Set error detection parameters.
        {
            .name = "errdet-s",
            .params = {{.letter = 'N', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'M', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'V', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'X', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'T', .type = CP_PARAM_TYPE_REAL}},
            .callback = setErrorDetectionParametersCallback,
            .numOfParams = 5},

        // Get error detection parameters.
        {
            .name = "errdet-g",
            .params = {},
            .callback = getErrorDetectionParametersCallback,
            .numOfParams = 0
        },

        // Set power tracking PID coefficients.
        {
            .name = "pwpid-s",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'I', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'D', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'T', .type = CP_PARAM_TYPE_REAL}},
            .callback = setPowerTrackingPidCoeffsCallback,
            .numOfParams = 4},

        // Get power tracking PID coefficients.
        {
            .name = "pwpid-g",
            .params = {},
            .callback = getPowerTrackingPidCoeffsCallback,
            .numOfParams = 0},

        // Set frequency tracking PID coefficients.
        {
            .name = "frpid-s",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'I', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'D', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'T', .type = CP_PARAM_TYPE_REAL}},
            .callback = setFrequencyTrackingPidCoeffsCallback,
            .numOfParams = 4},

        // Get frequency tracking PID coefficients.
        {
            .name = "frpid-g",
            .params = {},
            .callback = getFrequencyTrackingPidCoeffsCallback,
            .numOfParams = 0},

        // Set destination power.
        {
            .name = "setpw",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL}},
            .callback = setDestinationPowerCallback,
            .numOfParams = 1},

        // Measurement command.
        {
            .name = "meas",
            .params = {{.letter = 'S', .type = CP_PARAM_TYPE_INTEGER},
                       {.letter = 'P', .type = CP_PARAM_TYPE_REAL}},
            .callback = measureCallback,
            .numOfParams = 2},

        // Calibrate command.
        {
            .name = "calibrate",
            .params = {},
            .callback = calibrateCallback,
            .numOfParams = 0},

        // Reset command.
        {
            .name = "reset",
            .params = {},
            .callback = resetCallback,
            .numOfParams = 0},

        // Get status command.
        {
            .name = "gstat",
            .param = {},
            .callback = getStatusCallback,
            .numOfParams = 0},

        // Get error command.
        {
            .name = "gerror",
            .param = {},
            .callback = getErrorCallback,
            .numOfParams = 0}};

/* Expoted functions -------------------------------------------------------*/
void Cli_Init(void)
{
    // Initialize rs485 low level driver.
    Rs485_Init(&huart3, RS485_DEnRE_GPIO_Port, RS485_DEnRE_Pin);

    // Register trigger table.
    Cp_Register(TriggerTable, (sizeof(TriggerTable) / sizeof(TriggerTable[0])));
}

void Cli_Start(void)
{
    Rs485_Start();
}

void Cli_Execute(void)
{
    char line_term[] = CLI_LINE_TERMINATOR;

    // Search for the line terminator.
    uint16_t line_ending_idx;
    line_ending_idx = Queue_SearchArr(Rs485_GetRxBuffer(), (uint8_t *)line_term,
                                      sizeof(line_term));

    // If there exists a line terminator; feed line to command parser.
    if (line_ending_idx != 0xFFFF)
    {
        char line[CLI_MAX_COMMAND_LINE_LENGTH];

        // Parse line.
        Queue_DequeueArr(Rs485_GetRxBuffer(), line, line_ending_idx);

        // Feed line to the command parser.
        if (Cp_FeedLine(line, line_ending_idx) == TRUE)
        {
            // Send ok if the command parsed successfully.
            char rsp[] = {'o', 'k'};

            Rs485_Transmit(rsp, sizeof(rsp));
            Rs485_Transmit(line_term, sizeof(line_term));
        }

        // Remove line terminator in the buffer.
        Queue_Remove(Rs485_GetRxBuffer(), sizeof(line_term));
    }
}

void Cli_SendMsg(char *buff, uint8_t length)
{
    // Transmit message.
    Rs485_Transmit(buff, length);

    // Transmit terminator.
    char term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(term, sizeof(term));
}

void Cli_Stop(void)
{
    Rs485_Stop();
}

/* Private functions -------------------------------------------------------*/
void operationResult(Bool_t success)
{
    char buff[16];
    uint8_t ret;
    uint8_t length;

    if (success)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }

    // Send response message.
    length = snprintf(buff, sizeof(buff), "opres R%d", ret);
    Rs485_Transmit(buff, length);

    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}

/* Callback functions ------------------------------------------------------*/
void factoryCallback(Dictionary_t *params)
{
    int32_t *password;
    password = (int32_t *)Dictionary_Get(params, 'P', NULL);

    if (password)
    {
        operationResult(Lattice_Factory(*password));
    }
    else
    {
        operationResult(FALSE);
    }
}

void setConstraintsCallback(Dictionary_t *params)
{
    float *max_power;
    float *min_loading;
    float *max_loading;

    // Parse parameters from the dictionary.
    max_power = (float *)Dictionary_Get(params, 'P', NULL);
    min_loading = (float *)Dictionary_Get(params, 'N', NULL);
    max_loading = (float *)Dictionary_Get(params, 'X', NULL);

    // If every parameter parsed successfully, set the constraints.
    if (max_power && min_loading && max_loading)
    {
        operationResult(Lattice_SetConstraints(*max_power, *min_loading,
                                               *max_loading));
    }
    else
    {
        operationResult(FALSE);
    }
}

void getConstraintsCallback(Dictionary_t *params)
{
    float max_power;
    float min_loading;
    float max_loading;
    char buff[64];
    uint8_t ret;
    uint8_t length;

    // Get constraints. If they are set, send response.
    if (Lattice_GetConstraints(&max_power, &min_loading, &max_loading))
    {
        ret = 1;
        length = snprintf(buff, "cstr-gr R%d P%f N%f X%f", sizeof(buff),
                          ret, max_power, min_loading, max_loading);
    }
    else
    {
        ret = 0;
        length = snprintf(buff, "cstr-gr R%d", sizeof(buff), ret);
    }

    // Send response.
    Rs485_Transmit(buff, length);

    // Send line terminator.
    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}

void setDeviceInfoCallback(Dictionary_t *params)
{
    int32_t *deviceId;
    int32_t *versionMajor;
    int32_t *versionMinor;

    // Parse parameters from the dictionary.
    deviceId = (int32_t *)Dictionary_Get(params, 'I', NULL);
    versionMajor = (int32_t *)Dictionary_Get(params, 'J', NULL);
    versionMinor = (int32_t *)Dictionary_Get(params, 'N', NULL);

    // If every parameter parsed successfully, set device info.
    if (deviceId && versionMinor && versionMajor)
    {
        operationResult(Lattice_SetDeviceInfo(*deviceId, *versionMajor,
                                              *versionMinor));
    }
    else
    {
        operationResult(FALSE);
    }
}

void getDeviceInfoCallback(Dictionary_t *params)
{
    int32_t deviceId;
    int32_t versionMajor;
    int32_t versionMinor;
    char buff[64];
    uint8_t length;
    uint8_t ret;

    if (Lattice_GetDeviceInfo(&deviceId, &versionMajor, &versionMinor))
    {
        ret = 1;
        length = snprintf(buff, "devinf-gr R%d I%f J%f N%f", sizeof(buff),
                          ret, deviceId, versionMajor, versionMinor);
    }
    else
    {
        ret = 0;
        length = snprintf(buff, "devinf-gr R%d", sizeof(buff), ret);
    }

    // Send response.
    Rs485_Transmit(buff, length);

    // Send line terminator.
    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}

void setSearchParametersCallback(Dictionary_t *params)
{
    float *normalized_power;
    int32_t *steps;

    // Parse parameters from the dictionary.
    normalized_power = (float *)Dictionary_Get(params, 'P', NULL);
    steps = (int32_t *)Dictionary_Get(params, 'S', NULL);

    // Set searching parameters if parsing is successfull.
    if (normalized_power && steps)
    {
        operationResult(Lattice_SetSearchingParams(*normalized_power,
                                                   (uint16_t)*steps));
    }
    else
    {
        operationResult(FALSE);
    }
}

void getSearchParametersCallback(Dictionary_t *params)
{
    float normalized_power;
    int32_t steps;
    char buff[64];
    uint8_t length;
    uint8_t ret;

    if (Lattice_GetSearchingParams(&normalized_power, &steps))
    {
        ret = 1;
        length = snprintf(buff, "srcprams-gr R%d P%d S%f", sizeof(buff),
                          ret, normalized_power, steps);
    }
    else
    {
        ret = 0;
        length = snprintf(buff, "srcprams-gr R%d", sizeof(buff), ret);
    }

    // Send response.
    Rs485_Transmit(buff, length);

    // Send line terminator.
    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}

void setErrorDetectionParametersCallback(Dictionary_t *params)
{
}

void getErrorDetectionParametersCallback(Dictionary_t *params)
{
}

void setPowerTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float *kp;
    float *ki;
    float *kd;
    float *tf;

    // Parse parameters from the dictionary.
    kp = (float *)Dictionary_Get(params, 'P', NULL);
    ki = (float *)Dictionary_Get(params, 'I', NULL);
    kd = (float *)Dictionary_Get(params, 'D', NULL);
    tf = (float *)Dictionary_Get(params, 'T', NULL);

    // Set power tracking PID parameters.
    if (kp && ki && kd && tf)
    {
        operationResult(Lattice_SetPowerTrackingPidCoeffs(*kp, *ki, *kd, *tf));
    }
    else
    {
        operationResult(FALSE);
    }
}

void getPowerTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float kp, ki, kd, tf;
    char buff[64];
    uint8_t length;
    uint8_t ret;

    if (Lattice_GetFrequencyTrackingPidCoeffs(&kp, &ki, &kd, &tf))
    {
        ret = 1;
        length = snprintf(buff, "prpid-gr R%d P%d I%d D%f T%f", sizeof(buff),
                          ret, kp, ki, kd, tf);
    }
    else
    {
        ret = 0;
        length = snprintf(buff, "prpid-gr R%d", sizeof(buff), ret);
    }

    // Send response.
    Rs485_Transmit(buff, length);

    // Send line terminator.
    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}

void setFrequencyTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float *kp;
    float *ki;
    float *kd;
    float *tf;

    // Parse parameters from the dictionary.
    kp = (float *)Dictionary_Get(params, 'P', NULL);
    ki = (float *)Dictionary_Get(params, 'I', NULL);
    kd = (float *)Dictionary_Get(params, 'D', NULL);
    tf = (float *)Dictionary_Get(params, 'T', NULL);

    // Set frequency tracking PID parameters.
    if (kp && ki && kd && tf)
    {
        operationResult(Lattice_SetFrequencyTrackingPidCoeffs(*kp, *ki,
                                                              *kd, *tf));
    }
    else
    {
        operationResult(FALSE);
    }
}

void getFrequencyTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float kp, ki, kd, tf;
    char buff[64];
    uint8_t length;
    uint8_t ret;

    if (Lattice_GetFrequencyTrackingPidCoeffs(&kp, &ki, &kd, &tf))
    {
        ret = 1;
        length = snprintf(buff, "frpid-gr R%d P%d I%d D%f T%f", sizeof(buff),
                          ret, kp, ki, kd, tf);
    }
    else
    {
        ret = 0;
        length = snprintf(buff, "frpid-gr R%d", sizeof(buff), ret);
    }

    // Send response.
    Rs485_Transmit(buff, length);

    // Send line terminator.
    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}

void setDestinationPowerCallback(Dictionary_t *params)
{
    float *output_power;

    output_power = (float *)Dictionary_Get(params, 'P', NULL);
    if (output_power)
    {
        operationResult(Lattice_SetDestinationPower(*output_power));
    }
    else
    {
        operationResult(FALSE);
    }
}

void measureCallback(Dictionary_t *params)
{
    int32_t *is_on;
    float *measurement_period;

    // Parse parameters.
    is_on = (int32_t *)Dictionary_Get(params, 'S', NULL);
    measurement_period = (float *)Dictionary_Get(params, 'P', NULL);

    // If parsing is successfull, call measure function.
    if (is_on && measurement_period)
    {
        operationResult(Lattice_Measure(*is_on, *measurement_period));
    }
    else
    {
        operationResult(FALSE);
    }
}

void resetCallback(Dictionary_t *params)
{
    Lattice_Reset();
    operationResult(TRUE);
}

void calibrateCallback(Dictionary_t *params)
{
    operationResult(Lattice_Calibrate());
}

void getStatusCallback(Dictionary_t *params)
{
    Lattice_Status_t status;
    char buff[16];
    uint8_t length;

    status = Lattice_GetStatus();
    length = snprintf(buff, "gstat-r S%d", sizeof(buff), status);

    // Send response.
    Rs485_Transmit(buff, length);

    // Send line terminator.
    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}

void getErrorCallback(Dictionary_t *params)
{
    Lattice_Error_t error;
    char buff[16];
    uint8_t length;

    error = Lattice_GetError();
    length = snprintf(buff, "gerror-r E%d", sizeof(buff), error);

    // Send response.
    Rs485_Transmit(buff, length);

    // Send line terminator.
    char line_term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(line_term, sizeof(line_term));
}