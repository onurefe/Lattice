#include "cli.h"
#include "rs485.h"
#include "cparser.h"
#include "lattice.h"

/* Private function prototypes ---------------------------------------------*/
static void sendMsg(char *buff, uint16_t length);

// Triggered callbacks.
static void factoryCallback(Dictionary_t *params);
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
static void getDestinationPowerCallback(Dictionary_t *params);
static void reportCallback(Dictionary_t *params);
static void resetCallback(Dictionary_t *params);
static void calibrateCallback(Dictionary_t *params);
static void getStatusCallback(Dictionary_t *params);
static void getErrorCallback(Dictionary_t *params);

/* Imported variables ------------------------------------------------------*/
extern UART_HandleTypeDef huart3;

/* Private variables -------------------------------------------------------*/
// Buffer for string manipulations.
static char Buffer[CLI_MAX_COMMAND_LINE_LENGTH];

// Trigger table.
static const Cp_Trigger_t TriggerTable[] =
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
            .name = "srcpms-s",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'S', .type = CP_PARAM_TYPE_INTEGER}},
            .callback = setSearchParametersCallback,
            .numOfParams = 2},

        // Get search parameters.
        {
            .name = "srcpms-g",
            .params = {},
            .callback = getSearchParametersCallback,
            .numOfParams = 0},

        // Set error detection parameters.
        {
            .name = "errdet-s",
            .params = {{.letter = 'N', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'X', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'F', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'R', .type = CP_PARAM_TYPE_REAL},
                       {.letter = 'T', .type = CP_PARAM_TYPE_REAL}},
            .callback = setErrorDetectionParametersCallback,
            .numOfParams = 6},

        // Get error detection parameters.
        {
            .name = "errdet-g",
            .params = {},
            .callback = getErrorDetectionParametersCallback,
            .numOfParams = 0},

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
            .name = "power-s",
            .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL}},
            .callback = setDestinationPowerCallback,
            .numOfParams = 1},

        // Get destination power.
        {
            .name = "power-g",
            .params = {},
            .callback = getDestinationPowerCallback,
            .numOfParams = 0},

        // Report command.
        {
            .name = "report",
            .params = {{.letter = 'S', .type = CP_PARAM_TYPE_INTEGER}},
            .callback = reportCallback,
            .numOfParams = 1},

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
            .params = {},
            .callback = getStatusCallback,
            .numOfParams = 0},

        // Get error command.
        {
            .name = "gerror",
            .params = {},
            .callback = getErrorCallback,
            .numOfParams = 0}};

/* Expoted functions -------------------------------------------------------*/
/***
 * @brief Initializes RS485 driver and registers trigger table.
 */
void Cli_Init(void)
{
    // Initialize rs485 low level driver.
    Rs485_Init(&huart3, RS485_DEnRE_GPIO_Port, RS485_DEnRE_Pin);

    // Register trigger table.
    Cp_Register(TriggerTable, (sizeof(TriggerTable) / sizeof(TriggerTable[0])));
}

/***
 * @brief Starts RS485 driver operation.
 */
void Cli_Start(void)
{
    Rs485_Start();
}

/***
 * @brief Handles pending events and applies routine controls.
 */
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
        Queue_DequeueArr(Rs485_GetRxBuffer(), (uint8_t *)line, line_ending_idx);

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

/***
 * @brief Stops the RS485. 
 */
void Cli_Stop(void)
{
    Rs485_Stop();
}

/* Private functions -------------------------------------------------------*/
/***
 * @brief Sends the message given.
 * 
 * @param buff: Pointer to the message buffer.
 * @param length: Length of the message.
 */
void sendMsg(char *buff, uint16_t length)
{
    // Transmit message.
    Rs485_Transmit(buff, length);

    // Transmit terminator.
    char term[] = CLI_LINE_TERMINATOR;
    Rs485_Transmit(term, sizeof(term));
}

/***
 * @brief Sends operation result message.
 *
 * @param success: If operation result is successfull or not.
 */
void operationResult(Bool_t success)
{
    uint8_t ret;
    uint8_t length;

    ret = success ? 1 : 0;

    // Send response message.
    length = snprintf(Buffer, sizeof(Buffer), "opres R%d", ret);
    sendMsg(Buffer, length);
}

/* Callback functions ------------------------------------------------------*/
/***
 * @brief Callback function which gets triggered when "factory" command is
 * received. Puts the device into factory configuration mode.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void factoryCallback(Dictionary_t *params)
{
    int32_t *password;
    password = (int32_t *)Dictionary_Get(params, 'P', NULL);

    if (password)
    {
        operationResult(Lattice_FactoryMode(*password));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback function which gets triggered when "cstr-s" command is
 * received. Sets the device constraints.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
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

/***
 * @brief Callback function which gets triggered when "cstr-g" command is
 * received. Gets the device constraints and returns them to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getConstraintsCallback(Dictionary_t *params)
{
    float max_power;
    float min_loading;
    float max_loading;
    uint8_t ret;
    uint8_t length;

    // Get constraints. If they are set, send response.
    ret = Lattice_GetConstraints(&max_power, &min_loading, &max_loading) ? 1 : 0;
    length = snprintf(Buffer, sizeof(Buffer), "cstr-gr R%d P%.2f N%.2f X%.2f",
                      ret, max_power, min_loading, max_loading);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "devinf-s" command is
 * received. Updates device information if in factory mode.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
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

/***
 * @brief Callback function which gets triggered when "devinf-g" command is
 * received. Gets the device information and sends it to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getDeviceInfoCallback(Dictionary_t *params)
{
    uint32_t deviceId;
    uint16_t versionMajor;
    uint16_t versionMinor;
    uint8_t length;
    uint8_t ret;
    
    ret = Lattice_GetDeviceInfo(&deviceId, &versionMajor, &versionMinor) ? 1 : 0;
    length = snprintf(Buffer, sizeof(Buffer), "devinf-gr R%d I%lu J%u N%u", (int)ret, deviceId,
                      (int)versionMajor, (int)versionMinor);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "srcprams-s" command is
 * received. Sets resonance search parameters. 
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
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

/***
 * @brief Callback function which gets triggered when "srcprams-g" command is
 * received. Get resonance search parameters and returns to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getSearchParametersCallback(Dictionary_t *params)
{
    float normalized_power;
    uint16_t steps;
    uint8_t length;
    uint8_t ret;

    ret = Lattice_GetSearchingParams(&normalized_power, &steps) ? 1 : 0;
    length = snprintf(Buffer, sizeof(Buffer), "srcpms-gr R%d P%.2f S%d",
                      ret, normalized_power, steps);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "errdet-s" command is
 * received. Sets error detection parameters.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void setErrorDetectionParametersCallback(Dictionary_t *params)
{
    float *min_horn_imp;
    float *max_horn_imp;
    float *power_track_tolerance;
    float *freq_track_tolerance;
    float *monitoring_period;
    float *timeout;

    // Parse parameters from the dictionary.
    min_horn_imp = (float *)Dictionary_Get(params, 'N', NULL);
    max_horn_imp = (float *)Dictionary_Get(params, 'X', NULL);
    power_track_tolerance = (float *)Dictionary_Get(params, 'P', NULL);
    freq_track_tolerance = (float *)Dictionary_Get(params, 'F', NULL);
    monitoring_period = (float *)Dictionary_Get(params, 'R', NULL);
    timeout = (float *)Dictionary_Get(params, 'T', NULL);

    // If all parameters parsed successfully, set parameters.
    if (min_horn_imp && max_horn_imp && power_track_tolerance && freq_track_tolerance &&
        monitoring_period && timeout)
    {
        operationResult(Lattice_SetErrorDetectionParams(*min_horn_imp, *max_horn_imp,
                                                        *power_track_tolerance, *freq_track_tolerance,
                                                        *monitoring_period, *timeout));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback function which gets triggered when "errdet-g" command is
 * received. Gets error detection parameters and sends it to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getErrorDetectionParametersCallback(Dictionary_t *params)
{
    float min_horn_imp;
    float max_horn_imp;
    float power_track_tolerance;
    float freq_track_tolerance;
    float monitoring_period;
    float timeout;
    uint8_t length;
    uint8_t ret;

    ret = Lattice_GetErrorDetectionParams(&min_horn_imp, &max_horn_imp, &power_track_tolerance,
                                          &freq_track_tolerance, &monitoring_period, &timeout);
    ret = ret ? 1 : 0;
    length = snprintf(Buffer, sizeof(Buffer), "errdet-gr R%d N%.2f X%.2f P%.2f F%.2f R%.2f T%.2f",
                      ret, min_horn_imp, max_horn_imp, power_track_tolerance,
                      freq_track_tolerance, monitoring_period, timeout);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "prpid-s" command is
 * received. Sets power tracking PID parameters.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
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

/***
 * @brief Callback function which gets triggered when "prpid-g" command is
 * received. Gets power tracking PID parameters and sends them to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getPowerTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float kp = 0.0f, ki = 0.0f, kd = 0.0f, tf = 0.0f;
    uint8_t length;
    uint8_t ret;

    ret = Lattice_GetFrequencyTrackingPidCoeffs(&kp, &ki, &kd, &tf) ? 1 : 0;
    length = snprintf(Buffer, sizeof(Buffer), "prpid-gr R%d P%.2f I%.2f D%.2f T%.2f",
                      ret, kp, ki, kd, tf);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "frpid-s" command is
 * received. Sets frequency tracking PID parameters.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
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

/***
 * @brief Callback function which gets triggered when "frpid-g" command is
 * received. Gets frequency tracking PID parameters and sends them to the 
 * host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getFrequencyTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float kp = 0.0f, ki = 0.0f, kd = 0.0f, tf = 0.0f;
    uint8_t length;
    uint8_t ret;

    ret = Lattice_GetFrequencyTrackingPidCoeffs(&kp, &ki, &kd, &tf) ? 1 : 0;
    length = snprintf(Buffer, sizeof(Buffer), "frpid-gr R%d P%.2f I%.2f D%.2f T%.2f",
                      ret, kp, ki, kd, tf);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "power-s" command is
 * received. Sets destination power parameter.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void setDestinationPowerCallback(Dictionary_t *params)
{
    float *dest_power;

    dest_power = (float *)Dictionary_Get(params, 'P', NULL);
    if (dest_power)
    {
        operationResult(Lattice_SetDestinationPower(*dest_power));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback function which gets triggered when "power-g" command is
 * received. Gets the destination power parameter and sends it to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getDestinationPowerCallback(Dictionary_t *params)
{
    float dest_power;
    uint8_t length;
    uint8_t ret;

    ret = Lattice_GetDestinationPower(&dest_power) ? 1 : 0;
    length = snprintf(Buffer, sizeof(Buffer), "power-gr R%d P%.2f",
                      ret, dest_power);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "report" command is
 * received. Enables/disables reporting function.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void reportCallback(Dictionary_t *params)
{
    int32_t *is_on;

    // Parse parameters.
    is_on = (int32_t *)Dictionary_Get(params, 'S', NULL);

    // If parsing is successfull, call measure function.
    if (is_on)
    {
        operationResult(Lattice_Report(*is_on));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback function which gets triggered when "reset" command is
 * received. Applies system reset.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void resetCallback(Dictionary_t *params)
{
    operationResult(TRUE);
    Lattice_Reset();
}

/***
 * @brief Callback function which gets triggered when "calibrate" command is
 * received. Start calibration operation.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void calibrateCallback(Dictionary_t *params)
{
    operationResult(Lattice_Calibrate());
}

/***
 * @brief Callback function which gets triggered when "gstat" command is
 * received. Gets the device status and sends it to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getStatusCallback(Dictionary_t *params)
{
    Lattice_Status_t status;
    uint8_t length;

    status = Lattice_GetStatus();
    length = snprintf(Buffer, sizeof(Buffer), "gstat-r S%d", status);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "gerror" command is
 * received. Gets the error code and sends it to the host.
 * 
 * @param params: Dictionary object holding pointers to the parsed parameters.
 */
void getErrorCallback(Dictionary_t *params)
{
    Lattice_Error_t error;
    uint8_t length;

    error = Lattice_GetError();
    length = snprintf(Buffer, sizeof(Buffer), "gerror-r E%d", error);

    sendMsg(Buffer, length);
}

/* Lattice callback implementations ----------------------------------------*/
/***
 * @brief Gets triggered when horn impedance is not between limits for 
 * <timeout> seconds.
 * 
 * @param hornImpedance: Impedance of the horn.
 */
void Lattice_HornImpedanceOutofWindowCallback(float hornImpedance)
{
    uint8_t length;
    length = snprintf(Buffer, sizeof(Buffer), "errhimp I%.1f", hornImpedance);

    sendMsg(Buffer, length);
}

/***
 * @brief Gets triggered when measure is not between limits for 
 * <timeout> seconds.
 * 
 * @param trackingMeasure: Tracking measure value.
 */
void Lattice_FrequencyTrackingFailureCallback(float trackingMeasure)
{
    uint8_t length;
    length = snprintf(Buffer, sizeof(Buffer), "errftra M%.2f", trackingMeasure);

    sendMsg(Buffer, length);
}

/***
 * @brief Gets triggered when power is not between limits for 
 * <timeout> seconds.
 * 
 * @param power: Power transmitted to the horn.
 */
void Lattice_PowerTrackingFailureCallback(float power)
{
    uint8_t length;
    length = snprintf(Buffer, sizeof(Buffer), "errptra P%.1f", power);

    sendMsg(Buffer, length);
}

/***
 * @brief Gets triggered at every monitoring event if reporting flag is
 * set. 
 * @param triggered: If the device is triggered(power output enabled) or not.
 * @param frequency: Current drive frequency.
 * @param duty: Current driver duty.
 * @param power: Complex power transmitted to the horn.
 * @param impedance: Complex impedance of the horn.
 */
void Lattice_ReportingCallback(Bool_t triggered, float frequency, float duty,
                               Complex_t *power, Complex_t *impedance)
{
    uint8_t length;
    uint8_t trg;
    trg = triggered ? 1 : 0;

    length = snprintf(Buffer, sizeof(Buffer), "report T%d F%.0f D%.2f P%.1f I%.1f M%.1f J%.1f",
                      trg, frequency, duty, power->real, power->img,
                      impedance->real, impedance->img);

    sendMsg(Buffer, length);
}