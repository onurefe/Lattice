#include "cli.h"
#include "rs485.h"
#include "cparser.h"
#include "lattice.h"

/* Private function prototypes ---------------------------------------------*/
static void sendMsg(char *buff, uint16_t length);

// Triggered callbacks.
static void deviceInfoCallback(Dictionary_t *params);
static void configCallback(Dictionary_t *params);
static void defaultsCallback(Dictionary_t *params);
static void resetCallback(Dictionary_t *params);
static void getStatusCallback(Dictionary_t *params);
static void getErrorCallback(Dictionary_t *params);
static void calibrateCallback(Dictionary_t *params);
static void reportCallback(Dictionary_t *params);
static void setCalibrationPolynomialCallback(Dictionary_t *params);
static void setConstraintsCallback(Dictionary_t *params);
static void setSearchParametersCallback(Dictionary_t *params);
static void setErrorDetectionParametersCallback(Dictionary_t *params);
static void setPowerTrackingPidCoeffsCallback(Dictionary_t *params);
static void setFrequencyTrackingPidCoeffsCallback(Dictionary_t *params);
static void setDestinationPowerCallback(Dictionary_t *params);
static void getCalibrationPolynomialCallback(Dictionary_t *params);
static void getConstraintsCallback(Dictionary_t *params);
static void getSearchParametersCallback(Dictionary_t *params);
static void getErrorDetectionParametersCallback(Dictionary_t *params);
static void getPowerTrackingPidCoeffsCallback(Dictionary_t *params);
static void getFrequencyTrackingPidCoeffsCallback(Dictionary_t *params);
static void getDestinationPowerCallback(Dictionary_t *params);

/* Imported variables ------------------------------------------------------*/
extern UART_HandleTypeDef huart3;

/* Private variables -------------------------------------------------------*/
// Buffer for string manipulations.
static char Buffer[CLI_MAX_COMMAND_LINE_LENGTH];

// Trigger table.
static const Cp_Command_t CommandTable[] =
    {
        {.name = "devinf",
         .params = {},
         .callback = deviceInfoCallback,
         .numOfParams = 0},

        {.name = "config",
         .params = {{.letter = 'P', .type = CP_PARAM_TYPE_INTEGER}},
         .callback = configCallback,
         .numOfParams = 1},

        {.name = "defaults",
         .params = {},
         .callback = defaultsCallback,
         .numOfParams = 0},

        {.name = "reset",
         .params = {},
         .callback = resetCallback,
         .numOfParams = 0},

        {.name = "gstatus",
         .params = {},
         .callback = getStatusCallback,
         .numOfParams = 0},

        {.name = "gerror",
         .params = {},
         .callback = getErrorCallback,
         .numOfParams = 0},

        {.name = "calibrate",
         .params = {},
         .callback = calibrateCallback,
         .numOfParams = 0},

        {.name = "report",
         .params = {},
         .callback = reportCallback,
         .numOfParams = 0},

        {.name = "calpoly-s",
         .params = {{.letter = 'R', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'E', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'A', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'I', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'M', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'G', .type = CP_PARAM_TYPE_REAL}},
         .callback = setCalibrationPolynomialCallback,
         .numOfParams = 6},

        {.name = "constr-s",
         .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'F', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'R', .type = CP_PARAM_TYPE_REAL}},
         .callback = setConstraintsCallback,
         .numOfParams = 3},

        {.name = "srcpms-s",
         .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'S', .type = CP_PARAM_TYPE_INTEGER}},
         .callback = setSearchParametersCallback,
         .numOfParams = 2},

        {.name = "errdet-s",
         .params = {{.letter = 'N', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'X', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'F', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'O', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'T', .type = CP_PARAM_TYPE_REAL}},
         .callback = setErrorDetectionParametersCallback,
         .numOfParams = 6},

        {.name = "pwpid-s",
         .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'I', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'D', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'T', .type = CP_PARAM_TYPE_REAL}},
         .callback = setPowerTrackingPidCoeffsCallback,
         .numOfParams = 4},

        {.name = "frpid-s",
         .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'I', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'D', .type = CP_PARAM_TYPE_REAL},
                    {.letter = 'T', .type = CP_PARAM_TYPE_REAL}},
         .callback = setFrequencyTrackingPidCoeffsCallback,
         .numOfParams = 4},

        {.name = "destpow-s",
         .params = {{.letter = 'P', .type = CP_PARAM_TYPE_REAL}},
         .callback = setDestinationPowerCallback,
         .numOfParams = 1},

        {.name = "calpoly-g",
         .params = {},
         .callback = getCalibrationPolynomialCallback,
         .numOfParams = 0},

        {.name = "constr-g",
         .params = {},
         .callback = getConstraintsCallback,
         .numOfParams = 0},

        {.name = "srcpms-g",
         .params = {},
         .callback = getSearchParametersCallback,
         .numOfParams = 0},

        {.name = "errdet-g",
         .params = {},
         .callback = getErrorDetectionParametersCallback,
         .numOfParams = 0},

        {.name = "pwpid-g",
         .params = {},
         .callback = getPowerTrackingPidCoeffsCallback,
         .numOfParams = 0},

        {.name = "frpid-g",
         .params = {},
         .callback = getFrequencyTrackingPidCoeffsCallback,
         .numOfParams = 0},

        {.name = "destpow-g",
         .params = {},
         .callback = getDestinationPowerCallback,
         .numOfParams = 0}};

/* Expoted functions -------------------------------------------------------*/
/***
 * @brief Initializes RS485 driver and registers trigger table.
 */
void Cli_Init(void)
{
    // Initialize rs485 low level driver.
    Rs485_Init(&huart3, RS485_DEnRE_GPIO_Port, RS485_DEnRE_Pin);

    // Register command table.
    Cp_Register(CommandTable, (sizeof(CommandTable) / sizeof(CommandTable[0])));
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
        if (Cp_FeedLine(line, line_ending_idx) != TRUE)
        {
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
    length = snprintf(Buffer, sizeof(Buffer), "R%d", ret);
    sendMsg(Buffer, length);
}

/* Callback functions ------------------------------------------------------*/
/***
 * @brief Callback function which gets triggered when devinfo command is received.
 */
void deviceInfoCallback(Dictionary_t *params)
{
    uint8_t length;
    length = snprintf(Buffer, sizeof(Buffer), "F%.1f R%.1f P%.1f N%d J%d", MIN_FREQUENCY,
                      MAX_FREQUENCY, MAX_OUTPUT_POWER, FIRMWARE_VERSION_MINOR,
                      FIRMWARE_VERSION_MAJOR);
    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "config" command is
 * received. Puts the device into factory configuration mode.
 */
void configCallback(Dictionary_t *params)
{
    int32_t *password;
    password = (int32_t *)Dictionary_Get(params, 'P', NULL);

    if (password)
    {
        operationResult(Lattice_ConfigMode(*password));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback triggered when defaults command is received.
 */
void defaultsCallback(Dictionary_t *params)
{
    operationResult(Lattice_LoadDefaults());
}

/***
 * @brief Callback triggered when reset command is received.
 */
void resetCallback(Dictionary_t *params)
{
    Lattice_Reset();
}

/***
 * @brief Callback function which gets triggered when "gstat" command is
 * received. Gets the device status and sends it to the host.
 */
void getStatusCallback(Dictionary_t *params)
{
    Lattice_Status_t status;
    uint8_t length;

    status = Lattice_GetStatus();
    length = snprintf(Buffer, sizeof(Buffer), "S%d", status);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "gerror" command is
 * received. Gets the error code and sends it to the host.
 */
void getErrorCallback(Dictionary_t *params)
{
    Lattice_Error_t error;
    uint8_t length;

    error = Lattice_GetError();
    length = snprintf(Buffer, sizeof(Buffer), "E%d", error);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "calibrate" command is
 * received. Start calibration operation.
 */
void calibrateCallback(Dictionary_t *params)
{
    operationResult(Lattice_Calibrate());
}

/***
 * @brief Callback function which gets triggered when "report" command is
 * received. Enables/disables reporting function.
 */
void reportCallback(Dictionary_t *params)
{
    uint8_t length;
    uint8_t status;
    uint8_t trigger_status;
    float frequency;
    float duty;
    float power_real;
    float power_img;
    float imp_real;
    float imp_img;

    Lattice_GetReport(&status, &trigger_status, &frequency, &duty, &power_real, &power_img,
                      &imp_real, &imp_img);

    length = snprintf(Buffer, sizeof(Buffer), "S%d T%d F%f D%f W%f Q%f R%f I%f", status,
                      trigger_status, frequency, duty, power_real, power_img, imp_real, imp_img);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "calpoly-s"
 * command is received.
 */
void setCalibrationPolynomialCallback(Dictionary_t *params)
{
    float *a0;
    float *a1;
    float *a2;
    float *b0;
    float *b1;
    float *b2;

    a0 = (float *)Dictionary_Get(params, 'R', NULL);
    a1 = (float *)Dictionary_Get(params, 'E', NULL);
    a2 = (float *)Dictionary_Get(params, 'A', NULL);
    b0 = (float *)Dictionary_Get(params, 'I', NULL);
    b1 = (float *)Dictionary_Get(params, 'M', NULL);
    b2 = (float *)Dictionary_Get(params, 'G', NULL);

    if (a0 && a1 && a2 && b0 && b1 && b2)
    {
        operationResult(Lattice_SetCalibrationPolynomial(*a0, *a1, *a2, *b0, *b1, *b2));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback function which gets triggered when "cstr-s" command is
 * received. Sets the device constraints.
 */
void setConstraintsCallback(Dictionary_t *params)
{
    float *max_power;
    float *min_frequency;
    float *max_frequency;

    // Parse parameters from the dictionary.
    max_power = (float *)Dictionary_Get(params, 'P', NULL);
    min_frequency = (float *)Dictionary_Get(params, 'F', NULL);
    max_frequency = (float *)Dictionary_Get(params, 'R', NULL);

    // If every parameter parsed successfully, set the constraints.
    if (max_power && min_frequency && max_frequency)
    {
        operationResult(Lattice_SetConstraints(*max_power, *min_frequency, *max_frequency));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback function which gets triggered when "srcprams-s" command is
 * received. Sets resonance search parameters. 
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
 * @brief Callback function which gets triggered when "errdet-s" command is
 * received. Sets error detection parameters.
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
    monitoring_period = (float *)Dictionary_Get(params, 'O', NULL);
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
 * @brief Callback function which gets triggered when "prpid-s" command is
 * received. Sets power tracking PID parameters.
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
 * @brief Callback function which gets triggered when "frpid-s" command is
 * received. Sets frequency tracking PID parameters.
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
        operationResult(Lattice_SetFrequencyTrackingPidCoeffs(*kp, *ki, *kd, *tf));
    }
    else
    {
        operationResult(FALSE);
    }
}

/***
 * @brief Callback function which gets triggered when "power-s" command is
 * received. Sets destination power parameter.
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
 * @brief Callback function which gets triggered when calpoly-s command
 * is received.
 */
void getCalibrationPolynomialCallback(Dictionary_t *params)
{
    float a0, a1, a2;
    float b0, b1, b2;
    uint8_t length;

    Lattice_GetCalibrationPolynomial(&a0, &a1, &a2, &b0, &b1, &b2);

    length = snprintf(Buffer, sizeof(Buffer), "R%.4f E%.4f A%.4f I%.4f M%.4f G%.4f",
                      a0, a1, a2, b0, b1, b2);
    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "cstr-g" command is
 * received. Gets the device constraints and returns them to the host.
 */
void getConstraintsCallback(Dictionary_t *params)
{
    float max_power;
    float min_freq;
    float max_freq;
    uint8_t length;

    // Get constraints. If they are set, send response.
    Lattice_GetConstraints(&max_power, &min_freq, &max_freq);
    length = snprintf(Buffer, sizeof(Buffer), "P%.1f F%.1f R%.1f",
                      max_power, min_freq, max_freq);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "srcprams-g" command is
 * received. Get resonance search parameters and returns to the host.
 */
void getSearchParametersCallback(Dictionary_t *params)
{
    float normalized_power;
    uint16_t steps;
    uint8_t length;

    Lattice_GetSearchingParams(&normalized_power, &steps);

    length = snprintf(Buffer, sizeof(Buffer), "P%.2f S%d",
                      normalized_power, steps);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "errdet-g" command is
 * received. Gets error detection parameters and sends it to the host.
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

    Lattice_GetErrorDetectionParams(&min_horn_imp, &max_horn_imp, &power_track_tolerance,
                                    &freq_track_tolerance, &monitoring_period, &timeout);

    length = snprintf(Buffer, sizeof(Buffer), "N%.1f X%.1f P%.2f F%.2f O%.1f T%.1f",
                      min_horn_imp, max_horn_imp, power_track_tolerance,
                      freq_track_tolerance, monitoring_period, timeout);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "prpid-g" command is
 * received. Gets power tracking PID parameters and sends them to the host.
 */
void getPowerTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float kp = 0.0f, ki = 0.0f, kd = 0.0f, tf = 0.0f;
    uint8_t length;

    Lattice_GetFrequencyTrackingPidCoeffs(&kp, &ki, &kd, &tf);
    length = snprintf(Buffer, sizeof(Buffer), "P%.3f I%.3f D%.3f T%.3f",
                      kp, ki, kd, tf);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "frpid-g" command is
 * received. Gets frequency tracking PID parameters and sends them to the 
 * host.
 */
void getFrequencyTrackingPidCoeffsCallback(Dictionary_t *params)
{
    float kp = 0.0f, ki = 0.0f, kd = 0.0f, tf = 0.0f;
    uint8_t length;

    Lattice_GetFrequencyTrackingPidCoeffs(&kp, &ki, &kd, &tf);
    length = snprintf(Buffer, sizeof(Buffer), "P%.3f I%.3f D%.3f T%.3f",
                      kp, ki, kd, tf);

    sendMsg(Buffer, length);
}

/***
 * @brief Callback function which gets triggered when "power-g" command is
 * received. Gets the destination power parameter and sends it to the host.
 */
void getDestinationPowerCallback(Dictionary_t *params)
{
    float dest_power;
    uint8_t length;

    Lattice_GetDestinationPower(&dest_power);
    length = snprintf(Buffer, sizeof(Buffer), "P%.1f", dest_power);

    sendMsg(Buffer, length);
}