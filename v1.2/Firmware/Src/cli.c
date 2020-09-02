#include "cli.h"
#include "rs485.h"
#include "cparser.h"

/* Private typedefs --------------------------------------------------------*/
enum
{
    VAR_DESTINATION_POWER = 'P',
    VAR_TRACKING_WINDOW = 'T',
    VAR_ANCHOR_FREQUENCY = 'A',
    VAR_OUTPUT_POWER = 'O',
    VAR_RES_TRACKING_PI_GAIN = 'G',
    VAR_RES_TRACKING_PI_I_TC = 'I',
    VAR_POWER_TRACKING_PI_GAIN = 'H',
    VAR_POWER_TRACKING_PI_I_TC = 'J'
};
typedef char Variable_t;

/* Imported variables ------------------------------------------------------*/
extern UART_HandleTypeDef huart3;

/* Private variables -------------------------------------------------------*/
static Cp_Command_t CommandTable[] =
    {
        // Reset command
        {
            .name = "reset",
            .params = {},
            .callback = resetCallback,
            .numOfParams = 0},

        // Calibrate command.
        {
            .name = "calibrate",
            .params = {},
            .callback = calibrateCallback,
            .numOfParams = 0},

        // Get command.
        {
            .name = "get",
            .params = {{.letter = 'V', .type = CP_PARAM_TYPE_LETTER}},
            .callback = getCallback,
            .numOfParams = 1},

        // Set command.
        {
            .name = "set",
            .params = {{.letter = 'V', .type = CP_PARAM_TYPE_REAL}},
            .callback = setCallback,
            .numOfParams = 1},

        // Status command.
        {
            .name = "status",
            .params = {},
            .callback = statusCallback,
            .numOfParams = 0},

        // Error command.
        {
            .name = "error",
            .params = {},
            .callback = errorCallback,
            .numOfParams = 0},
};

/* Expoted functions -------------------------------------------------------*/
void Cli_Init(void)
{
    // Initialize rs485 low level driver.
    Rs485_Init(&huart3, RS485_DEnRE_GPIO_Port, RS485_DEnRE_Pin);
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

void Cli_Stop(void)
{
    Rs485_Stop();
}

/* Callback functions ------------------------------------------------------*/
void resetCallback(Dictionary_Dictionary_t *params)
{
}

void calibrateCallback(Dictionary_Dictionary_t *params)
{
}

void getCallback(Dictionary_Dictionary_t *params)
{
}

void setCallback(Dictionary_Dictionary_t *params)
{
}

void statusCallback(Dictionary_Dictionary_t *params)
{
}

void errorCallback(Dictionary_Dictionary_t *params)
{
}