/**
 * @file cparser.c
 * 
 * Cparser library core module.
 */
#include "../inc/cparser.h"
#include "math.h"

/* Private constants -------------------------------------------------------*/
#define MAX_FIELD_COUNT (CPARSER_CONFIG_MAX_NUM_OF_PARAMS + 1)
#define MAX_PARAM_DATA_SIZE sizeof(float)

/* Private typedefs --------------------------------------------------------*/
typedef struct
{
    uint8_t start;
    uint8_t length;
} Field_t;

/* Private function prototypes ---------------------------------------------*/
static Bool_t doesMatch(const char *cname, char *input, uint8_t length);
static void parseFields(char *input, uint8_t inputLength, Field_t *fields, uint8_t *numOfFields);
static Bool_t parseValue(char *input, uint8_t inputLength, uint8_t type, void *data, uint8_t *size);
static Bool_t parseFloat(char *input, uint8_t start_idx, uint8_t length, float *value);
static Bool_t parseFractional(char *input, uint8_t start_idx, uint8_t length, float *value);
static Bool_t parseSignedInteger(char *input, uint8_t start_idx, uint8_t length, int32_t *value);
static Bool_t parseUnsignedInteger(char *input, uint8_t start_idx, uint8_t length,
                                   uint32_t *value);
static void getSign(char *input, uint8_t start_idx, uint8_t length, int8_t *sign, uint8_t *stop_idx);
static void cropJerk(char *input, uint8_t start_idx, uint8_t length, uint8_t *stop_idx);
static uint8_t getLength(const char *input);

/* Private variables -------------------------------------------------------*/
static Cp_Command_t *CommandTable[CPARSER_CONFIG_MAX_NUM_OF_COMMANDS];
static uint16_t NumOfCommands = 0;
static Dictionary_Dictionary_t ParameterDictionary;

/* Exported functions ------------------------------------------------------*/
/**
 * @brief Clears the command parser registry.
 */
void Cp_Reset(void)
{
    NumOfCommands = 0;
}

/**
 * @brief Registers a command.
 */
void Cp_Register(Cp_Command_t *command)
{
    CommandTable[NumOfCommands++] = command;
}

/**
 * @brief Feeds line of char array.
 * 
 * @param input: Line char array.
 * @param length: Length of the char array.
 * 
 * @retval TRUE or FALSE.
 */
uint8_t Cp_FeedLine(char *input, uint16_t length)
{
    Field_t fields[MAX_FIELD_COUNT];
    uint8_t field_count;

    // Parse fields.
    parseFields(input, length, fields, &field_count);
    if (field_count == 0)
    {
        return FALSE;
    }

    // Find command.
    Cp_Command_t *command = NULL;
    for (uint8_t i = 0; i < NumOfCommands; i++)
    {
        if (doesMatch(CommandTable[i]->name, &input[fields[0].start],
                      fields[0].length))
        {
            command = CommandTable[i];
            break;
        }
    }

    // If the command is not found; return FALSE.
    if (!command)
    {
        return FALSE;
    }

    // Clear the dictionary for the current parsing.
    Dictionary_Clear(&ParameterDictionary);

    // Allocate memory for parsed values.
    uint8_t bulk_data[CPARSER_CONFIG_MAX_NUM_OF_PARAMS * MAX_PARAM_DATA_SIZE];
    uint16_t bulk_data_size = 0;

    // Find params and parse their values.
    for (uint8_t i = 0; i < command->numOfParams; i++)
    {
        for (uint8_t j = 1; j < field_count; j++)
        {
            uint8_t param_size;

            // If parameter is found; it should be parsed and added to the dictionary. Then next parameter
            //should be searched.
            if (input[fields[j].start] == command->params[i].letter)
            {
                if (parseValue(&input[fields[j].start + 1], fields[j].length - 1,
                               command->params[i].type, &bulk_data[bulk_data_size], &param_size))
                {
                    // Add parameter to dictionary.
                    Dictionary_Add(&ParameterDictionary, command->params[i].letter, &bulk_data[bulk_data_size]);
                    bulk_data_size += param_size;

                    break;
                }
                else
                {
                    return FALSE;
                }
            }
        }
    }

    // Call related callback.
    command->callback(&ParameterDictionary);

    return TRUE;
}

/* Private functions -------------------------------------------------------*/
/**
 * @brief Parse fields of the command string.
 * 
 * @param input: Command line string.
 * @param inputLength: Length of the input line string.
 * @param fields: Pointer to return fields of the line string.
 * @param numOfFields: Pointer to return number of fields.
 */
void parseFields(char *input, uint8_t inputLength, Field_t *fields, uint8_t *numOfFields)
{
    uint8_t field_count = 0;
    uint8_t field_start;

    field_start = 0;
    for (uint8_t i = 0; i < inputLength; i++)
    {
        // If seperator found or this is the last element,
        //this is the end of this field. Besides the next element is first element
        //of the next field.
        if ((input[i] == ' ' && field_start < i) || (i == (inputLength - 1) && field_start <= i))
        {
            if (i == (inputLength - 1))
            {
                fields[field_count].length = i + 1 - field_start;
            }
            else
            {
                fields[field_count].length = i - field_start;
            }

            fields[field_count].start = field_start;
            field_count++;
            field_start = i + 1;
        }
    }

    *numOfFields = field_count;
}

/**
 * @brief Compare if the input matches to the command name.
 * 
 * @param cname: Null terminated command name string.
 * @param input: Input char array.
 * @param length: Length of the input.
 * 
 * @retval TRUE or FALSE.
 */
Bool_t doesMatch(const char *cname, char *input, uint8_t length)
{
    uint8_t name_len = 0;

    // Parse length of the code name.
    name_len = getLength(cname);

    // If length of the name isn't equal to field length, return false.
    if (name_len != length)
    {
        return FALSE;
    }

    // Compare all the elements.
    Bool_t is_equal = TRUE;
    for (uint8_t i = 0; i < length; i++)
    {
        if (cname[i] != input[i])
        {
            is_equal = FALSE;
            break;
        }
    }

    return is_equal;
}

/**
 * @brief Parses the value of a given type.
 * 
 * @param input: Input char array.
 * @param inputLength: Length of the char array.
 * @param type: Type of the parameter.
 * @param data: Pointer to the return value.
 * @param size: Pointer to the return value memory size.
 * 
 * @retval TRUE or FALSE.
 */
Bool_t parseValue(char *input, uint8_t inputLength, Cp_ParamType_t type, void *data, uint8_t *size)
{
    Bool_t retval = FALSE;

    switch (type)
    {
    case CP_PARAM_TYPE_LETTER:
        retval = (inputLength == 1) ? TRUE : FALSE;
        *((char *)data) = input[0];
        *size = sizeof(char) + 3;
        break;

    case CP_PARAM_TYPE_INTEGER:
        retval = parseSignedInteger(input, 0, inputLength, data);
        *size = sizeof(int32_t);
        break;

    case CP_PARAM_TYPE_REAL:
        retval = parseFloat(input, 0, inputLength, data);
        *size = sizeof(float);
        break;

    default:
        break;
    }

    return retval;
}

/**
 * @brief Parses the floating point value.
 * 
 * @param input: Input char array.
 * @param start_idx: Start index of the number.
 * @param length: Length of the char array.
 * @param value: Pointer to the return value.
 * 
 * @retval TRUE or FALSE.
 */
Bool_t parseFloat(char *input, uint8_t start_idx, uint8_t length, float *value)
{
    uint8_t e_idx = 0xFF;

    if (length > start_idx)
    {
        // Search for scientific notation number.
        for (uint8_t i = start_idx; i < length; i++)
        {
            if ((input[i] == 'e' || input[i] == 'E'))
            {
                e_idx = i;
                break;
            }
        }

        // E sign at the end doesn't mean anything. So evaluate as
        //there isn't any exponent part if it's like that.
        e_idx = e_idx < length - 1 ? e_idx : 0xFF;

        // If scientific notation has not been used, simply parse the
        //fractional number.
        if (e_idx == 0xFF)
        {
            if (!parseFractional(input, start_idx, length, value))
            {
                return FALSE;
            }
        }
        else
        {
            float __value;
            int32_t power;

            // Parse until e sign.
            if (e_idx > 0)
            {
                if (!parseFractional(input, start_idx, e_idx, &__value))
                {
                    return FALSE;
                }
            }
            else
            {
                __value = 1.0f;
            }

            // Parse the exponent.
            if (!parseSignedInteger(input, (e_idx + 1), length, &power))
            {
                return FALSE;
            }

            uint32_t abspower = power > 0 ? power : -power;
            for (uint8_t i = 0; i < abspower; i++)
            {
                if (power > 0)
                {
                    __value *= 10.0f;
                }
                else
                {
                    __value *= 0.1f;
                }
            }

            *value = __value;
        }
    }
    else
    {
        *value = 0.0f;
    }

    return TRUE;
}

/**
 * @brief Parses the fractional value.
 * 
 * @param input: Input char array.
 * @param start_idx: Start index of the number.
 * @param length: Length of the char array.
 * @param value: Pointer to the return value.
 * 
 * @retval TRUE or FALSE.
 */
Bool_t parseFractional(char *input, uint8_t start_idx, uint8_t length, float *value)
{
    if (length > start_idx)
    {
        uint8_t decimal_point_idx = 0xFF;
        uint32_t integral = 0;
        int8_t sign;
        uint8_t parse_idx;
        float fraction = 0.0f;

        // Get sign.
        getSign(input, start_idx, length, &sign, &parse_idx);

        if (parse_idx >= length)
        {
            *value = 0.0f;
            return TRUE;
        }

        // Search for decimal point.
        for (uint8_t i = parse_idx; i < length; i++)
        {
            if (input[i] == '.')
            {
                decimal_point_idx = i;
                break;
            }
        }

        // Parse integral part.
        uint8_t integral_length;
        integral_length = decimal_point_idx != 0xFF ? decimal_point_idx : length;

        if (integral_length > parse_idx)
        {
            if (!parseUnsignedInteger(input, parse_idx, integral_length, &integral))
            {
                return FALSE;
            }
        }

        // Parse fractional part. Note that 0xFF is also bigger than any value of the (length - 1) expression.
        if (decimal_point_idx < length - 1)
        {
            uint32_t divisor = 10U;

            // Parse digits.
            for (uint8_t i = (decimal_point_idx + 1); i < length; i++)
            {
                uint8_t digit;
                digit = input[i] - 48;

                // Tried to parse something which is not a number.
                if (digit > 9)
                {
                    return FALSE;
                }

                fraction += (float)digit / divisor;
                divisor *= 10U;
            }
        }

        *value = sign * (((float)integral) + fraction);
    }
    else
    {
        *value = 0.0f;
    }

    return TRUE;
}

/**
 * @brief Parses the signed integer value.
 * 
 * @param input: Input char array.
 * @param start_idx: Start index of the number.
 * @param length: Length of the char array.
 * @param value: Pointer to the return value.
 * 
 * @retval TRUE or FALSE.
 */
Bool_t parseSignedInteger(char *input, uint8_t start_idx, uint8_t length, int32_t *value)
{
    if (length > start_idx)
    {
        uint32_t __value = 0;
        int8_t sign = 1;
        uint8_t parse_idx = 0;

        getSign(input, start_idx, length, &sign, &parse_idx);

        if (!parseUnsignedInteger(input, parse_idx, length, &__value))
        {
            return FALSE;
        }

        *value = __value * sign;
    }
    else
    {
        *value = 0;
    }

    return TRUE;
}

/**
 * @brief Parses the unsigned integer value.
 * 
 * @param input: Input char array.
 * @param start_idx: Start index of the number.
 * @param length: Length of the char array.
 * @param value: Pointer to the return value.
 * 
 * @retval TRUE or FALSE.
 */
Bool_t parseUnsignedInteger(char *input, uint8_t start_idx, uint8_t length,
                            uint32_t *value)
{
    if (length > start_idx)
    {
        uint32_t __value = 0;

        // Parse digits.
        for (uint8_t i = start_idx; i < length; i++)
        {
            uint8_t digit;
            digit = input[i] - 48;

            // Tried to parse something which is not a number.
            if (digit > 9)
            {
                return FALSE;
            }

            __value = __value * 10 + digit;
        }

        *value = __value;
    }
    else
    {
        *value = 0;
    }

    return TRUE;
}

/**
 * @brief Gets the sign of a given value in the given char array.
 * 
 * @param input: Char array to be processed.
 * @param start_idx: Start index of the processing.
 * @param length: Window of the process.
 * @param sign: Pointer to return 1 or -1(positive or negative respectively).
 * @param stop_idx: Pointer to return index next to the sign element index.
 */
void getSign(char *input, uint8_t start_idx, uint8_t length, int8_t *sign, uint8_t *stop_idx)
{
    uint8_t parse_idx = start_idx;
    int8_t __sign = 1;

    if (length > start_idx)
    {
        if (input[parse_idx] == '-')
        {
            parse_idx++;
            __sign = -1;
        }
        else if (input[parse_idx] == '+')
        {
            parse_idx++;
            __sign = 1;
        }

        cropJerk(input, parse_idx, length, stop_idx);

        *sign = __sign;
    }
    else
    {
        *sign = 1;
    }
}

/**
 * @brief Crops the meaningless data out of the char array.
 * 
 * @param input: Pointer to the input char array.
 * @param start_idx: Start index of the process.
 * @param length: Window of the process.
 * @param stop_idx: Pointer to the meaningfull data.
 */
void cropJerk(char *input, uint8_t start_idx, uint8_t length, uint8_t *stop_idx)
{
    uint8_t __stop_idx = start_idx;

    // Get rid of trailing spaces or zeros.
    while ((input[__stop_idx] == '0') || input[__stop_idx] == ' ')
    {
        __stop_idx++;
        if (__stop_idx == length)
        {
            *stop_idx = length - 1;
        }
    }

    *stop_idx = __stop_idx;
}

/**
 * @brief Gets the length of a null terminated string(length except null 
 * termination)
 * 
 * @param input: Pointer to the string.
 * 
 * @retval 0xFF if not found. Length if found.
 */
uint8_t getLength(const char *input)
{
    uint8_t i = 0;
    while (input[i] != '\0')
    {
        if (i == 0xFF)
            break;
        i++;
    }

    return i;
}