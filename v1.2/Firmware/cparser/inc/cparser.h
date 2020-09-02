/**
 * @file cparser.h
 * 
 * Interface of the cparser library.
 */
#ifndef __CPARSER_H
#define __CPARSER_H

#include "generic.h"
#include "cparser_config.h"
#include "dictionary.h"

#ifdef __cplusplus
extern "C"
{
#endif
    /* Typedefs ----------------------------------------------------------------*/
    /**
     * @brief Callback function prototype for the command parsed callbacks.
     * 
     * @param dictionary: Pointer to the dictionary of parameters.
     */
    typedef void (*Cp_ParsedCallback_t)(Dictionary_Dictionary_t *dictionary);

    /*! Parameter type enumeration. Determines type of the parameter to be 
    parsed. */
    enum _Cp_ParamType_t
    {
        CP_PARAM_TYPE_LETTER = 0,  /*!< Letter */
        CP_PARAM_TYPE_INTEGER = 1, /*!< Signed integer */
        CP_PARAM_TYPE_REAL = 2     /*!< Real number */
    };
    typedef uint8_t Cp_ParamType_t;

    /**
     * Parameter structure. Each parameter in a command structure is an instance
     * of this struct.
     */
    typedef struct
    {
        char letter;         /**< Letter of the parameter */
        Cp_ParamType_t type; /**< Type of the parameter */
    } Cp_Param_t;

    /**
     * Command structure. 
     */
    typedef struct
    {
        const char name[CPARSER_CONFIG_MAX_COMMAND_NAME_LENGTH + 1];
        /**< Null terminated command name string */
        const Cp_Param_t params[CPARSER_CONFIG_MAX_NUM_OF_PARAMS];
        /**< Array of parameters */
        const Cp_ParsedCallback_t callback;
        /**< Parsed callback function pointer */
        const uint8_t numOfParams;
        /**< Number of parameters */
    } Cp_Command_t;

    /* Functions ---------------------------------------------------------------*/
    extern void Cp_Reset(void);
    extern void Cp_Register(Cp_Command_t *command);
    extern uint8_t Cp_FeedLine(char *input, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif