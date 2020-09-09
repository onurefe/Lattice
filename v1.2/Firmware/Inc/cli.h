#ifndef __CLI_H
#define __CLI_H

#include "global.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Cli_SendMsg(char *buff, uint8_t length);
void Cli_Init(void);
void Cli_Start(void);
void Cli_Execute(void);
void Cli_Stop(void);

#ifdef __cplusplus
}
#endif

#endif