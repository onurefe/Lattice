#ifndef __RS485_H
#define __RS485_H

#include "global.h"
#include "queue.h"

/* Exported functions ------------------------------------------------------*/
void Rs485_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *denreGpio,
                uint32_t denrePin);
void Rs485_Start(void);
void Rs485_Stop(void);
Bool_t Rs485_TxBuffAvailable(void);
void Rs485_Transmit(char *buff, uint16_t length);
Queue_Buffer_t *Rs485_GetRxBuffer(void);

#endif