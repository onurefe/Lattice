#ifndef __RS485_H
#define __RS485_H

#include "global.h"
#include "queue.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* Exported functions ------------------------------------------------------*/
    extern void Rs485_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *denreGpio,
                           uint32_t denrePin);
    extern void Rs485_Start(void);
    extern void Rs485_Stop(void);
    extern Bool_t Rs485_TxBuffAvailable(void);
    extern void Rs485_Transmit(uint8_t *buff, uint16_t length);
    extern Queue_Buffer_t *Rs485_GetRxBuffer(void);

#ifdef __cplusplus
}
#endif

#endif