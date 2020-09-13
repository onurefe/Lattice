/***
  * @file   rs485.c
  * @author Onur Efe
  * @brief  RS485 protocol low level layer implementing buffering and sending
  * functionality. 
  */
/* Includes ----------------------------------------------------------------*/
#include "rs485.h"
#include "queue.h"

/* Private functions prototypes --------------------------------------------*/
static void setLineDrv(Bool_t tx);

/* Private variables -------------------------------------------------------*/
// Hardware config.
static UART_HandleTypeDef *ThisHuart;
static GPIO_TypeDef *ThisDenreGpio;
static uint32_t ThisDenrePin;

// Runtime variables.
static TaskStatus_t Status = UNINIT;
static volatile Bool_t TxActive;

// Buffers & containers.
static uint8_t TxBufferContainer[RS485_TX_BUFFER_SIZE];
static uint8_t RxBufferContainer[RS485_RX_BUFFER_SIZE];
static Queue_Buffer_t TxBuffer;
static Queue_Buffer_t RxBuffer;
static uint8_t TxReg;
static uint8_t RxReg;

/* Exported functions ------------------------------------------------------*/
/**
 * @brief  Function to initialize the module.
 */
void Rs485_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *denreGpio,
                uint32_t denrePin)
{
    // Guard for invalid operations.
    if (Status != UNINIT)
    {
        return;
    }

    if (!huart && !denreGpio)
    {
        return;
    }

    // Store hardware configuration.
    ThisHuart = huart;
    ThisDenreGpio = denreGpio;
    ThisDenrePin = denrePin;

    // Initialize buffers.
    Queue_InitBuffer(&RxBuffer, RxBufferContainer, RS485_RX_BUFFER_SIZE);
    Queue_InitBuffer(&TxBuffer, TxBufferContainer, RS485_TX_BUFFER_SIZE);

    Status = READY;
}

/**
 * @brief Function to start the module.
 */
void Rs485_Start(void)
{
    // Guard for invalid operations.
    if (Status != READY)
    {
        return;
    }

    // Clear buffers.
    Queue_ClearBuffer(&RxBuffer);
    Queue_ClearBuffer(&TxBuffer);

    // Set line into the reception mode.
    setLineDrv(FALSE);

    // Start receiving data.
    if (HAL_UART_Receive_IT(ThisHuart, &RxReg, sizeof(RxReg)) != HAL_OK)
    {
        while (TRUE)
            ;
    }

    TxActive = FALSE;
    Status = OPERATING;
}

/**
 * @brief Function to stop the module.
 */
void Rs485_Stop(void)
{
    // Guard for invalid operations.
    if (Status != OPERATING)
    {
        return;
    }

    HAL_UART_Abort_IT(ThisHuart);
    Status = READY;
}

/**
 * @brief Returns if there are any empty space in the TxBuffer.
 * 
 * @retval TRUE if there are any space; FALSE if not.
 */
Bool_t Rs485_TxBuffAvailable(void)
{
    return (!Queue_IsFull(&TxBuffer));
}

/**
 * @brief Enqueues array of data to the TxBuffer and starts the
 * transmission if it's not active.
 * 
 * @param buff: Pointer of the array.
 * @param length: Element count of the array.
 */
void Rs485_Transmit(char *buff, uint16_t length)
{
    // Enqueue data to the buffer.
    Queue_EnqueueArr(&TxBuffer, (uint8_t *)buff, length);

    // Start transmision if it wasn't active.
    if (!TxActive && !Queue_IsEmpty(&TxBuffer))
    {
        setLineDrv(TRUE);

        TxReg = Queue_Dequeue(&TxBuffer);

        if (HAL_UART_Transmit_IT(ThisHuart, &TxReg, sizeof(TxReg)) != HAL_OK)
        {
            while (TRUE)
                ;
        }

        TxActive = TRUE;
    }
}

/**
 * @brief Returns pointer to the rx buffer.
 * 
 * @retval RxBuffer pointer.
 */
Queue_Buffer_t *Rs485_GetRxBuffer(void)
{
    return &RxBuffer;
}

/* Callback functions ------------------------------------------------------*/
/**
 * @brief STM32 HAL Driver UART RX callback function implementation.
 * 
 * @param huart: Handle of the UART driver.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (ThisHuart == huart)
    {
        // Guard for invalid operations.
        if (Status != OPERATING)
        {
            return;
        }

        // Restart reception interrupt.
        if (HAL_UART_Receive_IT(ThisHuart, &RxReg, sizeof(RxReg)) != HAL_OK)
        {
            while (TRUE)
                ;
        }

        // Enqueue received data.
        Queue_Enqueue(&RxBuffer, RxReg);
    }
}

/**
 * @brief STM32 HAL Driver UART TX callback function implementation.
 * 
 * @param huart: Handle of the UART driver.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (ThisHuart == huart)
    {
        // Guard for invalid operations.
        if (Status != OPERATING)
        {
            return;
        }

        // If there are pending data on the TxBuffer; transmit it.
        if (!Queue_IsEmpty(&TxBuffer))
        {
            TxReg = Queue_Dequeue(&TxBuffer);

            // Restart transmission interrupt.
            if (HAL_UART_Transmit_IT(ThisHuart, &RxReg, sizeof(RxReg)) != HAL_OK)
            {
                while (TRUE)
                    ;
            }
        }
        else
        {
            // Switch line state to receive.
            setLineDrv(FALSE);
        }
    }
}

/* Private functions -------------------------------------------------------*/
void setLineDrv(Bool_t tx)
{
    if (tx)
    {
        HAL_GPIO_WritePin(ThisDenreGpio, ThisDenrePin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(ThisDenreGpio, ThisDenrePin, GPIO_PIN_RESET);
    }
}