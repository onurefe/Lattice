/***
  * @file    queue.h
  * @author  Onur Efe
  * @brief   Queue structure implementation.
  */
#ifndef __QUEUE_H
#define __QUEUE_H

/* Includes ------------------------------------------------------------------*/
#include "global.h"

/* Typedefs ------------------------------------------------------------------*/
typedef struct
{
    uint16_t tail;
    uint16_t head;
    uint16_t size;
    uint8_t *pContainer;
} Queue_Buffer_t;

/* Exported functions --------------------------------------------------------*/
/**
    * @brief    Creates a buffer, allocates it's memory and returns the pointer
    *   of it.
    * @param    buff: Pointer to buffer.
    * @param    container: Pointer of the data container.
    * @param    size: Size of the data container.
    */
static inline void Queue_InitBuffer(Queue_Buffer_t *buff, uint8_t *container, uint16_t size)
{
    /* Initialize buffer. */
    buff->head = 0;
    buff->tail = 0;
    buff->pContainer = container;
    buff->size = size;
}

/**
    * @brief    Clears the addressed buffer.
    *
    * @param    buff:Pointer to the buffer.
    */
static inline void Queue_ClearBuffer(Queue_Buffer_t *buff)
{
    buff->head = buff->tail;
}

/**
    * @brief    Enqueues byte to the given buffer.
    *
    * @param    buff: Pointer to the buffer.
    * @param    byte: Byte to be enqueued.
    */
static inline void Queue_Enqueue(Queue_Buffer_t *buff, uint8_t byte)
{
    uint16_t __tail = buff->tail;

    // Set new tail element and update the tail value.
    buff->pContainer[__tail++] = byte;
    if (__tail == buff->size)
    {
        __tail = 0;
    }

    buff->tail = __tail;
}

/**
    * @brief    Enqueues byte array to the given buffer.
    *
    * @param    buff: Pointer to the buffer.
    * @param    data: Pointer to byte array to be enqueued.
    */
static inline void Queue_EnqueueArr(Queue_Buffer_t *buff, uint8_t *data, uint16_t dataLength)
{
    uint16_t __tail = buff->tail;

    for (uint16_t i = 0; i < dataLength; i++)
    {
        buff->pContainer[__tail++] = data[i];
        if (__tail == buff->size)
        {
            __tail = 0;
        }
    }

    buff->tail = __tail;
}

/**
    * @brief    Dequeues element from the given buffer.
    *
    * @param    buff: Pointer to the buffer.
    *
    * @retval   Element.
    */
static inline uint8_t Queue_Dequeue(Queue_Buffer_t *buff)
{
    uint16_t __head = buff->head;

    // Parse head element.
    uint8_t element = buff->pContainer[__head++];
    if (__head == buff->size)
    {
        __head = 0;
    }

    buff->head = __head;

    return element;
}

/**
    * @brief    Dequeues array from the given buffer.
    *
    * @param    buff: Pointer to the buffer.
    * 
    * @retval   None.
    */
static inline void Queue_DequeueArr(Queue_Buffer_t *buff, uint8_t *data, uint16_t dataLength)
{
    uint16_t __head = buff->head;

    for (uint16_t i = 0; i < dataLength; i++)
    {
        data[i] = buff->pContainer[__head++];
        if (__head == buff->size)
        {
            __head = 0;
        }
    }

    buff->head = __head;
}

/**
    * @brief    Returns number of elements the buffer contains.
    *
    * @param    buff: Buffer pointer.
    *
    * @retval   Element count.
    */
static inline uint16_t Queue_GetElementCount(Queue_Buffer_t *buff)
{
    int32_t elements = buff->tail - buff->head;
    if (elements < 0)
    {
        elements += buff->size;
    }

    return ((uint16_t)elements);
}

/**
    * @brief    Removes elements starting from the head.
    *
    * @param    buff: Pointer to the buffer.
    * @param    length: Length of removal.
    */
static inline void Queue_Remove(Queue_Buffer_t *buff, uint16_t length)
{
    /* Just change head index. */
    if (length <= Queue_GetElementCount(buff))
    {
        uint16_t __head = buff->head;
        __head += length;
        if (__head >= buff->size)
        {
            __head -= buff->size;
        }
        buff->head = __head;
    }
}

/**
    * @brief    Peeks element in the buffer. Indexing starts at the first element.
    *
    * @param    buff: Pointer to the buffer.
    * @param    elementIndex: Index of the element.
    *
    * @retval   Element value.
    */
static inline uint8_t Queue_Peek(Queue_Buffer_t *buff, uint16_t elementIndex)
{
    uint16_t element_position = buff->head + elementIndex;
    if (element_position >= buff->size)
    {
        element_position -= buff->size;
    }

    return (buff->pContainer[element_position]);
}

/**
    * @brief    Searches an element in the buffer. Returns element index if the element
    *           exists. Returns 0xFFFF otherwise.
    *
    * @param    buff: Buffer to be searched.
    * @param    element: Element value.
    *
    * @retval   Element index or if not found 0xFFFF.
    */
static inline uint16_t Queue_Search(Queue_Buffer_t *buff, uint8_t element)
{
    uint16_t num_of_elements;
    num_of_elements = Queue_GetElementCount(buff);

    // Search buffer.
    for (uint16_t i = 0; i < num_of_elements; i++)
    {
        if (Queue_Peek(buff, i) == element)
        {
            return i;
        }
    }

    return 0xFFFF;
}

/**
    * @brief    Searches an array in the buffer. Returns array start index if the such an array
    *           exists. Returns 0xFFFF otherwise.
    *
    * @param    buff: Buffer to be searched.
    * @param    arr: Pointer to array.
    *
    * @retval   Array start index or 0xFFFF.
    */
static inline uint16_t Queue_SearchArr(Queue_Buffer_t *buff, uint8_t *arr, uint16_t arrLength)
{
    uint16_t num_of_elements;
    num_of_elements = Queue_GetElementCount(buff);

    // Search buffer.
    for (uint16_t i = 0; i < num_of_elements; i++)
    {
        Bool_t matched = TRUE;
        for (uint16_t j = 0; j < arrLength; j++)
        {
            if (Queue_Peek(buff, i + j) != arr[j])
            {
                matched = FALSE;
                continue;
            }
        }

        // If match found; return the start index.
        if (matched)
        {
            return i;
        }
    }

    return 0xFFFF;
}

/**
    * @brief    Gets array of data without touching the queue structure.
    *
    * @param    buff: Buffer to be peeked.
    * @param    idx: Start index of the array.
    * @param    data: Pointer of the array to be returned.
    * @param    dataLength: Length of the data to be peeked.
    */
static inline void Queue_PeekArr(Queue_Buffer_t *buff, uint16_t idx, uint8_t *data,
                                 uint16_t dataLength)
{
    uint16_t __head = buff->head + idx;
    if (__head >= buff->size)
    {
        __head -= buff->size;
    }

    for (uint16_t i = 0; i < dataLength; i++)
    {
        data[i] = buff->pContainer[__head++];
        if (__head == buff->size)
        {
            __head = 0;
        }
    }
}

/**
    * @brief    Writes array of data the queue from the given index.
    *
    * @param    buff: Buffer containing the data.
    * @param    idx: Write start index.
    * @param    data: Pointer of the array to be written.
    * @param    dataLength: Length of the data to be written.
    */
static inline void Queue_WriteArr(Queue_Buffer_t *buff, uint16_t idx, uint8_t *data,
                                  uint16_t dataLength)
{
    uint16_t __head = buff->head + idx;
    if (__head >= buff->size)
    {
        __head -= buff->size;
    }

    for (uint16_t i = 0; i < dataLength; i++)
    {
        buff->pContainer[__head++] = data[i];
        if (__head == buff->size)
        {
            __head = 0;
        }
    }
}

/**
    * @brief    Sets element in the buffer. Indexing starts at the first element.
    *
    * @param    buff: Pointer to the buffer.
    * @param    elementIndex: Index of the element.
    * @param    value: Value of the element to be set.
    */
static inline void Queue_Set(Queue_Buffer_t *buff, uint16_t elementIndex, uint8_t value)
{
    uint16_t element_position = buff->head + elementIndex;
    if (element_position >= buff->size)
    {
        element_position -= buff->size;
    }

    buff->pContainer[element_position] = value;
}

/**
    * @brief    Gets index of next write take place.
    *
    * @param    buff: Pointer to the buffer.
    *
    * @retval   Index.
    */
static inline uint16_t Queue_GetCurrentIdx(Queue_Buffer_t *buff)
{
    int32_t element_position = buff->tail - buff->head;
    if (element_position < 0)
    {
        element_position += buff->size;
    }

    return ((uint16_t)element_position);
}

/**
    * @brief    Returns available space of the buffer.
    *
    * @param    buff: Buffer pointer.
    *
    * @retval   Available space.
    */
static inline uint16_t Queue_GetAvailableSpace(Queue_Buffer_t *buff)
{
    int32_t elements = buff->tail - buff->head;
    if (elements < 0)
    {
        elements += buff->size;
    }

    return (buff->size - (elements + 1));
}

/***
    * @brief    Checks if the buffer is empty.
    *
    * @param    buff: Pointer to buffer.
    *
    * @retval   TRUE or FALSE.
    */
static inline Bool_t Queue_IsEmpty(Queue_Buffer_t *buff)
{
    return ((buff->tail == buff->head) ? TRUE : FALSE);
}

/**
    * @brief    Checks if the buffer is full.
    *
    * @param    buff: Pointer to buffer.
    *
    * @retval   TRUE or FALSE.
    */
static inline Bool_t Queue_IsFull(Queue_Buffer_t *buff)
{
    uint16_t next_tail = (buff->tail + 1);
    if (next_tail == buff->size)
    {
        next_tail = 0;
    }

    return ((next_tail == buff->head) ? TRUE : FALSE);
}

/**
   * @brief     Returns capacity of the buffer.
   *
   * @param     buff: Pointer to buffer.
   *
   * @retval    Capacity of the queue.
   */
static inline uint16_t Queue_GetCapacity(Queue_Buffer_t *buff)
{
    return (buff->size - 1);
}

#endif
