/**
  ******************************************************************************
  * @file    eeprom_emulator.h
  * @author  Onur Efe
  * @date    14.07.2020
  * @brief   EEPROM Emulator class interface.
  ******************************************************************************
  *
  * Caution!!
  *
  * #Flash sector 10 and 11 shouldn't be used for other purposes. 
  * #Object ID should be within the range 0-FFFEh.
  * #Module should be initialized first.
  * #Flash voltage range other than FLASH_VOLTAGE_RANGE_3 should be defined by using
  * FLASH_VOLTAGE_RANGE_{1|2|3|4} format.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __EEPROM_EMULATOR_H
#define __EEPROM_EMULATOR_H

/* Includes ------------------------------------------------------------------*/
#include "global.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Eeprom emulator module initializer. Should be called before any other 
  *         functions at this module could be used.
  *
  * @param  None.
  *
  * @retval None.
  */
extern void EepromEmulator_Init(void);

/**
  * @brief  Reads data object from the flash.
  *
  * @param  objectId: Data object id.
  * @param  offset: Offset of the read start address.
  * @param  maxLength: Maximum read length.
  * @param  pLength: Pointer to return length of the data object.
  * @param  pData: Pointer to return data.
  *
  * @retval True of False(If the object has been found, true).
  */
extern Bool_t EepromEmulator_ReadObject(uint16_t objectId, uint16_t offset, uint16_t maxLength,
                                        uint16_t *pLength, uint8_t *pData);

/**
  * @brief  Writes data object to the flash memory. 
  *
  * @param  objectId: Id of the data object which is to be written.
  * @param  length: Length of the data object.
  * @param  pData: Pointer of the data.
  *
  * @retval None.
  */
extern void EepromEmulator_WriteObject(uint16_t objectId, uint16_t length, uint8_t *pData);

#endif