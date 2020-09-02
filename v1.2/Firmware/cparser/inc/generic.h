/**
  * @file generic.h
  * 
  * Definitions which are used globally are contained in this file.
  */
#ifndef __GENERIC_H
#define __GENERIC_H

#include <stdint.h>

#ifndef NULL
#define NULL ((void *)0U)
#endif

/* Exported types ----------------------------------------------------------*/
/*! Boolean enumeration. */
enum _Bool_t
{
  FALSE = 0,    /*!< FALSE */
  TRUE = !FALSE /*!< TRUE */
};
typedef uint8_t Bool_t;

#endif
