#ifndef __GLOBAL_H
#define __GLOBAL_H

/* Global inclusions -------------------------------------------------------*/
#include "stdint.h"

/* Global types ------------------------------------------------------------*/
enum
{
    FALSE = 0,
    TRUE = !FALSE
};
typedef uint8_t Bool_t;

enum
{
    UNINIT = 0,
    READY,
    OPERATING
};
typedef uint8_t TaskStatus_t;

/* Global constants --------------------------------------------------------*/
#ifndef M_2PI
#define M_2PI 6.283185307179586f
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

#ifndef M_ROOT2
#define M_ROOT2 1.4142135623f
#endif

#endif
