#ifndef __CORE_H
#define __CORE_H

#include "global.h"

// Signal processing definitions.
#define PWM_CYCLES_PER_CONTROL_CYCLE 32
#define SAMPLES_PER_CYCLE 12
#define SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT (PWM_CYCLES_PER_CONTROL_CYCLE * SAMPLES_PER_CYCLE)

#define ADC_CHANNELS 2
#define ADC_BITS 12
#define ADC_DOUBLE_BUFFER_ELEMENT_COUNT (2 * ADC_CHANNELS * SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT)
#define ADC_SINGLE_BUFFER_ELEMENT_COUNT (ADC_DOUBLE_BUFFER_ELEMENT_COUNT >> 1)

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        float real;
        float img;
    } Complex_t;

    extern void getPhasors(uint16_t *bf, Complex_t *voltage, Complex_t *current);

#ifdef __cplusplus
}
#endif

#endif