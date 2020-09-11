#include "../inc/core.h"
#include "math.h"

/* Private definitions -----------------------------------------------------*/
// Measurement.
#define SHUNT_RESISTOR 0.01f
#define VDIV_RATIO (324.0f / 2200000.0f)

// ADC definitions.
#define ADC_VREF 3.3f

// Analog frontend definitions.
#define AFE_ISOAMP_GAIN 8.2f
#define AFE_POSTAMP_GAIN (1.0f + (1.0f / 3.3f))
#define AFE_LPASS_GAIN 0.8952749158393931f
#define AFE_VOLTAGE_GAIN (AFE_LPASS_GAIN * VDIV_RATIO * AFE_ISOAMP_GAIN * AFE_POSTAMP_GAIN)
#define AFE_CURRENT_GAIN (AFE_LPASS_GAIN * SHUNT_RESISTOR * AFE_ISOAMP_GAIN * AFE_POSTAMP_GAIN)

#define SIGNAL_PROCESSING_BLANKED_CYCLES 4
#define SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT (SIGNAL_PROCESSING_BLANKED_CYCLES * SAMPLES_PER_CYCLE)
#define SIGNAL_PROCESSING_VOLTAGE_MULTIPLIER \
    (2.0f / ((1 << ADC_BITS) * (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT - SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT)))
#define SIGNAL_PROCESSING_CURRENT_MULTIPLIER \
    (2.0f / ((1 << ADC_BITS) * (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT - SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT)))

/* Private functions -------------------------------------------------------*/
void multiplyReal(Complex_t *a, float value, Complex_t *result);

/* Exported functions ------------------------------------------------------*/
/***
 * @brief Returns the voltage and current complex phasors by processing 
 * sampled values.
 * 
 * @param bf: Pointer to the signal input buffer.
 * @param voltage: Voltage phasor.
 * @param current: Current phasor.
 */
void getPhasors(uint16_t *bf, Complex_t *voltage, Complex_t *current)
{
    static const float cosine[] = {1.0f, 0.8660254037844386f, 0.5f, 0.0f, -0.5f, -0.8660254037844386f};
    static const float sine[] = {0.0f, 0.5f, 0.8660254037844386f, 1.0f, 0.8660254037844386f, 0.5f};
    uint16_t off;
    float sum[24] = {0.0f};
    
    off = SIGNAL_PROCESSING_BLANKED_ELEMENT_COUNT * ADC_CHANNELS;

    // Sum elements which are to be multiplied with same modulation phase.
    while (off < (SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT * ADC_CHANNELS))
    {
        sum[0] += bf[off++]; // current sample(phase 0).
        sum[1] += bf[off++]; // voltage sample(phase 0).
        sum[2] += bf[off++]; // current sample(phase 30).
        sum[3] += bf[off++]; // voltage sample(phase 30).
        sum[4] += bf[off++]; // current sample(phase 60).
        sum[5] += bf[off++]; // Goes on like this...
        sum[6] += bf[off++];
        sum[7] += bf[off++];
        sum[8] += bf[off++];
        sum[9] += bf[off++];
        sum[10] += bf[off++];
        sum[11] += bf[off++];
        sum[12] += bf[off++];
        sum[13] += bf[off++];
        sum[14] += bf[off++];
        sum[15] += bf[off++];
        sum[16] += bf[off++];
        sum[17] += bf[off++];
        sum[18] += bf[off++];
        sum[19] += bf[off++];
        sum[20] += bf[off++];
        sum[21] += bf[off++];
        sum[22] += bf[off++];
        sum[23] += bf[off++];
    }

    Complex_t v = {0.0f, 0.0f};
    Complex_t i = {0.0f, 0.0f};

    // Apply weighted sum to find unnormalized voltage and current phasors.
    for (uint8_t n = 0; n < 6; n++)
    {
        v.real += (sum[2 * n + 1] - sum[2 * n + 13]) * cosine[n];
        v.img += (sum[2 * n + 13] - sum[2 * n + 1]) * sine[n];
        i.real += (sum[2 * n] - sum[2 * n + 12]) * cosine[n];
        i.img += (sum[2 * n + 12] - sum[2 * n]) * sine[n];
    }

    // Normalize voltage and current phasors.
    multiplyReal(&v, SIGNAL_PROCESSING_VOLTAGE_MULTIPLIER, voltage);
    multiplyReal(&i, SIGNAL_PROCESSING_CURRENT_MULTIPLIER, current);
}

/***
 * @brief Multiplies a complex number with a real number.
 * 
 * @param a: Pointer to the complex number to be multiplied.
 * @param value: Pointer to the real number to multiply.
 * @param result: Pointer to the result
 */
void multiplyReal(Complex_t *a, float value, Complex_t *result)
{
    result->real = a->real * value;
    result->img = a->img * value;
}