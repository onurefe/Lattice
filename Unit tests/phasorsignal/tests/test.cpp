#include "gtest/gtest.h"
#include "stdlib.h"
#include "../phasorsignal/inc/core.h"
#include "math.h"

/* Private function prototypes ---------------------------------------------*/
/* Private variables -------------------------------------------------------*/
struct PhasorSignalTest : public ::testing::Test
{
public:
    uint16_t AdcBuffer[ADC_DOUBLE_BUFFER_ELEMENT_COUNT];

    virtual void SetUp() override
    {
    }

    virtual void TearDown() override
    {
    }
};

TEST_F(PhasorSignalTest , Phase1)
{
    for (uint16_t i = 0; i < SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT; i++)
    {
        AdcBuffer[2 * i] = (uint16_t)((1 << ADC_BITS) * (1.5f + cosf(i * M_2PI / SAMPLES_PER_CYCLE)));
        AdcBuffer[2 * i + 1] = (uint16_t)((1 << ADC_BITS) * (1.5f + sinf(i * M_2PI / SAMPLES_PER_CYCLE)));
    }

    Complex_t voltage, current;
    getPhasors(AdcBuffer, &voltage, &current);

    EXPECT_NEAR(voltage.real, 0.0f, 1e-3);
    EXPECT_NEAR(voltage.img, -1.0f, 1e-3);
    EXPECT_NEAR(current.real, 1.0f, 1e-3);
    EXPECT_NEAR(current.img, 0.0f, 1e-3);
}

TEST_F(PhasorSignalTest , Phase2)
{
    for (uint16_t i = 0; i < SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT; i++)
    {
        AdcBuffer[2 * i] = (uint16_t)((1 << ADC_BITS) * (1.5f + cosf(M_PI_2 + (i * M_2PI / SAMPLES_PER_CYCLE))));
        AdcBuffer[2 * i + 1] = (uint16_t)((1 << ADC_BITS) * (1.5f + sinf(M_PI_2 + (i * M_2PI / SAMPLES_PER_CYCLE))));
    }

    Complex_t voltage, current;
    getPhasors(AdcBuffer, &voltage, &current);

    EXPECT_NEAR(voltage.real, 1.0f, 1e-3);
    EXPECT_NEAR(voltage.img, 0.0f, 1e-3);
    EXPECT_NEAR(current.real, 0.0f, 1e-3);
    EXPECT_NEAR(current.img, 1.0f, 1e-3);
}

TEST_F(PhasorSignalTest , Phase3)
{
    for (uint16_t i = 0; i < SIGNAL_PROCESSING_WINDOW_ELEMENT_COUNT; i++)
    {
        AdcBuffer[2 * i] = (uint16_t)((1 << ADC_BITS) * (1.5f + cosf(M_PI + M_PI_4 + (i * M_2PI / SAMPLES_PER_CYCLE))));
        AdcBuffer[2 * i + 1] = (uint16_t)((1 << ADC_BITS) * (1.5f + cosf(M_PI + M_PI_4 + (i * M_2PI / SAMPLES_PER_CYCLE))));
    }

    Complex_t voltage, current;
    getPhasors(AdcBuffer, &voltage, &current);

    EXPECT_NEAR(voltage.real, -M_SQRT2 * 0.5f, 1e-3);
    EXPECT_NEAR(voltage.img, -M_SQRT2 * 0.5f, 1e-3);
    EXPECT_NEAR(current.real, -M_SQRT2 * 0.5f, 1e-3);
    EXPECT_NEAR(current.img, -M_SQRT2 * 0.5f, 1e-3);
}