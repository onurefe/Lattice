#include "gtest/gtest.h"
#include "stdlib.h"
#include "../regression/inc/polyreg.h"
#include "math.h"

/* Private definitions -----------------------------------------------------*/
#define X_START 1.8f
#define X_STOP 2.2f
#define SAMPLES 100
#define TEST_POLY {1.0f, 0.5f, 3.0f}

/* Private variables -------------------------------------------------------*/
struct RegressionTest : public ::testing::Test
{
public:
    virtual void SetUp() override
    {
    }

    virtual void TearDown() override
    {
    }
};

TEST_F(RegressionTest, Polynomial1)
{
    float poly[] = TEST_POLY;
    float regpoly[sizeof(poly) / sizeof(poly[0])];
    float output[SAMPLES] = {0.0f};
    float x;

    // Calculate complex polynomial for every value.
    for (uint16_t i = 0; i < SAMPLES; i++)
    {
        x = ((X_STOP - X_START) / SAMPLES) * ((float)i) + X_START;
        for (int8_t j = ((sizeof(poly) / sizeof(poly[0])) - 1); j >= 0; j--)
        {
            output[i] *= x; 
            output[i] += poly[j];
        }
    }

    // Apply regression.
    Polyreg_Fit(output, regpoly);

    // Check if the coefficients are same.
    for (uint8_t i = 0; i < sizeof(poly) / sizeof(poly[0]); i++)
    {
        EXPECT_NEAR(poly[i], regpoly[i], 1e-3);
    }
}