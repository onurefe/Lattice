#include "gtest/gtest.h"
#include "stdlib.h"
#include "../pid/inc/pid.h"
#include "math.h"

/* Private function prototypes ---------------------------------------------*/
/* Private variables -------------------------------------------------------*/
struct PidTest : public ::testing::Test
{
public:
    Pid_t Pid;

    virtual void SetUp() override
    {
    }

    virtual void TearDown() override
    {
    }
};

TEST_F(PidTest, ConstantInput)
{
    Pid_Params_t params;

    params.kp = 1.0f;
    params.ki = 0.5f;
    params.kd = 0.0f;
    params.tfilt = 0.0f;
    Pid_Setup(&Pid, &params, __FLT_MIN__, __FLT_MAX__);

    float dt = 0.001f;
    float t = 0.0f;
    float output;
    float error;
    float variance = 0.0f;

    for (uint16_t i = 0; i < 1000; i++)
    {
        t = dt * ((float)i);
        error = Pid_Exe(&Pid, 1.0f, dt) - (Pid.kp + Pid.ki * t);
        variance += (error * error * dt);
    }

    float stdev;
    stdev = sqrtf(variance);
    EXPECT_NEAR(stdev, 0.0f, 1e-3);
}

TEST_F(PidTest, RampInput)
{
    Pid_Params_t params;

    params.kp = 1.5f;
    params.ki = 0.5f;
    params.kd = 1.25f;
    params.tfilt = 0.0f;
    Pid_Setup(&Pid, &params, __FLT_MIN__, __FLT_MAX__);

    float dt = 0.001f;
    float t = 0.0f;
    float output;
    float error;
    float variance = 0.0f;

    for (uint32_t i = 0; i < 1000; i++)
    {
        t = dt * ((float)i);
        error = Pid_Exe(&Pid, t, dt) - (Pid.kp * t + 0.5f * Pid.ki * t * t + Pid.kd * (i > 0));
        variance += (error * error * dt);
    }

    float stdev;
    stdev = sqrtf(variance);
    EXPECT_NEAR(stdev, 0.0f, 1e-3);
}