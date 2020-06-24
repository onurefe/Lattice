#include "lattice.h"
#include "piezo_driver.h"
#include "ammeter.h"
#include "synchronous_rectifier.h"
#include "math.h"

/* Private definitions -----------------------------------------------------*/
#define POWER_UP_STABILIZATION 1000U
#define START_PERIOD (MCU_CLOCK_FREQ / 18000)
#define END_PERIOD (MCU_CLOCK_FREQ / 24000)
#define PLAYERS (START_PERIOD - END_PERIOD + 1)
#define VERY_HIGH_CURRENT 1e3

/* Private function prototypes ---------------------------------------------*/
static uint16_t findTheBestPlayer(void);
static inline uint16_t playerToPeriod(uint16_t idx);
static inline uint16_t periodToPlayer(uint16_t period);
static inline int16_t absint(int16_t val);

/* Private variables -------------------------------------------------------*/
static volatile float Hands[PLAYERS];
static uint16_t BestPlayer;
static float LuckyHand;

/* Exported functions ------------------------------------------------------*/
void Lattice_Start(void)
{
    HAL_Delay(POWER_UP_STABILIZATION);

    // Start soft starter.
    Sr_Start();
    Sr_SetVoltage(1.0f);

    // Wait till stabilization.
    while (!Sr_IsStabilized())
        ;

    // As if highest period has the maximum value. The algorithm would lead to
    //searching automatically.
    for (uint16_t i = 0; i < PLAYERS; i++)
    {
        Hands[i] = (i + 1) * VERY_HIGH_CURRENT;
    }

    BestPlayer = 0;
    LuckyHand = 0.0f;
    
    Pd_SetPeriod(START_PERIOD);
    Pd_CmdSignalGeneration(TRUE);
    Ammeter_Start();
}

void Lattice_Execute(void)
{
    Ammeter_Execute();

    // If measurement completed find the best period.
    if (!Ammeter_IsBusy())
    {
        uint16_t new_best_player = findTheBestPlayer();
        if (BestPlayer != new_best_player)
        {
            BestPlayer = new_best_player;
            Pd_SetPeriod(playerToPeriod(BestPlayer));
        }

        HAL_Delay(10);
        Ammeter_Measure(playerToPeriod(BestPlayer));
    }
}

void Lattice_Stop(void)
{
    // Stop signal generation.
    Pd_CmdSignalGeneration(FALSE);

    // Stop ammeter.
    Ammeter_Stop();

    // Set power stage voltage to zero and wait till stabilized.
    Sr_SetVoltage(0.0f);
    while (!Sr_IsStabilized())
        ;

    // Stop synchronous rectifier.
    Sr_Stop();
}

/* Private functions -------------------------------------------------------*/
uint16_t findTheBestPlayer(void)
{
    float best_hand = 0.0f;
    uint16_t best_player;

    // Search the biased maxima.
    for (uint16_t player = 0; player < (sizeof(Hands) / sizeof(Hands[0])); player++)
    {
        if (Hands[player] > best_hand)
        {
            best_hand = Hands[player];
            best_player = player;
        }
    }

    return best_player;
}

static inline uint16_t playerToPeriod(uint16_t player)
{
    return (player + END_PERIOD);
}

static inline uint16_t periodToPlayer(uint16_t period)
{
    return (period - END_PERIOD);
}

static inline int16_t absint(int16_t val)
{
    if (val > 0)
    {
        return val;
    }
    else
    {
        return -val;
    }
}

/* Delegates ---------------------------------------------------------------*/
void Ammeter_MeasurementCompletedCb(float rmsCurrent)
{
    Hands[BestPlayer] = rmsCurrent;
}

void Ammeter_PitfallDetectedCb(void)
{
    Hands[BestPlayer] = __FLT_MIN__;
}