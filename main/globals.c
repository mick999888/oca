#include "globals.h"

volatile uint64_t   uiTimeBetweenInterrupts = 0;
volatile uint64_t   uiCurrentTime           = 0;
volatile uint64_t   uiLastInterruptTime     = 0;

volatile int        iBitSet                 = 0;
volatile int        iCount                  = 0;
volatile int        iSet_preamble           = 1;
volatile int        iSet_Turnover           = 0;
volatile int        iCount_Byte             = 0;
volatile int        iFinalTelegram          = 0;

volatile int        iCountVehicleBlock      = 0;
volatile int        iStream                 = 0;
volatile int        iLongBreak              = 0;

volatile int        i_2_IN                     = 0;
volatile int        i_3_SUB1                   = 0;
volatile int        i_9_SUB2                   = 0;
volatile int        i_4_HALT1                  = 0;
volatile int        i_5_HALT2                  = 0;
volatile int        i_6_MAIN1                  = 0;
volatile int        i_7_MAIN2                  = 0;
volatile int        i_8_BUTTON                 = 0;

volatile uint8_t bByte[10];