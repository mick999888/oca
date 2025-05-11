#ifndef GLOBALS_H
#define GLOBALS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"

extern volatile uint64_t   uiTimeBetweenInterrupts; 
extern volatile uint64_t   uiCurrentTime;          
extern volatile uint64_t   uiLastInterruptTime;   

extern volatile int        iBitSet;              
extern volatile int        iCount;             
extern volatile int        iSet_preamble;   
extern volatile int        iSet_Turnover;  
extern volatile int        iCount_Byte;
extern volatile int        iFinalTelegram;
extern volatile int        iStream;
extern volatile int        iLongBreak;

extern volatile int        iCountVehicleBlock;

extern volatile int        i_2_IN;
extern volatile int        i_3_SUB1;
extern volatile int        i_9_SUB2;
extern volatile int        i_4_HALT1;
extern volatile int        i_5_HALT2;
extern volatile int        i_6_MAIN1;
extern volatile int        i_7_MAIN2;
extern volatile int        i_8_BUTTON;

extern volatile int        iBufVal;
extern volatile int        iSetBig;

extern volatile uint8_t bByte[10];
extern volatile unsigned char aOut[80];
extern volatile unsigned char bOut[80];

extern bool bTrigger;

extern volatile unsigned int iBufOUT[10];
extern volatile unsigned int iAverageBuf;

//extern adc_oneshot_unit_handle_t adc1_handle;

#endif