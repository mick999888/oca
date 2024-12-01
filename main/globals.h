#ifndef GLOBALS_H
#define GLOBALS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

extern volatile uint32_t   uiTimeBetweenInterrupts; 
extern volatile uint32_t   uiCurrentTime;          
extern volatile uint32_t   uiLastInterruptTime;   

extern volatile int        iBitSet;              
extern volatile int        iCount;             
extern volatile int        iSet_preamble;     
extern volatile int        iCount_Byte;
extern volatile int        iFinalTelegram;

extern volatile int        iCountVehicleBlock;

extern volatile int        i_2_IN;
extern volatile int        i_3_SUB1;
extern volatile int        i_9_SUB2;
extern volatile int        i_4_HALT1;
extern volatile int        i_5_HALT2;
extern volatile int        i_6_MAIN1;
extern volatile int        i_7_MAIN2;
extern volatile int        i_8_BUTTON;

extern volatile unsigned char bByte[10];

#endif