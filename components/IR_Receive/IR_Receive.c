#include "defines.h"
#include "globals.h"
#include "SW_Function.h"
#include "IR_Receive.h"
#include "IR_Send.h"
#include "oca.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//#define GPIO03_PIN34_I_MAIN_ISR_1   34  // ISR_1 Erkennung Haupteinfahrt 
//#define GPIO12_PIN13_I_IN_ISR_2     13  // ISR_2 Erkennung Einfahrt
//#define GPIO15_PIN23_I_SUB1_ISR_3   23  // ISR_3 Erkennung Sub 1
//#define GPIO35_PIN09_I_SUB2_ISR_9    6  // ISR_9 Erkennung Sub 2
//#define GPIO17_PIN28_I_HALT1_ISR_4  28  // ISR_4 Haltestelle 1
//#define GPIO34_PIN05_I_HALT2_ISR_5   5  // ISR_5 Haltestelle 2
//#define GPIO22_PIN36_I_MAIN1_ISR6   36  // ISR_6 Hauptstrecke Belegtmelder Sub1 
//#define GPIO26_PIN10_I_MAIN2_ISR7   10  // ISR_7 Hauptstrecke Belegtmelder Sub2
//#define GPIO33_PIN08_I_BUTTON_ISR8   8  // ISR_8 Taste Bus start


void IRAM_ATTR ISR_TIMER_SUB1(void* arg)
{
  static bool ON;
  ON = !ON;
}

void IRAM_ATTR ISR_TIMER_SUB2(void* arg)
{
  static bool ON;
  ON = !ON;
}

void IRAM_ATTR ISR_8_BUTTON(void* arg) // Start Knopf
{
	i_8_BUTTON = 1;
}

void IRAM_ATTR ISR_7_MAIN2(void* arg)  // Main2
{
	i_7_MAIN2 = 1;
}

void IRAM_ATTR ISR_6_MAIN1(void* arg)  // Main1
{
	i_6_MAIN1 = 1;
}

void IRAM_ATTR ISR_5_HALT2(void* arg) // Halt2
{
	i_5_HALT2 = 1;
	IR_Blinker_aus();
	IR_Stop();
	//ESP_ERROR_CHECK(esp_timer_start_periodic(ISR_SUB1, 6000000));
	esp_intr_disable(ISR_SUB1);
	esp_timer_start_periodic(ISR_SUB2, 6000000);
}

void IRAM_ATTR ISR_4_HALT1(void* arg) // Halt1
{
	i_4_HALT1 = 1;
	IR_Blinker_aus();
	IR_Stop();
}

void IRAM_ATTR ISR_9_SUB2(void* arg) // Sub2 Einfahrt
{
    i_9_SUB2 = 1;

	// falls SUB1 frei ist
	if (i_3_SUB1 == 0)
	    SW_Sub1(i_SW_L);
}

void IRAM_ATTR ISR_3_SUB1(void* arg) // Sub1 Einfahrt
{
	i_3_SUB1 = 1;

    // falls SUB2 frei ist
	if (i_9_SUB2 == 0)
	    SW_Sub1(i_SW_R);
}

void IRAM_ATTR ISR_2_IN(void* arg)  // Einfahrt - Weiche zurücksetzen
{
	i_2_IN = 1;

	if (iFinalTelegram == 1) {
		if (iCountVehicleBlock < 2) {

			switch (iCountVehicleBlock) {
				case 0:
				    SW_Sub1(i_SW_R);  // ==> Sub2
				    break;
				case 1:
				    if      ((i_3_SUB1 == 0) && (i_4_HALT1 == 0)) // Sub1 frei -> Einfahrt Sub1
				        SW_Sub1(i_SW_L);
					else if ((i_9_SUB2 == 0) && (i_5_HALT2 == 0)) // Sub2 frei -> Einfahrt Sub2
					    SW_Sub1(i_SW_R);
					break;
			}

		    if (xSemaphoreTake(xMutex, 10)) {
				iCountVehicleBlock++;
				SW_Main(i_SW_L);       // schalten auf Hauptstrecke
				iFinalTelegram = 0;
				if (iCountVehicleBlock == 2) {
				    esp_intr_disable(ISR_1);
					esp_intr_disable(ISR_2);
				}
				xSemaphoreGive(xMutex);
			}
		}
	}
}

void IRAM_ATTR ISR_1_Einfahrt(void* arg) 
{

    uiCurrentTime = esp_timer_get_time();
    uiTimeBetweenInterrupts = uiCurrentTime - uiLastInterruptTime;

    // -> detect pur "0"
    if ((210 < uiTimeBetweenInterrupts) && (uiTimeBetweenInterrupts < 240)) {

        // set bit to "0"
        iBitSet = 0;

	    // start bit   
	    if ((iCount == 8) && (iSet_preamble == 0))  {
		    iCount_Byte++;
		    iCount = 0;
	    }    

	    // part of eight bit word - collect here "0"
	    else if ((iCount > 8) && (iSet_preamble == 0))  {
		    bByte[iCount_Byte] = (bByte[iCount_Byte] << 1) | 0;
	    }            
    }

	// -> detect pur "1"
	else if ((109 < uiTimeBetweenInterrupts ) && (uiTimeBetweenInterrupts < 130)) {

        // set bit to "1"
        iBitSet = 1;

        // work off preamble first
	    if (iSet_preamble == 1) {
		    iCount++;
	    }
	  
	    // final stop bit
	    else if ((iCount == 8) && ((iSet_preamble == 0))) {
		    iCount = 0;	
		    iFinalTelegram = 1;	  
	    }
	  
	    // part of eight bit word - collect here "1"
	    else if ((iCount > 8) && (iSet_preamble == 0))  {
		    iCount++;
		    bByte[iCount_Byte] = (bByte[iCount_Byte] << 1) | 1;
	    }
    }

	// -> detect "transmit" bit
	else if ((160 < uiTimeBetweenInterrupts) && (uiTimeBetweenInterrupts < 190)) {

        // detect here "0"
        if (iBitSet == 1) {

            iBitSet = 0;

	        // first start bit after preamble - start here with normal protocol
            // finish preamble
	        if ((iCount < 11) && (iSet_preamble == 1)) {
		        iCount = 0;
		        iSet_preamble = 0;
	        }            

	        // start bit	    
	        if ((iCount == 8) && (iSet_preamble == 0))  {
		        iCount_Byte++;
		        iCount = 0;
	        }    

	        // part of eight bit word - collect here "0"
	        else if ((iCount > 8) && (iSet_preamble == 0))  {
                iCount++;
		        bByte[iCount_Byte] = (bByte[iCount_Byte] << 1) | 0;
	        }                

        }

        // detect here "1"
        if (iBitSet == 0) {

            iBitSet = 1;

	        // part of eight bit word - collect here "1"
	        if ((iCount > 8) && (iSet_preamble == 0))  {
		        iCount++;
		        bByte[iCount_Byte] = (bByte[iCount_Byte] << 1) | 1;
	        }

	        // final stop bit
	        else if ((iCount == 8) && ((iSet_preamble == 0))) {
		        iCount = 0;	
				if (xSemaphoreTake(xMutex, 10)) {
				    SW_Main(i_SW_R);   // Hauptstrecke verlassen
				    iFinalTelegram = 1;
				    xSemaphoreGive(xMutex);
			    }
	        }            

        }

    }

    uiLastInterruptTime = uiCurrentTime;

}