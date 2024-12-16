#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/semphr.h"
#include "driver/timer.h"

#include "mqtt_client.h"
// #include "mqtt_client_priv.h"
// #include "mqtt_msg.h"
// #include "mqtt_outbox.h"

#include "globals.h"
#include "defines.h"
#include "IR_Send.h"
#include "SW_Function.h"

////////////////////////////////////////////////////////////////////////////////////////////
// Semaphore + ISR hanlde setup
////////////////////////////////////////////////////////////////////////////////////////////
SemaphoreHandle_t xMutex = NULL;
intr_handle_t ISR_1;
intr_handle_t ISR_2;
intr_handle_t ISR_3;
intr_handle_t ISR_4;
intr_handle_t ISR_5;
intr_handle_t ISR_6;
intr_handle_t ISR_7;
intr_handle_t ISR_8;
intr_handle_t ISR_9;
esp_timer_handle_t ISR_SUB1;
esp_timer_handle_t ISR_SUB2;
esp_timer_handle_t ISR_MAIN1;
esp_timer_handle_t ISR_MAIN2;


////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt routines
////////////////////////////////////////////////////////////////////////////////////////////

void IRAM_ATTR ISR_TIMER_MAIN1(void* arg)
{
  i_2_IN    = 0;
  i_3_SUB1  = 0;
  i_4_HALT1 = 0;
  i_6_MAIN1 = 0;
  esp_intr_enable(ISR_1);
  esp_intr_enable(ISR_2);     
  esp_intr_enable(ISR_3);   
  IR_Start();  
}

void IRAM_ATTR ISR_TIMER_MAIN2(void* arg)
{
  i_2_IN    = 0;
  i_9_SUB2  = 0;
  i_5_HALT2 = 0;
  i_7_MAIN2 = 0;
  esp_intr_enable(ISR_1);
  esp_intr_enable(ISR_2);     
  esp_intr_enable(ISR_9);   
  IR_Start();      
}

void IRAM_ATTR ISR_TIMER_SUB1(void* arg)
{
    // go on
   if ((i_4_HALT1 == 1) && (i_6_MAIN1 == 0)) { // Belegtmelder auf Hauptstrecke muss "0" sein
       i_4_HALT1 = 0;
       IR_Start();
       esp_intr_enable(ISR_1);
       esp_intr_enable(ISR_2);   
   }
}

void IRAM_ATTR ISR_TIMER_SUB2(void* arg)
{
    // go on
   if ((i_5_HALT2 == 1) && (i_7_MAIN2 == 0)) { // Belegtmelder auf Hauptstrecke muss "0" sein
       i_5_HALT2 = 0;
       IR_Start();
       esp_intr_enable(ISR_1);
       esp_intr_enable(ISR_2);   

   }
}

void IRAM_ATTR ISR_8_BUTTON(void* arg) // Start Knopf
{
	i_8_BUTTON = 1;
}

void IRAM_ATTR ISR_7_MAIN2(void* arg)  // Main2
{
    bool is_active;

	i_7_MAIN2 = 1;
    is_active = esp_timer_is_active(ISR_TIMER_MAIN2);
    if (is_active) {
      esp_timer_stop(ISR_TIMER_MAIN2);
      esp_timer_start_periodic(ISR_TIMER_MAIN2, 200000);
    } else {
      esp_timer_start_periodic(ISR_TIMER_MAIN2, 200000);   // Belegtmelder #1, 2 sek pause
    }
}

void IRAM_ATTR ISR_6_MAIN1(void* arg)  // Main1
{
    bool is_active;

	i_6_MAIN1 = 1;
    is_active = esp_timer_is_active(ISR_TIMER_MAIN1);
    if (is_active) {
      esp_timer_stop(ISR_TIMER_MAIN1);
      esp_timer_start_periodic(ISR_TIMER_MAIN1,200000);
    } else {
      esp_timer_start_periodic(ISR_TIMER_MAIN1, 200000);
    }

    esp_timer_start_periodic(ISR_TIMER_MAIN1, 200000);    // Belegtmelder #2, 2 sek pause
}

void IRAM_ATTR ISR_5_HALT2(void* arg) // Halt2
{
	i_5_HALT2 = 1;
	IR_Blinker_aus();
	IR_Stop();

	//ESP_ERROR_CHECK(esp_timer_start_periodic(ISR_SUB1, 6000000));
	esp_intr_disable(ISR_5); // save CPU load
    esp_intr_disable(ISR_9); // save CPU load
	esp_timer_start_periodic(ISR_SUB2, 6000000); // 1 min 

    // full house
    if ((i_5_HALT2 == 1) && (i_4_HALT1 == 1)) {
        esp_intr_disable(ISR_1);
        esp_intr_disable(ISR_2);        
    }
}

void IRAM_ATTR ISR_4_HALT1(void* arg) // Halt1
{
	i_4_HALT1 = 1;
	IR_Blinker_aus();
	IR_Stop();

	esp_intr_disable(ISR_3); // save CPU load
    esp_intr_disable(ISR_4); // save CPU load
	esp_timer_start_periodic(ISR_SUB1, 6000000); // 1 min 

    // full house
    if ((i_5_HALT2 == 1) && (i_4_HALT1 == 1)) {
        esp_intr_disable(ISR_1);
        esp_intr_disable(ISR_2);        
    }    
}

void IRAM_ATTR ISR_9_SUB2(void* arg) // Sub2 Einfahrt
{
    i_9_SUB2 = 1;

	// falls SUB1 frei ist
	if (i_3_SUB1 == 0)
	    SW_Sub1(i_SW_L);
    
    esp_intr_enable(ISR_5);
}

void IRAM_ATTR ISR_3_SUB1(void* arg) // Sub1 Einfahrt
{
	i_3_SUB1 = 1;

    // falls SUB2 frei ist
	if (i_9_SUB2 == 0)
	    SW_Sub1(i_SW_R);

    esp_intr_enable(ISR_4);
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

////////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////////


void app_main(void) 
{
    ////////////////////////////////////////////////////////////////////////////////////////////
    // define here all ISR routines
    ////////////////////////////////////////////////////////////////////////////////////////////

    const esp_timer_create_args_t sub1_timer_args = {
      .callback = &ISR_TIMER_SUB1,
      .name = "TIMER_SUB1"};
    esp_timer_handle_t ISR_SUB1;
    ESP_ERROR_CHECK(esp_timer_create(&sub1_timer_args, &ISR_SUB1));

    const esp_timer_create_args_t sub2_timer_args = {
      .callback = &ISR_TIMER_SUB2,
      .name = "TIMER_SUB2"};
    esp_timer_handle_t ISR_SUB2;
    ESP_ERROR_CHECK(esp_timer_create(&sub2_timer_args, &ISR_SUB2));

    const esp_timer_create_args_t main1_timer_args = {
      .callback = &ISR_TIMER_MAIN1,
      .name = "TIMER_MAIN1"};
    esp_timer_handle_t ISR_MAIN1;
    ESP_ERROR_CHECK(esp_timer_create(&main1_timer_args, &ISR_MAIN1));

    const esp_timer_create_args_t main2_timer_args = {
      .callback = &ISR_TIMER_MAIN2,
      .name = "TIMER_MAIN2"};
    esp_timer_handle_t ISR_MAIN2;
    ESP_ERROR_CHECK(esp_timer_create(&main1_timer_args, &ISR_MAIN2));

    //gpio_config_t io_conf = {
    //    .intr_type = GPIO_INTR_POSEDGE, // Interrupt on high level
    //    .pin_bit_mask = (1ULL << GPIO03_I_MAIN_ISR_1)|(1ULL << GPIO12_I_IN_ISR_2)|(1ULL << GPIO15_O_SPI_CS)|(1ULL << GPIO35_I_SUB2_ISR_9)|(1ULL << GPIO17_I_HALT1_ISR_4)|(1ULL << GPIO34_I_HALT2_ISR_5)|(1ULL << GPIO03_I_MAIN_ISR_1)|(1ULL << GPIO26_I_MAIN2_ISR7)|(1ULL << GPIO33_I_BUTTON_ISR8), // Bit mask of the pin
    //    .mode = GPIO_MODE_INPUT, // Set as input mode
    //    .pull_up_en = GPIO_PULLUP_DISABLE, // Disable pull-up
    //    .pull_down_en = GPIO_PULLUP_ENABLE // Disable pull-down
    //};

    gpio_config_t io_conf;

    // configure GPIO03_I_MAIN_ISR_1 
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO03_I_MAIN_ISR_1);
    io_conf.mode         = GPIO_MODE_INPUT;      
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure GPIO12_I_IN_ISR_2
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO12_I_IN_ISR_2);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;    
    gpio_config(&io_conf);

    // configure GPIO15_O_SPI_CS 
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO15_O_SPI_CS);
    io_conf.mode         = GPIO_MODE_INPUT;      
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure GPIO35_I_SUB2_ISR_9
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO35_I_SUB2_ISR_9);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;    
    gpio_config(&io_conf);

    // configure GPIO17_I_HALT1_ISR_4 
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO17_I_HALT1_ISR_4);
    io_conf.mode         = GPIO_MODE_INPUT;      
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure GPIO34_I_HALT2_ISR_5
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO34_I_HALT2_ISR_5);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;    
    gpio_config(&io_conf);

    // configure GPIO26_I_MAIN2_ISR7 
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO26_I_MAIN2_ISR7);
    io_conf.mode         = GPIO_MODE_INPUT;      
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure GPIO33_I_BUTTON_ISR8
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO33_I_BUTTON_ISR8);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;    
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL5); // Use level 5 interrupt

    // ISR #1 - Hauptstrecke
    //intr_handle_t ISR_1;
    gpio_isr_handler_add(GPIO03_I_MAIN_ISR_1, ISR_1_Einfahrt, (void*)GPIO03_I_MAIN_ISR_1);
    esp_err_t ISR_1_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_1_Einfahrt, (void*) GPIO03_I_MAIN_ISR_1, &ISR_1);
    if (ISR_1_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_1_err));
    }

    // ISR #2 - Eingang
    //intr_handle_t ISR_2;
    gpio_isr_handler_add(GPIO12_I_IN_ISR_2, ISR_2_IN, (void*)GPIO12_I_IN_ISR_2);
    esp_err_t ISR_2_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_2_IN, (void*) GPIO12_I_IN_ISR_2, &ISR_2);
    if (ISR_2_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_2_err));
    }

    // ISR #3 - Sub1
    //intr_handle_t ISR_3;
    gpio_isr_handler_add(GPIO15_O_SPI_CS, ISR_3_SUB1, (void*)GPIO15_O_SPI_CS);
    esp_err_t ISR_3_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_3_SUB1, (void*) GPIO15_O_SPI_CS, &ISR_3);
    if (ISR_3_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_3_err));
    }


    // ISR #9 - Sub2
    //intr_handle_t ISR_9;
    gpio_isr_handler_add(GPIO35_I_SUB2_ISR_9, ISR_9_SUB2, (void*)GPIO35_I_SUB2_ISR_9);
    esp_err_t ISR_9_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_9_SUB2, (void*) GPIO35_I_SUB2_ISR_9, &ISR_9);
    if (ISR_9_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_9_err));
    }


    // ISR #4 - Halt1
    //intr_handle_t ISR_4;
    gpio_isr_handler_add(GPIO17_I_HALT1_ISR_4, ISR_4_HALT1, (void*)GPIO17_I_HALT1_ISR_4);
    esp_err_t ISR_4_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_4_HALT1, (void*) GPIO17_I_HALT1_ISR_4, &ISR_4);
    if (ISR_4_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_4_err));
    }

    // ISR #5 - Halt2
    //intr_handle_t ISR_5;
    gpio_isr_handler_add(GPIO34_I_HALT2_ISR_5, ISR_5_HALT2, (void*)GPIO34_I_HALT2_ISR_5);
    esp_err_t ISR_5_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_5_HALT2, (void*) GPIO34_I_HALT2_ISR_5, &ISR_5);
    if (ISR_5_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_5_err));
    }

    // ISR #6 - Main1
    //intr_handle_t ISR_6;
    gpio_isr_handler_add(GPIO22_I_MAIN1_ISR6, ISR_6_MAIN1, (void*)GPIO22_I_MAIN1_ISR6);
    esp_err_t ISR_6_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_6_MAIN1, (void*) GPIO22_I_MAIN1_ISR6, &ISR_6);
    if (ISR_6_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_6_err));
    }

    // ISR #7 - Main2
    //intr_handle_t ISR_7;
    gpio_isr_handler_add(GPIO26_I_MAIN2_ISR7, ISR_7_MAIN2, (void*)GPIO26_I_MAIN2_ISR7);
    esp_err_t ISR_7_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_7_MAIN2, (void*) GPIO26_I_MAIN2_ISR7, &ISR_7);
    if (ISR_7_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_7_err));
    }

    // ISR #8 - button start bus
    //intr_handle_t ISR_8;
    gpio_isr_handler_add(GPIO33_I_BUTTON_ISR8, ISR_8_BUTTON, (void*)GPIO33_I_BUTTON_ISR8);
    esp_err_t ISR_8_err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, ISR_8_BUTTON, (void*) GPIO33_I_BUTTON_ISR8, &ISR_8);
    if (ISR_8_err != ESP_OK) {
        ESP_LOGE("app_main", "Failed to allocate interrupt: %s", esp_err_to_name(ISR_8_err));
    }

    // clear beginning - set all interrupts online
    esp_intr_enable(ISR_1);
    esp_intr_enable(ISR_2);
    esp_intr_enable(ISR_3);    
    esp_intr_enable(ISR_4);
    esp_intr_enable(ISR_5);
    // esp_intr_enable(ISR_6); Main 1
    // esp_intr_enable(ISR_7); Main 2   
    // esp_intr_enable(ISR_8); Button
    esp_intr_enable(ISR_9);

    ////////////////////////////////////////////////////////////////////////////////////////////
    // define all output pins
    ////////////////////////////////////////////////////////////////////////////////////////////

    // dcc signal
    esp_rom_gpio_pad_select_gpio(GPIO02_O_MAIN_BLINKER);  gpio_set_direction(GPIO02_O_MAIN_BLINKER, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO05_O_SUB1_BLINKER);  gpio_set_direction(GPIO05_O_SUB1_BLINKER, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO16_O_SUB1_EFFEKT);   gpio_set_direction(GPIO16_O_SUB1_EFFEKT, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO21_O_SUB2_EFFEKT);   gpio_set_direction(GPIO21_O_SUB2_EFFEKT, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO32_O_MAIN_BLINKER);  gpio_set_direction(GPIO32_O_MAIN_BLINKER, GPIO_MODE_OUTPUT);

    // straight signal
    esp_rom_gpio_pad_select_gpio(GPIO04_O_MAIN_WEICHE);   gpio_set_direction(GPIO04_O_MAIN_WEICHE, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO18_O_SUB1_WEICHE);   gpio_set_direction(GPIO18_O_SUB1_WEICHE, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO25_O_SUB1_STOP);     gpio_set_direction(GPIO25_O_SUB1_STOP, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO27_O_SUB2_STOP);     gpio_set_direction(GPIO27_O_SUB2_STOP, GPIO_MODE_OUTPUT);

/*
    xTaskCreatePinnedToCore(IR_Blinker_rechts_ein, "IR_Blinker_rechts_ein", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(IR_Blinker_links_ein, "IR_Blinker_links_ein",  2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(IR_Blinker_aus, "IR_Blinker_aus",  2048, NULL, 5, NULL, 0);
    
    xTaskCreatePinnedToCore(IR_Stop, "IR_Stop", 2048, NULL, 5, NULL, 0);  
    xTaskCreatePinnedToCore(IR_Start, "IR_Start", 2048, NULL, 5, NULL, 0);

    xTaskCreatePinnedToCore(SW_Main, "SW_Main", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(SW_Sub1, "SW_Sub1", 2048, NULL, 5, NULL, 0);
*/



    // Set the GPIO pin high
    // gpio_set_level(GPIO04_PIN26_O_MAIN_WEICHE, 1);

    xMutex = xSemaphoreCreateBinary();


    while (1) {

        

        //    portDISABLE_INTERRUPTS();
        //    printf( "%d %d %d", bByte[0], bByte[1], bByte[2]);
        //    portENABLE_INTERRUPTS();        }

    }
}
