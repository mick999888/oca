#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_adc_cal.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "rom/ets_sys.h"
#include "rtc_wdt.h"

#include "defines.h"
#include "globals.h"
#include "IR_Send.h"
#include "SW_Function.h"

#define WDT_TIMEOUT_S 5  // Watchdog timeout in seconds
#define TASK_LIST_BUFFER_SIZE 1024  // Buffer size for task list

static const char *TAG = "mqtt_example";

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

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

///////////////////////////////////////////////////////////////////////////////////////////
// Timer routines
////////////////////////////////////////////////////////////////////////////////////////////
static void IRAM_ATTR ISR_TIMER_MAIN1(void* arg)
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

static void IRAM_ATTR ISR_TIMER_MAIN2(void* arg)
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

static void IRAM_ATTR ISR_TIMER_SUB1(void* arg)
{
    // go on
   if ((i_4_HALT1 == 1) && (i_6_MAIN1 == 0)) { // Belegtmelder auf Hauptstrecke muss "0" sein
       i_4_HALT1 = 0;
       IR_Start();
       esp_intr_enable(ISR_1);
       esp_intr_enable(ISR_2);   
   }

   // vTaskDelay(1 / portTICK_RATE_MS);

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

   // vTaskDelay(1 / portTICK_RATE_MS);

}
////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt routines
////////////////////////////////////////////////////////////////////////////////////////////
static void IRAM_ATTR ISR_1_Einfahrt(void* args) 
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
				//if (xSemaphoreTake(xMutex, 10)) {
				    SW_Main(i_SW_R);   // Hauptstrecke verlassen
				    iFinalTelegram = 1;
				//    xSemaphoreGive(xMutex);
			    //}
	        }            
        }
    }
    uiLastInterruptTime = uiCurrentTime;
}

static void IRAM_ATTR ISR_2_IN(void* arg)  // Einfahrt - Weiche zurücksetzen
{
    i_2_IN = 1;

    if (iFinalTelegram == 1) {
        if (iCountVehicleBlock < 2) {

            switch (iCountVehicleBlock) {
            case 0:
                SW_Sub1(i_SW_R);  // ==> Sub2
                break;
            case 1:
                if ((i_3_SUB1 == 0) && (i_4_HALT1 == 0)) // Sub1 frei -> Einfahrt Sub1
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

static void IRAM_ATTR ISR_3_SUB1(void* arg) // Sub1 Einfahrt
{
    i_3_SUB1 = 1;

    // falls SUB2 frei ist
    if (i_9_SUB2 == 0)
        SW_Sub1(i_SW_R);

    esp_intr_enable(ISR_4);
}

static void IRAM_ATTR ISR_4_HALT1(void* arg) // Halt1
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

static void IRAM_ATTR ISR_5_HALT2(void* arg) // Halt2
{
    i_5_HALT2 = 1;
    IR_Blinker_aus();
    IR_Stop();

    //ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_periodic(ISR_SUB1, 6000000));
    esp_intr_disable(ISR_5); // save CPU load
    esp_intr_disable(ISR_9); // save CPU load
    esp_timer_start_periodic(ISR_SUB2, 6000000); // 1 min 

    // full house
    if ((i_5_HALT2 == 1) && (i_4_HALT1 == 1)) {
        esp_intr_disable(ISR_1);
        esp_intr_disable(ISR_2);
    }
}

static void IRAM_ATTR ISR_6_MAIN1(void* arg)  // Main1
{
    bool is_active;

    i_6_MAIN1 = 1;
    is_active = esp_timer_is_active(ISR_TIMER_MAIN1);
    if (is_active) {
        esp_timer_stop(ISR_TIMER_MAIN1);
        esp_timer_start_periodic(ISR_TIMER_MAIN1, 200000);
    }
    else {
        esp_timer_start_periodic(ISR_TIMER_MAIN1, 200000);
    }

    esp_timer_start_periodic(ISR_TIMER_MAIN1, 200000);    // Belegtmelder #2, 2 sek pause
}

static void IRAM_ATTR ISR_7_MAIN2(void* arg)  // Main2
{
	bool is_active;

	i_7_MAIN2 = 1;
	is_active = esp_timer_is_active(ISR_TIMER_MAIN2);
	if (is_active) {
		esp_timer_stop(ISR_TIMER_MAIN2);
		esp_timer_start_periodic(ISR_TIMER_MAIN2, 200000);
	}
	else {
		esp_timer_start_periodic(ISR_TIMER_MAIN2, 200000);   // Belegtmelder #1, 2 sek pause
	}
}

static void IRAM_ATTR ISR_8_BUTTON(void* arg) // Start Knopf
{
	i_8_BUTTON = 1;
}

static void IRAM_ATTR ISR_9_SUB2(void* arg) // Sub2 Einfahrt
{
	i_9_SUB2 = 1;

	// falls SUB1 frei ist
	if (i_3_SUB1 == 0)
		SW_Sub1(i_SW_L);

	esp_intr_enable(ISR_5);
}

////////////////////////////////////////////////////////////////////////////////////////////
//////////// tasks for all interrupts //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void ISR_1_Einfahrt_handler(void* pvParameters)
{
    gpio_config_t io_conf;

    // configure GPIO12_I_IN_ISR_2
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO12_I_IN_ISR_2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    //ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO12_I_IN_ISR_2, ISR_1_Einfahrt, GPIO12_I_IN_ISR_2));

    while (1) {
        vTaskDelay(1);
    }
}

void ISR_2_IN_handler (void* pvParameters)
{
    gpio_config_t io_conf;

    // configure GPIO03_I_MAIN_ISR_1 
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO03_I_MAIN_ISR_1);
    io_conf.mode         = GPIO_MODE_INPUT;      
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    //ESP_ERROR_CHECK(gpio_install_isr_service(0));                    
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO03_I_MAIN_ISR_1, ISR_2_IN, GPIO03_I_MAIN_ISR_1));

    while(1) {
        vTaskDelay(1);
    }
}

void ISR_3_SUB1_handler (void* pvParameters)
{
    gpio_config_t io_conf;

    // Configure GPIO19_I_SUB1_ISR_3
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO19_I_SUB1_ISR_3);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    //ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO19_I_SUB1_ISR_3, ISR_2_IN, GPIO19_I_SUB1_ISR_3));

    while (1) {
        vTaskDelay(1);
    }
}

void ISR_4_HALT1_handler(void* pvParameters)
{
    gpio_config_t io_conf;

    // configure GPIO17_I_HALT1_ISR_4 
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO17_I_HALT1_ISR_4);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO17_I_HALT1_ISR_4, ISR_2_IN, GPIO17_I_HALT1_ISR_4));

    while (1) {
        vTaskDelay(1);
    }
}

void ISR_5_HALT2_handler(void* pvParameters)
{
    gpio_config_t io_conf;

    // Configure GPIO34_I_HALT2_ISR_5
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO34_I_HALT2_ISR_5);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO17_I_HALT1_ISR_4, ISR_2_IN, GPIO17_I_HALT1_ISR_4));

    while (1) {
        vTaskDelay(1);
    }
}

void ISR_6_MAIN1_handler(void* pvParameters)
{
	gpio_config_t io_conf;

	// Configure GPIO22_I_MAIN1_ISR6
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.pin_bit_mask = (1ULL << GPIO22_I_MAIN1_ISR6);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO22_I_MAIN1_ISR6, ISR_2_IN, GPIO22_I_MAIN1_ISR6));

	while (1) {
		vTaskDelay(1);
	}
}

void ISR_7_MAIN2_handler(void* pvParameters)
{
	gpio_config_t io_conf;

	// Configure GPIO36_I_MAIN2_ISR7
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.pin_bit_mask = (1ULL << GPIO36_I_MAIN2_ISR7);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO36_I_MAIN2_ISR7, ISR_2_IN, GPIO36_I_MAIN2_ISR7));

	while (1) {
		vTaskDelay(1);
	}
}

void ISR_8_BUTTON_handler(void* pvParameters)
{
	gpio_config_t io_conf;

	// Configure GPIO39_I_BUTTON_ISR8
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.pin_bit_mask = (1ULL << GPIO39_I_BUTTON_ISR8);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO39_I_BUTTON_ISR8, ISR_2_IN, GPIO39_I_BUTTON_ISR8));

	while (1) {
		vTaskDelay(1);
	}
}

void ISR_9_SUB2_handler(void* pvParameters)
{
	gpio_config_t io_conf;

	// Configure GPIO35_I_SUB2_ISR_9
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.pin_bit_mask = (1ULL << GPIO35_I_SUB2_ISR_9);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
	ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO35_I_SUB2_ISR_9, ISR_2_IN, GPIO35_I_SUB2_ISR_9));

	while (1) {
		vTaskDelay(1);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////
// MQTT setup
////////////////////////////////////////////////////////////////////////////////////////////

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_handler(void *pvParameters) {
    mqtt_app_start();
    vTaskDelete(NULL);
    https://github.com/tuanpmt/esp32-mqtt/blob/master/main/app_main.c#L124
}
////////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////////
void app_main(void) 
{
    ////////////////////////////////////////////////////////////////////////////////////////////
    // W5500 / TCP setup
    ////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());

    // Create default event loop that running in background
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

    esp_err_t ret;
    // dcc signal
    esp_rom_gpio_pad_select_gpio(GPIO02_O_MAIN_BLINKER);  gpio_set_direction(GPIO02_O_MAIN_BLINKER, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO05_O_SUB1_BLINKER);  gpio_set_direction(GPIO05_O_SUB1_BLINKER, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO16_O_SUB1_EFFEKT);   gpio_set_direction(GPIO16_O_SUB1_EFFEKT,  GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO21_O_SUB2_EFFEKT);   gpio_set_direction(GPIO21_O_SUB2_EFFEKT,  GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO32_O_MAIN_BLINKER);  gpio_set_direction(GPIO32_O_MAIN_BLINKER, GPIO_MODE_OUTPUT);

    // straight signal
    esp_rom_gpio_pad_select_gpio(GPIO04_O_MAIN_WEICHE);   gpio_set_direction(GPIO04_O_MAIN_WEICHE,  GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO18_O_SUB1_WEICHE);   gpio_set_direction(GPIO18_O_SUB1_WEICHE,  GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO25_O_SUB1_STOP);     gpio_set_direction(GPIO25_O_SUB1_STOP,    GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO27_O_SUB2_STOP);     gpio_set_direction(GPIO27_O_SUB2_STOP,    GPIO_MODE_OUTPUT);

    ////////////////////////////////////////////////////////////////////////////////////////////
    // define all input pins
    ////////////////////////////////////////////////////////////////////////////////////////////
    esp_rom_gpio_pad_select_gpio(GPIO03_I_MAIN_ISR_1);    gpio_set_direction(GPIO03_I_MAIN_ISR_1,  GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO12_I_IN_ISR_2);      gpio_set_direction(GPIO12_I_IN_ISR_2,    GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO19_I_SUB1_ISR_3);    gpio_set_direction(GPIO19_I_SUB1_ISR_3,  GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO35_I_SUB2_ISR_9);    gpio_set_direction(GPIO35_I_SUB2_ISR_9,  GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO17_I_HALT1_ISR_4);   gpio_set_direction(GPIO17_I_HALT1_ISR_4, GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO34_I_HALT2_ISR_5);   gpio_set_direction(GPIO34_I_HALT2_ISR_5, GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO22_I_MAIN1_ISR6);    gpio_set_direction(GPIO22_I_MAIN1_ISR6,  GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO36_I_MAIN2_ISR7);    gpio_set_direction(GPIO36_I_MAIN2_ISR7,  GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(GPIO39_I_BUTTON_ISR8);   gpio_set_direction(GPIO39_I_BUTTON_ISR8, GPIO_MODE_INPUT);

    ////////////////////////////////////////////////////////////////////////////////////////////
    // define ethernet pins
    ////////////////////////////////////////////////////////////////////////////////////////////
    esp_rom_gpio_pad_select_gpio(GPIO33_O_INTERRUPT);     gpio_set_direction(GPIO33_O_INTERRUPT,  GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO23_O_SPI_CS);        gpio_set_direction(GPIO23_O_SPI_CS,     GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO15_I_SPI_MISO);      gpio_set_direction(GPIO15_I_SPI_MISO,   GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO13_O_SPI_MOSI);      gpio_set_direction(GPIO13_O_SPI_MOSI,   GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(GPIO15_I_SPI_MISO);      gpio_set_direction(GPIO15_I_SPI_MISO,   GPIO_MODE_INPUT);

    //  create here tasks
    vTaskDelay (1000/portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // pin to core "0"
    xTaskCreatePinnedToCore(ISR_1_Einfahrt_handler, "ISR_1_Einfahrt_handler",   2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_2_IN_handler,       "ISR_2_IN_handler",         2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_3_SUB1_handler,     "ISR_3_SUB1_handler",       2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_4_HALT1_handler,    "ISR4_HALT1_handler",       2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_5_HALT2_handler,    "ISR5_HALT2_handler",       2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_6_MAIN1_handler,    "ISR6_MAIN1_handler",       2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_7_MAIN2_handler,    "ISR7_MAIN2_handler",       2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_8_BUTTON_handler,   "ISR8_BUTTON_handler",      2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR_9_SUB2_handler,     "ISR9_SUB2_handler",        2048, NULL, 5, NULL, 0);

    // pin to core "1"
    xTaskCreatePinnedToCore(mqtt_handler, "mqtt_handler", 4096, NULL, 5, NULL, 1);
    

    // clear beginning - set all interrupts online
    esp_intr_enable(ISR_1);
    esp_intr_enable(ISR_2);
    esp_intr_enable(ISR_3);
    esp_intr_enable(ISR_4);

    while (1) {
        vTaskDelay(1);  
    }
}