
//#include esp_adc/adc_cali.h

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_master.h"
#include "driver/timer.h"

#include "esp_check.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
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
//#include "ledc.h"

#include "defines.h"
#include "globals.h"
#include "IR_Send.h"
#include "SW_Function.h"

#define WDT_TIMEOUT_S 5  // Watchdog timeout in seconds
#define TASK_LIST_BUFFER_SIZE 1024  // Buffer size for task list

static const char *TAG_MQTT = "mqtt_example";
static const char *TAG_ETH  = "eth_example";
static const char *TAG_ISR  = "isr_example";

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


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG_MQTT, "Last error %s: 0x%x", message, error_code);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt routines
////////////////////////////////////////////////////////////////////////////////////////////
static void IRAM_ATTR ISR_1_Einfahrt(void *args) 
{
    
    uiCurrentTime = esp_timer_get_time();
    uiTimeBetweenInterrupts = uiCurrentTime - uiLastInterruptTime;

    // -> detect "0"
    if ((210 < uiTimeBetweenInterrupts) && (uiTimeBetweenInterrupts < 250)) {

        iStream = 0;

	    // start bit   
	    if ((iCount == 8) && (iSet_preamble == 0))  {
		    iCount_Byte++;
		    iCount = 0;
	    }    

	    // part of eight bit word - collect here "0"
	    else if ((iCount < 8) && (iSet_preamble == 0))  {
                switch (iCount) {
                    case 0: bByte[iCount_Byte] &=~1; break;
                    case 1: bByte[iCount_Byte] &=~2; break;  
                    case 2: bByte[iCount_Byte] &=~4; break;
                    case 3: bByte[iCount_Byte] &=~8; break;  
                    case 4: bByte[iCount_Byte] &=~16; break;
                    case 5: bByte[iCount_Byte] &=~32; break;  
                    case 6: bByte[iCount_Byte] &=~64; break;
                    case 7: bByte[iCount_Byte] &=~128; break;  
                }
            iCount++;            
	    }       
        
        else if ((10 < iCount) && (iSet_preamble == 1)) {
            iCount_Byte = 0;
            iCount = 0;
            iSet_preamble = 0;
            //Memset(bByte, 0, sizeof(bByte));
            bByte[0] = 0;
            bByte[1] = 0;
            bByte[2] = 0;
            bByte[3] = 0;
            bByte[4] = 0;
            bByte[5] = 0;
            bByte[6] = 0;
            bByte[7] = 0;

        }          
    }

	// -> detect "1"
	else if ((100 < uiTimeBetweenInterrupts ) && (uiTimeBetweenInterrupts < 130)) {

        iStream++;

        // work off preamble first
	    if (iSet_preamble == 1) {
		    iCount++;
	    }
	  
	    // final stop bit
	    //else if ((iCount == 8) && ((iSet_preamble == 0))) {		    
		//    iFinalTelegram = 1;	  
        //    iSet_preamble = 1;
        //    iCount_Byte = 0;
        //    iCount = 0;
	    //}
	  
	    // part of eight bit word - collect here "1"
	    else if ((iCount < 8) && (iSet_preamble == 0))  {
            switch (iCount) {
                case 0: bByte[iCount_Byte] |=   1; break;
                case 1: bByte[iCount_Byte] |=   2; break;  
                case 2: bByte[iCount_Byte] |=   4; break;
                case 3: bByte[iCount_Byte] |=   8; break;  
                case 4: bByte[iCount_Byte] |=  16; break;
                case 5: bByte[iCount_Byte] |=  32; break;  
                case 6: bByte[iCount_Byte] |=  64; break;
                case 7: bByte[iCount_Byte] |= 128; break;  
            }
            iCount++;
	    }

        if (8 < iStream) {
            iSet_preamble = 1;
            iCount_Byte = 0;
            iCount = iStream;
            iStream = 0;
        }        
    }

    uiLastInterruptTime = uiCurrentTime;
}

////////////////////////////////////////////////////////////////////////////////////////////
//////////// tasks for all interrupts //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void ISR1_feedback(void* arg) {
    while (1) {
        //if (iFinalTelegram == 1) {
            ESP_LOGI(TAG_ISR, "preamble : %d  stream : %d -- 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", iSet_preamble, iStream, bByte[0], bByte[1], bByte[2], bByte[3], bByte[5], bByte[6]);    
            //iFinalTelegram = 0; // Reset the flag
        //}
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to avoid busy-waiting
    }
}


void ISR_1_Einfahrt_handler(void* pvParameters)
{
    gpio_config_t io_conf;

    // configure GPIO03_I_MAIN_ISR_1
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO03_I_MAIN_ISR_1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    //ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO03_I_MAIN_ISR_1, ISR_1_Einfahrt, (void*)GPIO03_I_MAIN_ISR_1));

    ESP_LOGI(TAG_ISR, "ISR 1 handler");
    //while (1) {
    //    vTaskDelay (10/portTICK_PERIOD_MS);
    //    // vTaskDelay(1);
    //}

    vTaskDelete(NULL);
}


////////////////////////////////////////////////////////////////////////////////////////////
// SPI setup
////////////////////////////////////////////////////////////////////////////////////////////
static void spi_app_start(void) {
    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO15_I_SPI_MISO,
        .mosi_io_num = GPIO13_O_SPI_MOSI,
        .sclk_io_num = GPIO14_O_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_handle_t spi_handle; 

    // Configure SPI device interface
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1 * 1000 * 1000,
        .spics_io_num = GPIO23_O_SPI_CS,
        .flags = 0,
        .queue_size = 1,
    };
    
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle));

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
}
////////////////////////////////////////////////////////////////////////////////////////////
// Ethernet setup
////////////////////////////////////////////////////////////////////////////////////////////
 /** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG_ETH, "Ethernet Link Up");
        ESP_LOGI(TAG_ETH, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_ETH, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG_ETH, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG_ETH, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG_ETH, "Ethernet Got IP Address");
    ESP_LOGI(TAG_ETH, "~~~~~~~~~~~");
    ESP_LOGI(TAG_ETH, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG_ETH, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG_ETH, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG_ETH, "~~~~~~~~~~~");
}
////////////////////////////////////////////////////////////////////////////////////////////
// PWM setup
////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////
// MQTT setup
////////////////////////////////////////////////////////////////////////////////////////////
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG_MQTT, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG_MQTT, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
        ESP_LOGI(TAG_MQTT, "DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

void  mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = "mqtts://mqtt.example.com:1883",
      .credentials.username = "user",
      .credentials.authentication.password = "pass",
      .credentials.client_id = "esp32_client",
      .session.last_will.topic = "/lwt",
      .session.last_will.msg = "offline",
      .session.last_will.qos = 1,
      .session.last_will.retain = 1};
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);  // Start the MQTT client        
    

    vTaskDelete(NULL);
    //https://github.com/tuanpmt/esp32-mqtt/blob/master/main/app_main.c#L124
    //https://medium.com/gravio-edge-iot-platform/how-to-set-up-a-mosquitto-mqtt-broker-securely-using-client-certificates-82b2aaaef9c8
    //https://stackoverflow.com/questions/34693520/mqtt-server-with-ssl-tls-error-unable-to-load-server-key-file
}
////////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////////
void app_main(void) 
{
    esp_log_level_set("TAG_ISR", ESP_LOG_INFO);
    ESP_LOGI(TAG_ISR, "bin da");


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
    // vTaskDelay (10/portTICK_PERIOD_MS);

/*

    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO03_I_MAIN_ISR_1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // pin to core "0"
    //xTaskCreatePinnedToCore(ISR_1_Einfahrt_handler, "ISR_1_Einfahrt_handler",   2048, NULL, 5, NULL, 0);
    // configure GPIO03_I_MAIN_ISR_1
    gpio_config(&io_conf);
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO03_I_MAIN_ISR_1, ISR_1_Einfahrt, (void*)GPIO03_I_MAIN_ISR_1));    

*/

    


    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    xTaskCreatePinnedToCore(ISR_1_Einfahrt_handler, "ISR_1_Einfahrt_handler",   2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ISR1_feedback,          "ISR1_feedback",  2048, NULL, 5, NULL, 0);

    iCount = 0;
    

    // pin to core "1"
    // xTaskCreatePinnedToCore(mqtt_app_start, "mqtt_app_start", 4096, NULL, 5, NULL, 1);

    // clear beginning - set all interrupts online
    esp_intr_enable(ISR_1);
    // esp_log_level_set("*", ESP_LOG_DEBUG);
    ESP_LOGI(TAG_ISR, "bin da");

    //spi_app_start();
    //mqtt_app_start();

    //while (1) {
    //    vTaskDelay (10/portTICK_PERIOD_MS);
        // ESP_LOGI(TAG_ISR, "bin da 111");
    //}
}