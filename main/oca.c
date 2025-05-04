
//#include esp_adc/adc_cali.h

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_master.h"
#include "driver/gptimer.h"
//#include "driver/timer.h"

//#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"

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
#define BETWEEN(value, min, max) (value < max && value > min)

static const char *TAG_MQTT = "mqtt_example";
static const char *TAG_ETH  = "eth_example";
static const char *TAG_ISR  = "isr_example";


adc_oneshot_unit_handle_t adc1_handle;
TaskHandle_t adc_task_handle = NULL;
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

typedef struct xQeue_val
{
  float iPos;
  unsigned char bByte;
  int iTime;
} xQeue_val;

//struct xQueue_val  = {0,0};
QueueHandle_t xQueue_Handler;


//====================================================================================================================
//====================================================================================================================
//====================================================================================================================

void ISR_1_Timer_handler(void* pvParameters)
{
    int  oread  = 0;
    while (1) {

        uiCurrentTime = esp_timer_get_time();
        uiTimeBetweenInterrupts = uiCurrentTime - uiLastInterruptTime;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &oread);
        ESP_LOGI(TAG_ISR, "pos : %d, time : %d", (int)oread, (int)uiTimeBetweenInterrupts);

        uiLastInterruptTime = uiCurrentTime;

    }
}

bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(adc_task_handle, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////////
void app_main(void)
{
    struct xQeue_val a = { .iPos = 1, .bByte = 0, .iTime = 1 };

    esp_log_level_set("TAG_ISR", ESP_LOG_INFO);
    xQueue_Handler = xQueueCreate (10, sizeof(xQeue_val));

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

    //adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &chan_config));

    xTaskCreatePinnedToCore(
        ISR_1_Timer_handler, 
        "ISR_1_Timer_handler", 
        2048, 
        NULL, 
        5, 
        &adc_task_handle, 
        0
    );

    gptimer_handle_t gptimer = NULL;
    // Configure timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // set alarm of timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000, // 1 sec    
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));




    gptimer_enable(gptimer);
    gptimer_start(gptimer);


    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    //xTaskCreatePinnedToCore(ISR_1_Einfahrt_handler, "ISR_1_Einfahrt_handler",   2048, NULL, 5, NULL, 0);
    //xTaskCreatePinnedToCore(ISR1_feedback,          "ISR1_feedback",  2048, NULL, 5, NULL, 0);

    
    //xTaskCreatePinnedToCore(ISR1_Timer_feedback, "ISR1_Timer_feedback", 2048, NULL, 5, NULL, 0);

    iCount = 0;


    // pin to core "1"
    // xTaskCreatePinnedToCore(mqtt_app_start, "mqtt_app_start", 4096, NULL, 5, NULL, 1);

    // clear beginning - set all interrupts online
    esp_intr_enable(ISR_1);
    // esp_log_level_set("*", ESP_LOG_DEBUG);
    ESP_LOGI(TAG_ISR, "bin da");
    //ESP_LOGI(TAG_ISR, "a.iAmount %d, a.iSet %d, a.iTime %d", a.iAmount, a.iSet, a.iTime);

    //spi_app_start();
    //mqtt_app_start();

    //while (1) {
    //    vTaskDelay (10/portTICK_PERIOD_MS);
        // ESP_LOGI(TAG_ISR, "bin da 111");
    //}
}
