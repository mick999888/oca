
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
#include <esp_task_wdt.h>

//#include "driver/i2s.h"
#include "driver/i2s_std.h" 
#include "driver/i2s_pdm.h" 
#include "driver/i2s_tdm.h" 

//#include "ledc.h"

#include "defines.h"
#include "globals.h"
#include "IR_Send.h"
#include "SW_Function.h"

#define WDT_TIMEOUT_S 5  // Watchdog timeout in seconds
#define TASK_LIST_BUFFER_SIZE 1024  // Buffer size for task list
#define BETWEEN(value, min, max) (value < max && value > min)

// I2S
#define I2S_NUM         (0)
#define SAMPLE_RATE     (1000000) // 1 MHz requested
#define BUF_LEN         (1024)

static const char *TAG_MQTT = "mqtt_example";
static const char *TAG_ETH  = "eth_example";
static const char *TAG_ISR  = "isr_example";

// queues defined
static QueueHandle_t queue_result = NULL;

// tasks defined
TaskHandle_t handle_task_adc = NULL;

adc_oneshot_unit_handle_t adc1_handle;
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

void task_timer_handler(void* pvParameters)
{
    int  oread   = 0, iBuf = 0;
    int  iMin = 0, iMax = 0; 
    int  iSum = 0, iAverageNow = 0;
    int  iCnt    = 0;
    int  iBufIN[10];

    //esp_task_wdt_add(NULL);

    while (1) {


        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uiLastInterruptTime = esp_timer_get_time();
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &oread);
        uiCurrentTime = esp_timer_get_time();
        uiTimeBetweenInterrupts = uiCurrentTime - uiLastInterruptTime;  
        ESP_LOGI(TAG_ISR, "min : %d,  time : %d", (int)oread, (int)uiTimeBetweenInterrupts); 
    }
} 
    /*
        if (oread < 1000) {
            if (oread < iBuf) {
                iMax = iBuf;
                ESP_LOGI(TAG_ISR, "max : %d,  time : %d", (int)iMax, (int)uiTimeBetweenInterrupts);  
            } else if (oread > iBuf) {
                iMin = iBuf;
                ESP_LOGI(TAG_ISR, "min : %d,  time : %d", (int)iMin, (int)uiTimeBetweenInterrupts);  
            }

            iBuf = oread;
        }
        */

        //ESP_LOGI(TAG_ISR, "oread : %d < iAverageBuf : %d,  time : %d", (int)oread, (int)iAverageBuf, (int)uiTimeBetweenInterrupts);                     

/*        
        if (oread < 1000) {

            // measure gab
            uiCurrentTime = esp_timer_get_time();

            // calc average
            iSum = 0;
            for (iCnt = 0; iCnt < 10; iCnt++) {
                if (iCnt == 0)      iBufIN[iCnt] = oread;
                else                iBufIN[iCnt] = iBufOUT[iCnt - 1];

                iSum += iBufIN[iCnt];
            }
            iAverageNow = iSum / 10;

            if (iAverageNow > iAverageBuf) {
                iSetBig = 1;
                //ESP_LOGI(TAG_ISR, "oread : %d > iAverageBuf : %d,  time : %d", (int)oread, (int)iAverageBuf, (int)uiTimeBetweenInterrupts);
                uiTimeBetweenInterrupts = uiCurrentTime - uiLastInterruptTime;
                uiLastInterruptTime = uiCurrentTime;
            }

            else if (iAverageNow < iAverageBuf) {
                //uiTimeBetweenInterrupts = uiCurrentTime - uiLastInterruptTime;
                if (iSetBig == 1) {
                    uiTimeBetweenInterrupts = uiCurrentTime - uiLastInterruptTime;
                    uiLastInterruptTime = uiCurrentTime;

                    ESP_LOGI(TAG_ISR, "oread : %d < iAverageBuf : %d,  time : %d", (int)oread, (int)iAverageBuf, (int)uiTimeBetweenInterrupts);

                    if (uiTimeBetweenInterrupts > 20000) {
                        
                        xQueueSend(queue_result, bResult, portMAX_DELAY);
                        //ESP_LOGI(TAG_ISR, "oread : %d < iAverageBuf : %d,  time : %d", (int)oread, (int)iAverageBuf, (int)uiTimeBetweenInterrupts);
                        memset(bResult, 0, sizeof(bResult));
                        iCnt = 0;

                    } else {

                        if     ((105 < uiTimeBetweenInterrupts && uiTimeBetweenInterrupts < 130) && (iCnt>10)) {
                            bResult[iCnt] = true;
                            iCnt++;
                            ESP_LOGI(TAG_ISR, "iAverageNow : %d < iAverageBuf : %d,  time : %d", (int)iAverageNow, (int)iAverageBuf, (int)uiTimeBetweenInterrupts);
                        }
                        else if ((210 < uiTimeBetweenInterrupts && uiTimeBetweenInterrupts < 240) && (iCnt>10)) {
                            bResult[iCnt] = false;
                            iCnt++;
                            ESP_LOGI(TAG_ISR, "iAverageNow : %d < iAverageBuf : %d,  time : %d", (int)iAverageNow, (int)iAverageBuf, (int)uiTimeBetweenInterrupts);
                        }
                        iSetBig = 0;                        
                    }
                }
            }

            // esp_task_wdt_reset();

            for (iCnt = 0; iCnt < 10; iCnt++) {
                iBufOUT[iCnt] = iBufIN[iCnt];
            }

            iAverageBuf = iAverageNow;

        } else {

            iAverageNow = 0;
        }

     //vTaskDelay(10/portTICK_PERIOD_MS);
     esp_rom_delay_us(10);

    }

}
*/
void task_show_result(void* pvParameters)
{
    bool bA[100];

    while (1) {
        // Try to receive array from queue
        if (queue_result && xQueueReceive(queue_result, bA, 0) == pdPASS) {
            ESP_LOGI(TAG_ISR, ": [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]", bA[0],  bA[1], bA[2], bA[3], bA[4], bA[5], bA[6], bA[7], bA[8], bA[9]);
        }
    }
}


bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {

    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(handle_task_adc, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;;
}
////////////////////////////////////////////////////////////////////////////////////////////
// I2C
////////////////////////////////////////////////////////////////////////////////////////////
void task_i2c_handler(void* pvParameters)
{
    uint16_t buffer[BUF_LEN];
    size_t bytes_read = 0;

    while (1) {
        int64_t start = esp_timer_get_time();
        i2s_std_read(I2S_NUM, (void*)buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
        int64_t end = esp_timer_get_time();

        float samples = bytes_read / 2.0; // 16-bit samples
        float time_us = end - start;
        float actual_rate = samples / (time_us / 1e6);

        ESP_LOGI(TAG_ISR, "Read %d samples in %.0f us (%.2f samples/sec)", (int)samples, time_us, actual_rate);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Never reached, but for completeness:
    i2s_adc_disable(I2S_NUM);
    i2s_driver_uninstall(I2S_NUM);
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

    //===============================================================
    //======== oneshot ==============================================
    /*
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
        task_timer_handler, 
        "task_timer_handler", 
        2048, 
        NULL, 
        5, 
        &handle_task_adc, 
        0
    );
    */
    //===============================================================
    //======== i2c===================================================
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_12);

    // Configure I2S for ADC
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL_6);

    // Enable ADC
    i2s_adc_enable(I2S_NUM);



    //===============================================================
    //======== timer ================================================

    gptimer_handle_t gptimer = NULL;
    // Configure timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 100, // 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // set alarm of timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 10, // 1 sec    
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    
    gptimer_enable(gptimer);
    gptimer_start(gptimer);



    //ESP_ERROR_CHECK(esp_task_wdt_deinit());
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 500,
        .idle_core_mask = (1<<0),
        .trigger_panic = false,
    };
    //ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));    

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    //xTaskCreatePinnedToCore(ISR_1_Einfahrt_handler, "ISR_1_Einfahrt_handler",   2048, NULL, 5, NULL, 0);
    //xTaskCreatePinnedToCore(ISR1_feedback,          "ISR1_feedback",  2048, NULL, 5, NULL, 0);

    
    //xTaskCreatePinnedToCore(ISR1_Timer_feedback, "ISR1_Timer_feedback", 2048, NULL, 5, NULL, 0);

    iCount = 0;

    queue_result = xQueueCreate(100, sizeof(bool) * 100);

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
