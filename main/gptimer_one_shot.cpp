#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "GPTIMER_EXAMPLE";
static TaskHandle_t s_task_handle = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static QueueHandle_t s_array_queue = NULL;

static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void sender_task(void *arg) {
    int arr[4] = {1, 2, 3, 4};
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000)); // Send every 3 seconds
        if (s_array_queue) {
            xQueueSend(s_array_queue, arr, portMAX_DELAY);
            ESP_LOGI(TAG, "Sender: Array sent to queue");
        }
    }
}

void timer_task(void *arg) {
    int adc_raw = 0;
    int arr[4];
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Read ADC value on notification
        if (adc_handle) {
            adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &adc_raw); // GPIO34 (ADC1_CH6) as example
            ESP_LOGI(TAG, "Timer interrupt received! ADC Value: %d", adc_raw);
        } else {
            ESP_LOGI(TAG, "Timer interrupt received! ADC not initialized");
        }
        // Try to receive array from queue
        if (s_array_queue && xQueueReceive(s_array_queue, arr, 0) == pdPASS) {
            ESP_LOGI(TAG, "Timer task: Received array: [%d, %d, %d, %d]", arr[0], arr[1], arr[2], arr[3]);
        }
    }
}

void app_main(void) {
    // ADC one-shot config
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&adc_init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t adc_chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &adc_chan_cfg); // GPIO34 (ADC1_CH6)

    xTaskCreate(timer_task, "timer_task", 2048, NULL, 5, &s_task_handle);
    xTaskCreate(sender_task, "sender_task", 2048, NULL, 5, NULL);

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz = 1us per tick
    };
    gptimer_new_timer(&timer_config, &gptimer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000, // 1 second
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);

    gptimer_enable(gptimer);
    gptimer_start(gptimer);

    // Create queue for 4-int array
    s_array_queue = xQueueCreate(4, sizeof(int) * 4);
}