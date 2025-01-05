#include "defines.h"
#include "globals.h"
#include "SW_Function.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void SW_Main (int iDir) {
    gpio_set_level(GPIO04_O_MAIN_WEICHE, iDir);
}

void SW_Sub1 (int iDir) {
    gpio_set_level(GPIO18_O_SUB1_WEICHE, iDir);
}

