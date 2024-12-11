#include "defines.h"
#include "globals.h"
#include "SW_Function.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// #define GPIO02_PIN24_O_MAIN_BLINKER 24  // Blinker Haupteinfahrt
// #define GPIO04_PIN26_O_MAIN_WEICHE  26  // Weiche Haupteinfahrt
// #define GPIO13_PIN15_O_SUB1_BLINKER 15  // Blinker Sub 1
// #define GPIO14_PIN12_O_SUB1_WEICHE  12  // Weiche Sub 1
// #define GPIO16_PIN27_O_SUB1_EFFEKT  27  // Haltestelle 1 Blinker aus/an, stop/start
// #define GPIO21_PIN33_O_SUB2_EFFEKT  33  // Haltestelle 2 Blinker aus/an, stop/start
// #define GPIO25_PIN09_O_SUB1_STOP     9  // Stopp Sub 1 Ausfahrt Hauptstrecke
// #define GPIO27_PIN11_O_SUB2_STOP    11  // Stopp Sub 2 Ausfahrt Hauptstrecke
// #define GPIO32_PIN07_O_MAIN_BLINKER  7  // Hauptstrecke Blinker aus

void SW_Main (int iDir) {
    gpio_set_level(GPIO04_O_MAIN_WEICHE, iDir);
}

void SW_Sub1 (int iDir) {
    gpio_set_level(GPIO18_O_SUB1_WEICHE, iDir);
}

