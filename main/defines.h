#ifndef DEFINES_H
#define DEFINES_H

#define GPIO02_PIN19_O_MAIN_BLINKER 19  // Blinker Haupteinfahrt
#define GPIO04_PIN20_O_MAIN_WEICHE  20  // Weiche Haupteinfahrt
#define GPIO05_PIN23_O_SUB1_BLINKER 23  // Blinker Sub 1
#define GPIO18_PIN24_O_SUB1_WEICHE  24  // Weiche Sub 1
#define GPIO16_PIN27_O_SUB1_EFFEKT  27  // Haltestelle 1 Blinker aus/an, stop/start
#define GPIO21_PIN33_O_SUB2_EFFEKT  33  // Haltestelle 2 Blinker aus/an, stop/start
#define GPIO25_PIN09_O_SUB1_STOP     9  // Stopp Sub 1 Ausfahrt Hauptstrecke
#define GPIO27_PIN11_O_SUB2_STOP    11  // Stopp Sub 2 Ausfahrt Hauptstrecke
#define GPIO32_PIN07_O_MAIN_BLINKER  7  // Hauptstrecke Blinker aus

#define GPIO03_PIN34_I_MAIN_ISR_1   34  // ISR_1 Erkennung Haupteinfahrt 
#define GPIO12_PIN13_I_IN_ISR_2     13  // ISR_2 Erkennung Einfahrt
#define GPIO19_PIN25_I_SUB1_ISR_3   25  // ISR_3 Erkennung Sub 1
#define GPIO35_PIN05_I_SUB2_ISR_9    5  // ISR_9 Erkennung Sub 2
#define GPIO17_PIN22_I_HALT1_ISR_4  22  // ISR_4 Haltestelle 1
#define GPIO34_PIN04_I_HALT2_ISR_5   4  // ISR_5 Haltestelle 2
#define GPIO22_PIN29_I_MAIN1_ISR6   29  // ISR_6 Hauptstrecke Belegtmelder Sub1 
#define GPIO26_PIN09_I_MAIN2_ISR7    9  // ISR_7 Hauptstrecke Belegtmelder Sub2
#define GPIO33_PIN07_I_BUTTON_ISR8   7  // ISR_8 Taste Bus start

#define GPIO04_PIN20_O_INTERRUPT    20  // Ethernet Anbindung
#define GPIO15_PIN18_O_SPI_CS       18  // Ethernet Anbindung
#define GPIO12_PIN12_I_SPI_MISO     12  // Ethernet Anbindung
#define GPIO13_PIN13_O_SPI_MOSI     13  // Ethernet Anbindung
#define GPIO14_PIN11_O_SPI_SCK      11  // Ethernet Anbindung


//| GPIO   | DM9051      |
//| ------ | ----------- |
//| GPIO14 | SPI_CLK     | 11
//| GPIO13 | SPI_MOSI    | 13
//| GPIO12 | SPI_MISO    | 12
//| GPIO15 | SPI_CS      | 18
//| GPIO4  | Interrupt   | 20
//| NC     | Reset       |

#define i_SW_R                      0   // rechts
#define i_SW_L                      1   // links

#define ISR_on                      1   // Interrupt enable
#define ISR_off                     0   // Interrupt disable

#endif