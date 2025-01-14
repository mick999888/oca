#ifndef DEFINES_H
#define DEFINES_H


// output
#define GPIO02_O_MAIN_BLINKER 19  // Blinker Haupteinfahrt
#define GPIO04_O_MAIN_WEICHE  20  // Weiche Haupteinfahrt
#define GPIO05_O_SUB1_BLINKER 23  // Blinker Sub 1
#define GPIO18_O_SUB1_WEICHE  24  // Weiche Sub 1
#define GPIO16_O_SUB1_EFFEKT  27  // Haltestelle 1 Blinker aus/an, stop/start
#define GPIO21_O_SUB2_EFFEKT  33  // Haltestelle 2 Blinker aus/an, stop/start
#define GPIO25_O_SUB1_STOP     9  // Stopp Sub 1 Ausfahrt Hauptstrecke
#define GPIO27_O_SUB2_STOP    11  // Stopp Sub 2 Ausfahrt Hauptstrecke
#define GPIO32_O_MAIN_BLINKER  7  // Hauptstrecke Blinker aus

// input
#define GPIO03_I_MAIN_ISR_1   34  // ISR_1 Erkennung Haupteinfahrt 
#define GPIO12_I_IN_ISR_2     13  // ISR_2 Erkennung Einfahrt
#define GPIO19_I_SUB1_ISR_3   25  // ISR_3 Erkennung Sub 1
#define GPIO35_I_SUB2_ISR_9    5  // ISR_9 Erkennung Sub 2
#define GPIO17_I_HALT1_ISR_4  22  // ISR_4 Haltestelle 1
#define GPIO34_I_HALT2_ISR_5   4  // ISR_5 Haltestelle 2
#define GPIO22_I_MAIN1_ISR6   29  // ISR_6 Hauptstrecke Belegtmelder Sub1 
#define GPIO36_I_MAIN2_ISR7    9  // ISR_7 Hauptstrecke Belegtmelder Sub2
#define GPIO39_I_BUTTON_ISR8   7  // ISR_8 Taste Bus start

// ethernet
#define GPIO04_O_INTERRUPT    20  // Ethernet Anbindung
#define GPIO15_O_SPI_CS       18  // Ethernet Anbindung
#define GPIO12_I_SPI_MISO     12  // Ethernet Anbindung
#define GPIO13_O_SPI_MOSI     13  // Ethernet Anbindung
#define GPIO14_O_SPI_SCK      11  // Ethernet Anbindung

// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

//ESP32	                                w5500
//D18 / GPIO15                          CS
//D11 / GPIO14                          SCK
//D12 / GPIO12                          MISO
//D13 / GPIO 	                                MOSI
//3.3v                                  (better with external 200mha)	VCC
//GND	                                GND

// 
// GPIO00
// GPIO01
// GPIO20
// GPIO23
// GPIO26  
// GPIO33  

// SPI blocked 
// GPIO06 xx
// GPIO07 xx
// GPIO08 xx
// GPIO09 xx
// GPIO10 xx
// GPIO11 xx

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


// GPIO  PIN	Input	    Output	Notes
// 0	  25    pulled up	OK	    outputs PWM signal at boot, must be LOW to enter flashing mode
// 1	        TX pin	    OK	    debug output at boot
// 2	    OK	OK	connected to on-board LED, must be left floating or LOW to enter flashing mode
// 3	    OK	RX pin	HIGH at boot
// 4	    OK	OK	
// 5	    OK	OK	outputs PWM signal at boot, strapping pin
// 6	    x	x	connected to the integrated SPI flash
// 7	    x	x	connected to the integrated SPI flash
// 8	    x	x	connected to the integrated SPI flash
// 9	    x	x	connected to the integrated SPI flash
//10	    x	x	connected to the integrated SPI flash
//11	    x	x	connected to the integrated SPI flash
//12	    OK	OK	boot fails if pulled high, strapping pin
//13	    OK	OK	
//14	    OK	OK	outputs PWM signal at boot
//15	    OK	OK	outputs PWM signal at boot, strapping pin
//16	    OK	OK	
//17	    OK	OK	
//18	    OK	OK	
//19	    OK	OK	
//21	    OK	OK	
//22	    OK	OK	
//23	    OK	OK	
//25	    OK	OK	
//26	    OK	OK	
//27	    OK	OK	
//32	    OK	OK	
//33	    OK	OK	
//34	    OK		input only
//35	    OK		input only
//36	    OK		input only
//39	    OK		input only