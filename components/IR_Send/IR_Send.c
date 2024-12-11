#include "defines.h"
#include "globals.h"

#include "IR_Send.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void IR_Blinker_rechts_ein(void) {

    unsigned char szPreamble_0 = 0b11111111; 
    unsigned char szPreamble_1 = 0b11111111; 
    unsigned char szStartBit   = 0b00000000; 
    unsigned char szAddress    = 0b00000000; 
    unsigned char szCommand    = 0b01000001; 
    unsigned char szXOR        = 0b00000000; 
    unsigned char szStopBit    = 0b10000000; 
    unsigned char szBuffer     = 0b00000000; 

    int iByteCount       = 0;
    int iState           = 0;
    int iLength          = 0;
    int iFor             = 0;
    uint8_t iBit, uiBuf;

    struct telegram
    {
	    unsigned char uByte;
	    unsigned int  uAttribute;
    };    

    struct telegram s[9];

    szXOR = szAddress ^ szCommand;
	
	s[0].uAttribute = szPreamble_0;          s[0].uByte = 7;
	s[1].uAttribute = szPreamble_1;          s[1].uByte = 7;	
	s[2].uAttribute = szStartBit;            s[2].uByte = 0;
	s[3].uAttribute = szAddress;             s[3].uByte = 7;
	s[4].uAttribute = szStartBit;            s[4].uByte = 0;
	s[5].uAttribute = szCommand;             s[5].uByte = 7;
	s[6].uAttribute = szStartBit;            s[6].uByte = 0;
	s[7].uAttribute = szXOR;                 s[7].uByte = 7;
	s[8].uAttribute = szPreamble_0;          s[8].uByte = 0;    

	for (iFor = 0;  iFor < 5; iFor++)
	{
	    ets_delay_us(10000);
		for (iByteCount = 0; iByteCount < 9; iByteCount++) {
			for (iBit = 0; iBit <= s[iByteCount].uByte; iBit++) {
				uiBuf = (s[iByteCount].uAttribute & (1 << iBit));
    			if (uiBuf == 0)
					iLength = 116;
				else 
					iLength = 58;

				// perform here on signal high/low
				for (iState = 0; iState <= 1; iState++) {
					switch (iState) {
						case 0:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 0);									
						    break;
						case 1:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 1);
							break;
						}
							
						if (iLength == 58) 
							ets_delay_us(58);
						else if (iLength == 116) 
	                        ets_delay_us(116);									}		
																		
					}
                }
	}
}

void IR_Blinker_links_ein(void) {

    unsigned char szPreamble_0 = 0b11111111; 
    unsigned char szPreamble_1 = 0b11111111; 
    unsigned char szStartBit   = 0b00000000; 
    unsigned char szAddress    = 0b00000000; 
    unsigned char szCommand    = 0b01000010; 
    unsigned char szXOR        = 0b00000000; 
    unsigned char szStopBit    = 0b10000000; 
    unsigned char szBuffer     = 0b00000000; 

    int iByteCount       = 0;
    int iState           = 0;
    int iLength          = 0;
    int iFor             = 0;
    uint8_t iBit, uiBuf;

    struct telegram
    {
	    unsigned char uByte;
	    unsigned int  uAttribute;
    };    

    struct telegram s[9];

    szXOR = szAddress ^ szCommand;
	
	s[0].uAttribute = szPreamble_0;          s[0].uByte = 7;
	s[1].uAttribute = szPreamble_1;          s[1].uByte = 7;	
	s[2].uAttribute = szStartBit;            s[2].uByte = 0;
	s[3].uAttribute = szAddress;             s[3].uByte = 7;
	s[4].uAttribute = szStartBit;            s[4].uByte = 0;
	s[5].uAttribute = szCommand;             s[5].uByte = 7;
	s[6].uAttribute = szStartBit;            s[6].uByte = 0;
	s[7].uAttribute = szXOR;                 s[7].uByte = 7;
	s[8].uAttribute = szPreamble_0;          s[8].uByte = 0;    

	for (iFor = 0;  iFor < 5; iFor++)
	{
	    ets_delay_us(10000);
		for (iByteCount = 0; iByteCount < 9; iByteCount++) {
			for (iBit = 0; iBit <= s[iByteCount].uByte; iBit++) {
				uiBuf = (s[iByteCount].uAttribute & (1 << iBit));
    			if (uiBuf == 0)
					iLength = 116;
				else 
					iLength = 58;

				// perform here on signal high/low
				for (iState = 0; iState <= 1; iState++) {
					switch (iState) {
						case 0:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 0);									
						    break;
						case 1:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 1);
							break;
						}
							
						if (iLength == 58) 
							ets_delay_us(58);
						else if (iLength == 116) 
	                        ets_delay_us(116);									}		
																		
					}
                }
	}
}

void IR_Blinker_aus(void) {

    unsigned char szPreamble_0 = 0b11111111; 
    unsigned char szPreamble_1 = 0b11111111; 
    unsigned char szStartBit   = 0b00000000; 
    unsigned char szAddress    = 0b00000000; 
    unsigned char szCommand    = 0b01000001; 
    unsigned char szXOR        = 0b00000000; 
    unsigned char szStopBit    = 0b10000000; 
    unsigned char szBuffer     = 0b00000000; 

    int iByteCount       = 0;
    int iState           = 0;
    int iLength          = 0;
    int iFor             = 0;
    uint8_t iBit, uiBuf;

    struct telegram
    {
	    unsigned char uByte;
	    unsigned int  uAttribute;
    };    

    struct telegram s[9];

    szXOR = szAddress ^ szCommand;
	
	s[0].uAttribute = szPreamble_0;          s[0].uByte = 7;
	s[1].uAttribute = szPreamble_1;          s[1].uByte = 7;	
	s[2].uAttribute = szStartBit;            s[2].uByte = 0;
	s[3].uAttribute = szAddress;             s[3].uByte = 7;
	s[4].uAttribute = szStartBit;            s[4].uByte = 0;
	s[5].uAttribute = szCommand;             s[5].uByte = 7;
	s[6].uAttribute = szStartBit;            s[6].uByte = 0;
	s[7].uAttribute = szXOR;                 s[7].uByte = 7;
	s[8].uAttribute = szPreamble_0;          s[8].uByte = 0;    

	for (iFor = 0;  iFor < 5; iFor++)
	{
	    ets_delay_us(10000);
		for (iByteCount = 0; iByteCount < 9; iByteCount++) {
			for (iBit = 0; iBit <= s[iByteCount].uByte; iBit++) {
				uiBuf = (s[iByteCount].uAttribute & (1 << iBit));
    			if (uiBuf == 0)
					iLength = 116;
				else 
					iLength = 58;

				// perform here on signal high/low
				for (iState = 0; iState <= 1; iState++) {
					switch (iState) {
						case 0:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 0);									
						    break;
						case 1:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 1);
							break;
						}
							
						if (iLength == 58) 
							ets_delay_us(58);
						else if (iLength == 116) 
	                        ets_delay_us(116);									}		
																		
					}
                }
	}
}


void IR_Stop(void) {

    unsigned char szPreamble_0 = 0b11111111; 
    unsigned char szPreamble_1 = 0b11111111; 
    unsigned char szStartBit   = 0b00000000; 
    unsigned char szAddress    = 0b00000000; 
    unsigned char szCommand    = 0b01100000; 
    unsigned char szXOR        = 0b00000000; 
    unsigned char szStopBit    = 0b10000000; 
    unsigned char szBuffer     = 0b00000000; 

    int iByteCount       = 0;
    int iState           = 0;
    int iLength          = 0;
    int iFor             = 0;
    uint8_t iBit, uiBuf;

    struct telegram
    {
	    unsigned char uByte;
	    unsigned int  uAttribute;
    };    

    struct telegram s[9];

    szXOR = szAddress ^ szCommand;
	
	s[0].uAttribute = szPreamble_0;          s[0].uByte = 7;
	s[1].uAttribute = szPreamble_1;          s[1].uByte = 7;	
	s[2].uAttribute = szStartBit;            s[2].uByte = 0;
	s[3].uAttribute = szAddress;             s[3].uByte = 7;
	s[4].uAttribute = szStartBit;            s[4].uByte = 0;
	s[5].uAttribute = szCommand;             s[5].uByte = 7;
	s[6].uAttribute = szStartBit;            s[6].uByte = 0;
	s[7].uAttribute = szXOR;                 s[7].uByte = 7;
	s[8].uAttribute = szPreamble_0;          s[8].uByte = 0;    

	for (iFor = 0;  iFor < 5; iFor++)
	{
	    ets_delay_us(10000);
		for (iByteCount = 0; iByteCount < 9; iByteCount++) {
			for (iBit = 0; iBit <= s[iByteCount].uByte; iBit++) {
				uiBuf = (s[iByteCount].uAttribute & (1 << iBit));
    			if (uiBuf == 0)
					iLength = 116;
				else 
					iLength = 58;

				// perform here on signal high/low
				for (iState = 0; iState <= 1; iState++) {
					switch (iState) {
						case 0:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 0);									
						    break;
						case 1:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 1);
							break;
						}
							
						if (iLength == 58) 
							ets_delay_us(58);
						else if (iLength == 116) 
	                        ets_delay_us(116);									}		
																		
					}
                }
	}
}

void IR_Start(void) {

    unsigned char szPreamble_0 = 0b11111111; 
    unsigned char szPreamble_1 = 0b11111111; 
    unsigned char szStartBit   = 0b00000000; 
    unsigned char szAddress    = 0b00000000; 
    unsigned char szCommand    = 0b01101101; // Fahrstufe mus noch eingestellt werden
    unsigned char szXOR        = 0b00000000; 
    unsigned char szStopBit    = 0b10000000; 
    unsigned char szBuffer     = 0b00000000; 

    int iByteCount       = 0;
    int iState           = 0;
    int iLength          = 0;
    int iFor             = 0;
    uint8_t iBit, uiBuf;

    struct telegram
    {
	    unsigned char uByte;
	    unsigned int  uAttribute;
    };    

    struct telegram s[9];

    szXOR = szAddress ^ szCommand;
	
	s[0].uAttribute = szPreamble_0;          s[0].uByte = 7;
	s[1].uAttribute = szPreamble_1;          s[1].uByte = 7;	
	s[2].uAttribute = szStartBit;            s[2].uByte = 0;
	s[3].uAttribute = szAddress;             s[3].uByte = 7;
	s[4].uAttribute = szStartBit;            s[4].uByte = 0;
	s[5].uAttribute = szCommand;             s[5].uByte = 7;
	s[6].uAttribute = szStartBit;            s[6].uByte = 0;
	s[7].uAttribute = szXOR;                 s[7].uByte = 7;
	s[8].uAttribute = szPreamble_0;          s[8].uByte = 0;    

	for (iFor = 0;  iFor < 5; iFor++)
	{
	    ets_delay_us(10000);
		for (iByteCount = 0; iByteCount < 9; iByteCount++) {
			for (iBit = 0; iBit <= s[iByteCount].uByte; iBit++) {
				uiBuf = (s[iByteCount].uAttribute & (1 << iBit));
    			if (uiBuf == 0)
					iLength = 116;
				else 
					iLength = 58;

				// perform here on signal high/low
				for (iState = 0; iState <= 1; iState++) {
					switch (iState) {
						case 0:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 0);									
						    break;
						case 1:
							gpio_set_level(GPIO02_O_MAIN_BLINKER, 1);
							break;
						}
							
						if (iLength == 58) 
							ets_delay_us(58);
						else if (iLength == 116) 
	                        ets_delay_us(116);									}		
																		
					}
                }
	}
}

