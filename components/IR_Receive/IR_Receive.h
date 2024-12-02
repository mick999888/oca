#ifndef IR_RECEIVE_H
#define IR_RECEIVE_H

// receive functions
void ISR_1_Einfahrt (void* arg);
void IRAM_ATTR ISR_2_IN (void* arg);
void IRAM_ATTR ISR_3_SUB1 (void* arg);
void IRAM_ATTR ISR_9_SUB2 (void* arg);
void IRAM_ATTR ISR_4_HALT1 (void* arg);
void IRAM_ATTR ISR_5_HALT2 (void* arg);
void IRAM_ATTR ISR_6_MAIN1 (void* arg);
void IRAM_ATTR ISR_7_MAIN2 (void* arg);
void IRAM_ATTR ISR_8_BUTTON (void* arg);

void IRAM_ATTR ISR_TIMER_SUB1(void* arg);
void IRAM_ATTR ISR_TIMER_SUB2(void* arg);

#endif
