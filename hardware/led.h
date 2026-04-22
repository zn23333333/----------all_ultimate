#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"

/* Heartbeat LED on PG14 */
#define LED1_PIN             GPIO_Pin_14
#define LED1_PORT            GPIOG
#define LED1_CLK             RCC_AHB1Periph_GPIOG

#define LED1_ON              GPIO_ResetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF             GPIO_SetBits(LED1_PORT, LED1_PIN)
#define LED1_Toggle          GPIO_ToggleBits(LED1_PORT, LED1_PIN)

void LED_Init(void);

#endif /* __LED_H */
