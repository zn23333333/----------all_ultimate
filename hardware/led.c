#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_AHB1PeriphClockCmd(LED1_CLK, ENABLE);

    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    gpio_init.GPIO_Pin = LED1_PIN;
    GPIO_Init(LED1_PORT, &gpio_init);

    GPIO_ResetBits(LED1_PORT, LED1_PIN);
}
