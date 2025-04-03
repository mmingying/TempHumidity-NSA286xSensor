#include "relay.h"

void Relay_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
	
    /* ??GPIO?? */
    RCC_APB2PeriphClockCmd(RELAY_IN1_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RELAY_IN2_GPIO_CLK, ENABLE);
    /* ???GPIO */
    gpio_initstruct.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_initstruct.GPIO_Pin = RELAY_IN1_GPIO_PIN;
    gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(RELAY_IN1_GPIO_PORT, &gpio_initstruct);
	
	 

 
    gpio_initstruct.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_initstruct.GPIO_Pin = RELAY_IN2_GPIO_PIN;
    gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(RELAY_IN2_GPIO_PORT, &gpio_initstruct);
}
