#ifndef __RELAY_H
#define __RELAY_H

#include "stm32f10x.h"


#define RELAY_IN1_GPIO_CLK_ENABLE   RCC_APB2PeriphClockCmd
#define RELAY_IN1_GPIO_CLK          RCC_APB2Periph_GPIOB
#define RELAY_IN1_GPIO_PORT         GPIOB
#define RELAY_IN1_GPIO_PIN          GPIO_Pin_12

#define RELAY_IN2_GPIO_CLK_ENABLE   RCC_APB2PeriphClockCmd
#define RELAY_IN2_GPIO_CLK          RCC_APB2Periph_GPIOB
#define RELAY_IN2_GPIO_PORT         GPIOB
#define RELAY_IN2_GPIO_PIN          GPIO_Pin_13

#define RELAY1(a)                    GPIO_WriteBit(RELAY_IN1_GPIO_PORT,RELAY_IN1_GPIO_PIN,a)
#define RELAY2(a)                    GPIO_WriteBit(RELAY_IN2_GPIO_PORT,RELAY_IN2_GPIO_PIN,a)

void Relay_GPIO_Init(void);




#endif // !__RELAY_H
