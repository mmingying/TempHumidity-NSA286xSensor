#ifndef __DRIVER_HT1621_H
#define __DRIVER_HT1621_H

#include "stm32f10x.h"

// LCD
#define HT1621_GPIO_CS_PORT    	GPIOB			             
#define HT1621_GPIO_CS_CLK 	    RCC_APB2Periph_GPIOB	
#define HT1621_GPIO_CS_PIN		GPIO_Pin_8			   

#define HT1621_GPIO_WR_PORT    	GPIOB			             
#define HT1621_GPIO_WR_CLK 	    RCC_APB2Periph_GPIOB	
#define HT1621_GPIO_WR_PIN		GPIO_Pin_7

#define HT1621_GPIO_DATA_PORT    	GPIOB			             
#define HT1621_GPIO_DATA_CLK 	    RCC_APB2Periph_GPIOB	
#define HT1621_GPIO_DATA_PIN		GPIO_Pin_6		



#define LCD_CS_0()                  GPIO_ResetBits(HT1621_GPIO_CS_PORT,HT1621_GPIO_CS_PIN) 
#define LCD_CS_1()                  GPIO_SetBits(HT1621_GPIO_CS_PORT,HT1621_GPIO_CS_PIN) 
#define LCD_DATA_0()                GPIO_ResetBits(HT1621_GPIO_DATA_PORT,HT1621_GPIO_DATA_PIN) 
#define LCD_DATA_1()                GPIO_SetBits(HT1621_GPIO_DATA_PORT,HT1621_GPIO_DATA_PIN)
#define LCD_WR_0()                  GPIO_ResetBits(HT1621_GPIO_WR_PORT,HT1621_GPIO_WR_PIN) 
#define LCD_WR_1()                  GPIO_SetBits(HT1621_GPIO_WR_PORT,HT1621_GPIO_WR_PIN)

#define COMMAND_CODE                0x80                    // ???
#define WRITE_DATA_CODE             0xA0                    // ???

#define HT1621_BIAS                 0x52                    // 1/3duty 4com
#define HT1621_SYS_DIS              0x00                    // ???????LCD?????
#define HT1621_SYS_EN               0x02                    // ???????
#define HT1621_LCD_OFF              0x04                    // ??LCD??
#define HT1621_LCD_ON               0x06                    // ??LCE??
#define HT1621_XTAL                 0x28                    // ?????
#define HT1621_RC256                0x30                    // ????
#define HT1621_WDT_DIS              0X0A                    // ?????
#define HT1621_TONE_ON              0x12                    // ??????
#define HT1621_TONE_OFF             0x10                    // ??????


void HT1621_Init(void);
void HT1621_WriteCommand(uint8_t cmd);
void HT1621_WriteData4Bit(uint8_t addr, uint8_t data);
void HT1621_WriteData8Bit(uint8_t addr, uint8_t data);
void HT1621_WriteData8Bits(uint8_t addr, uint8_t data);

#endif /* USER_INCLUDE_DRIVER_VKL144_H_ */
