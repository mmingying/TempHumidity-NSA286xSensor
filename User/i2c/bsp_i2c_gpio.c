/**
  ******************************************************************************
  * @file    bsp_i2c_ee.c
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ��gpioģ��i2c����, ������STM32ϵ��CPU����ģ�鲻����Ӧ�ò�����֡��������I2C���߻�������������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-ָ���� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

/*
	Ӧ��˵����
	�ڷ���I2C�豸ǰ�����ȵ��� i2c_CheckDevice() ���I2C�豸�Ƿ��������ú���������GPIO

*/
#include "bsp_i2c_gpio.h"
#include "stm32f10x.h"


static void i2c_Delay(void)
{
    uint8_t i;
    for (i = 0; i < 10; i++);  
}


void i2c_Init(I2C_PinConfig_t *pinConfig)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


    GPIO_InitStructure.GPIO_Pin = pinConfig->SCL_Pin | pinConfig->SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
    GPIO_Init(pinConfig->GPIO_Port, &GPIO_InitStructure);


    i2c_Stop(pinConfig);
}


void i2c_Start(I2C_PinConfig_t *pinConfig)
{
    GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);
    GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
    i2c_Delay();
    GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);
    i2c_Delay();
    GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
    i2c_Delay();
}


void i2c_Stop(I2C_PinConfig_t *pinConfig)
{
    GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);
    GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
    i2c_Delay();
    GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);
    i2c_Delay();
}


void i2c_SendByte(I2C_PinConfig_t *pinConfig, uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);
        }
        else
        {
            GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);
        }
        i2c_Delay();
        GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
        i2c_Delay();
        GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
        data <<= 1;
    }
}


uint8_t i2c_ReadByte(I2C_PinConfig_t *pinConfig)
{
    uint8_t i, value = 0;
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
        i2c_Delay();
        if (GPIO_ReadInputDataBit(pinConfig->GPIO_Port, pinConfig->SDA_Pin))
        {
            value++;
        }
        GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
        i2c_Delay();
    }
    return value;
}


uint8_t i2c_WaitAck(I2C_PinConfig_t *pinConfig)
{
    uint8_t re;
    GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);
    i2c_Delay();
    GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
    i2c_Delay();
    if (GPIO_ReadInputDataBit(pinConfig->GPIO_Port, pinConfig->SDA_Pin))
    {
        re = 1;  
    }
    else
    {
        re = 0;  
    }
    GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
    i2c_Delay();
    return re;
}

void i2c_Ack(I2C_PinConfig_t *pinConfig)
{
	GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);	/* CPU����SDA = 0 */
	i2c_Delay();
	GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);	/* CPU����1��ʱ�� */
	i2c_Delay();
	GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
	i2c_Delay();
	GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);	/* CPU�ͷ�SDA���� */
}

void i2c_NAck(I2C_PinConfig_t *pinConfig)
{
	GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SDA_Pin);/* CPU����SDA = 1 */
	i2c_Delay();
	GPIO_SetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);	/* CPU����1��ʱ�� */
	i2c_Delay();
	GPIO_ResetBits(pinConfig->GPIO_Port, pinConfig->SCL_Pin);
	i2c_Delay();	
}

uint8_t i2c_CheckDevice(I2C_PinConfig_t *pinConfig, uint8_t _Address)
{
    uint8_t ucAck;

   

    i2c_Start(pinConfig);  

 
    i2c_SendByte(pinConfig, _Address | EEPROM_I2C_WR);
    ucAck = i2c_WaitAck(pinConfig);  

    i2c_Stop(pinConfig);  

    return ucAck;
}

uint8_t ee_CheckOk(I2C_PinConfig_t *pinConfig, uint8_t address)
{
    if (i2c_CheckDevice(pinConfig, address) == 0)
    {
        return 1;
    }
    else
    {
     
        i2c_Stop(pinConfig);        
        return 0;
    }
}

void GP8212S_write(I2C_PinConfig_t *pinConfig, uint8_t devAddr, uint16_t DAC_data)
{
    i2c_Start(pinConfig);
    i2c_SendByte(pinConfig, devAddr); 
    i2c_WaitAck(pinConfig);
    i2c_SendByte(pinConfig, 0x02);  
    i2c_WaitAck(pinConfig);
    i2c_SendByte(pinConfig, DAC_data & 0xFF); 
    i2c_WaitAck(pinConfig);
    i2c_SendByte(pinConfig, (DAC_data >> 8) & 0xFF);  
    i2c_WaitAck(pinConfig);
    i2c_Stop(pinConfig);
}
