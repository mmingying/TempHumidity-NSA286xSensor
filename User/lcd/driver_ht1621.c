	#include "driver_ht1621.h"
#include "FreeRTOS.h"
#include "task.h"

static void HT1621_GPIO_Config(void);


void HT1621_Init(void)
{
    HT1621_GPIO_Config();
    
    vTaskDelay(10);
    
    LCD_CS_1();
    LCD_DATA_1();
    LCD_WR_1();
    
    vTaskDelay(100);                   
    
    HT1621_WriteCommand(HT1621_SYS_EN);
    HT1621_WriteCommand(HT1621_BIAS);  
    HT1621_WriteCommand(HT1621_RC256);  
    HT1621_WriteCommand(HT1621_WDT_DIS);
    HT1621_WriteCommand(HT1621_LCD_ON);
}


void HT1621_WriteCommand(uint8_t cmd)
{
    uint8_t i;
    
    LCD_CS_0();                         // CS = 0 
    vTaskDelay(1);
    
   
    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_1();                       // DATA = 1
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_0();                       // DATA = 0
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_0();                       // DATA = 0
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    

    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_0();                       // DATA = 0
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
  
    for(i = 0; i < 8; i++)
    {
        LCD_WR_0();                     // WR = 0
        vTaskDelay(1);
        if((cmd << i) & 0x80)
        {
            LCD_DATA_1();               // DATA = 1
        }
        else
        {
            LCD_DATA_0();               // DATA = 0
        }
        vTaskDelay(1);
        LCD_WR_1();                     // WR = 1
        vTaskDelay(1);
    }
    
    LCD_CS_1();                         // CS = 1
    vTaskDelay(1);
}



void HT1621_WriteData4Bit(uint8_t addr, uint8_t data)
{
    uint8_t i;
    
    LCD_CS_0();                         // CS = 0 
    vTaskDelay(1);

    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_1();                       // DATA = 1
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_0();                       // DATA = 0
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_1();                       // DATA = 1
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
		
    addr <<= 2;

    for(i = 0; i < 6; i++)
    {
        LCD_WR_0();                     // WR = 0
        vTaskDelay(1);
        if((addr << i) & 0x80)
        {
            LCD_DATA_1();               // DATA = 1
        }
        else
        {
            LCD_DATA_0();               // DATA = 0
        }
        vTaskDelay(1);
        LCD_WR_1();                     // WR = 1
        vTaskDelay(1);
    }

    for(i = 0; i < 4; i++)
    {
        LCD_WR_0();                     // WR = 0
        vTaskDelay(1);
        if((data >> i) & 0x01)
        {
            LCD_DATA_1();               // DATA = 1
        }
        else
        {
            LCD_DATA_0();               // DATA = 0
        }
        vTaskDelay(1);
        LCD_WR_1();                     // WR = 1
        vTaskDelay(1);
    }
    
    LCD_CS_1();                         // CS = 1
    vTaskDelay(1);    
}

void HT1621_WriteData8Bits(uint8_t addr, uint8_t data)
{
	uint8_t dataH = (0xF0 & data) >> 4;
	HT1621_WriteData4Bit(addr++, dataH);
	HT1621_WriteData4Bit(addr++, data);
}


void HT1621_WriteData8Bit(uint8_t addr, uint8_t data)
{
    uint8_t i;
    
    LCD_CS_0();                         // CS = 0 
    vTaskDelay(1);
    

    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_1();                       // DATA = 1
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_0();                       // DATA = 0
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
    LCD_WR_0();                         // WR = 0
    vTaskDelay(1);
    LCD_DATA_1();                       // DATA = 1
    vTaskDelay(1);
    LCD_WR_1();                         // WR = 1
    vTaskDelay(1);
    
		addr <<= 2;
  
    for(i = 0; i < 6; i++)
    {
        LCD_WR_0();                     // WR = 0
        vTaskDelay(1);
        if((addr << i) & 0x80)
        {
            LCD_DATA_1();               // DATA = 1
        }
        else
        {
            LCD_DATA_0();               // DATA = 0
        }
        vTaskDelay(1);
        LCD_WR_1();                     // WR = 1
        vTaskDelay(1);
    }
  
    for(i = 0; i < 8; i++)
    {
        LCD_WR_0();                     // WR = 0
        vTaskDelay(1);
        if((data >> i) & 0x01)
        {
            LCD_DATA_1();               // DATA = 1
        }
        else
        {
            LCD_DATA_0();               // DATA = 0
        }
        vTaskDelay(1);
        LCD_WR_1();                     // WR = 1
        vTaskDelay(1);
    }
    
    LCD_CS_1();                         // CS = 1
    vTaskDelay(1);       
}




void HT1621_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����LED��ص�GPIO����ʱ��*/
		RCC_APB2PeriphClockCmd( HT1621_GPIO_CS_CLK |HT1621_GPIO_WR_CLK | HT1621_GPIO_DATA_CLK, ENABLE);
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = HT1621_GPIO_CS_PIN;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(HT1621_GPIO_CS_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = HT1621_GPIO_WR_PIN;

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(HT1621_GPIO_WR_PORT, &GPIO_InitStructure);
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = HT1621_GPIO_DATA_PIN;

		/*���ÿ⺯������ʼ��GPIOF*/
		GPIO_Init(HT1621_GPIO_DATA_PORT, &GPIO_InitStructure);

}

