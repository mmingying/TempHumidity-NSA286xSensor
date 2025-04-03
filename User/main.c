/**
  *********************************************************************
  * @file    ��ʪ��-sht20
  * @author  ChenMing
  * @version V1.0
  * @date    2024-12-24
  * @brief   FreeRTOS-sht20-��ʾ��-4-20mA���-modbus485-4G
  *********************************************************************
  * @attention
  *
  * ��˾��Ŧ������(�Ͼ�)���޹�˾
  * ��ϵ��ʽ��CYMJ17203730@163.com
  *
  *
  **********************************************************************
  */ 
	
/* FreeRTOSͷ�ļ� */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* ������Ӳ��bspͷ�ļ� */
#include "bsp_led.h"
#include "bsp_usart.h"
#include "mb.h"
#include "mbutils.h"
#include "bsp_i2c.h"
#include "SHT20.h"
#include "bsp_lcd.h"
#include "bsp_i2c_gpio.h"
#include "bsp_key.h"  
#include "relay.h"
#include "queue.h"
#include "bsp_iwdg.h" 
#include "driver_ht1621.h"
#include "stm32f10x_flash.h"  
#include "bsp_internal_flash.h"
#include <string.h>
#include "Modbus.h"
#include "bsp_nsa2862.h"

/*���С��ź�������*/
QueueHandle_t xTempQueue;
QueueHandle_t xHumQueue;
QueueHandle_t xFlashQueue;
SemaphoreHandle_t xFlashLoadedSemaphore;  

/*�ɼ�����ʪ�ȱ�������sht20.c�ж���*/
//extern float temperatureC;
//extern float humidityRH;
float presure;
uint32_t presure1;
/*I2C��SDA��SCL���Ŷ��壬����ʹ��I2C����ֱ����Ӳ�������*/
I2C_PinConfig_t I2C1_Config = {GPIOA, GPIO_Pin_2, GPIO_Pin_1}; 
I2C_PinConfig_t I2C2_Config = {GPIOA, GPIO_Pin_4, GPIO_Pin_3}; 
I2C_PinConfig_t I2C3_Config = {GPIOA, GPIO_Pin_12, GPIO_Pin_11};

/*������*/
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t Flash_Task_Handle = NULL;
static TaskHandle_t MODBUS_Task_Handle = NULL;
static TaskHandle_t SHT20_Task_Handle = NULL;
static TaskHandle_t HardWare_Task_Handle = NULL;

/*��غ�������*/
static void AppTaskCreate(void);
static void Flash_Task(void* pvParameters);
static void BSP_Init(void);
static void MODBUS_Task(void* pvParameters);
static void SHT20_Task(void* pvParameters);
static void HardWare_Task(void* pvParameters);

/*Flash������ʼ-������ַ*/
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800C000)    //��ʼ��ַ
#define FLASH_MAGIC_NUMBER    0x1234                      //�Զ������Ч���ݱ��
#define FLASH_MAGIC_ADDR      FLASH_USER_START_ADDR       //���ݱ����ʼ��ַ
#define FLASH_USER_DATA_ADDR  (FLASH_USER_START_ADDR + 4) //�Զ���������ʼ��ַ 

/*���ּĴ�����ʼֵ����*/
static uint16_t previousUsRegHoldingBuf[REG_HOLDING_NREGS];
static const uint16_t usRegHoldingBufInitial[REG_HOLDING_NREGS] = {
    0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1388,
    0x00, 0x2710, 0x01, 0x1450, 0x663A, 0x1478, 0x6680,
    0x00, 0x1388, 0x00, 0x2710, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};

/*I2C��ʼ������*/
void init_I2C_modules(void)
{
    i2c_Init(&I2C1_Config);  
    i2c_Init(&I2C2_Config);
    i2c_Init(&I2C3_Config);  
}
/*4-20mA�������*/
void set_4_20mA_output(uint16_t dac_value1, uint16_t dac_value2)
{
    GP8212S_write(&I2C1_Config, 0xB0, dac_value1);  
    GP8212S_write(&I2C2_Config, 0xB0, dac_value2);  
}
/*FLASH���뺯��*/
void SaveUsRegHoldingBufToFlash(uint16_t* buf, uint16_t length)
{
    
    FLASH_Unlock();
   
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
    FLASH_ErasePage(FLASH_USER_START_ADDR);

    FLASH_ProgramHalfWord(FLASH_MAGIC_ADDR, FLASH_MAGIC_NUMBER);

    for (uint32_t i = 0; i < length; i++) {
        FLASH_ProgramHalfWord(FLASH_USER_DATA_ADDR + i * 2, buf[i]);
    }
    FLASH_Lock();
}
/*FLASH���غ���*/
int LoadUsRegHoldingBufFromFlash(uint16_t* buf, uint16_t length)
{
    uint16_t magicNumber = *(__IO uint16_t*)FLASH_MAGIC_ADDR;

    /*���ݱ�Ǵ��󷵻�0*/
    if (magicNumber != FLASH_MAGIC_NUMBER) {
       
        return 0;
    }
   /*���ݶ�ȡ��������1*/
    for (uint32_t i = 0; i < length; i++) {
        buf[i] = *(__IO uint16_t*)(FLASH_USER_DATA_ADDR + i * 2);
    }

   
    return 1;
}

/*****************************************************************
  * @brief  ������
  * @param  ��
  * @retval ��
  * @note   Ӳ����ʼ��-������ʼ����
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;
	
/* ��ע��༶��ʼ�������в�Ҫ�漰��vTaskDelay��������Ϊ��û��
	�����������*/
  BSP_Init();
	
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate, 
                        (const char*    )"AppTaskCreate",
                        (uint16_t       )512,  
                        (void*          )NULL,
                        (UBaseType_t    )1, 
                        (TaskHandle_t*  )&AppTaskCreate_Handle);
          
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* �������񣬿������� */
  else
    return -1;  
  
  while(1);   /* ��������ִ�е����� */    
}


/***********************************************************************
  * @ ������  �� AppTaskCreate
  * @ ����˵���� Ϊ�˷���������е����񴴽����������������������
  * @ ����    �� ��  
  * @ ����ֵ  �� ��
  **********************************************************************/
static void AppTaskCreate(void)
{
  
  
  taskENTER_CRITICAL();           //�����ٽ���
  
  /* ����LED_Task���� */
  xTaskCreate((TaskFunction_t )Flash_Task, /* ������ں��� */
                        (const char*    )"Flash_Task",/* �������� */
                        (uint16_t       )128,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )2,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&Flash_Task_Handle);/* ������ƿ�ָ�� */
  
   
  xTaskCreate((TaskFunction_t )MODBUS_Task, /* ������ں��� */
                        (const char*    )"MODBUS_Task",/* �������� */
                        (uint16_t       )128,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )3,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&MODBUS_Task_Handle);/* ������ƿ�ָ�� */
  xTaskCreate((TaskFunction_t )SHT20_Task, /* ������ں��� */
                        (const char*    )"SHT20_Task",/* �������� */
                        (uint16_t       )512,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )2,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&SHT20_Task_Handle);/* ������ƿ�ָ�� */
	xTaskCreate((TaskFunction_t )HardWare_Task, /* ������ں��� */
                        (const char*    )"HardWare_Task",/* �������� */
                        (uint16_t       )512,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )2,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&HardWare_Task_Handle);/* ������ƿ�ָ�� */
  
  IWDG_Feed();
   
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}



/**********************************************************************
  * @ ������  �� Flash_Task
  * @ ����˵���� Flash_Taskʵ��Flash�洢���ּĴ������ݣ��Լ���λ���ص����ã�
                ʵ�ָ�λ���ּĴ���Ϊ��ʼ����
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/

static void Flash_Task(void* parameter)
{
    uint16_t bufferToWrite[REG_HOLDING_NREGS];
    uint8_t previousSwitchState = KEY_OFF;  

    while (1)
    {    
        /*���������뿪��ʱ���ὫusRegHoldingBuf�Ĵ�����ֵ����Ϊ����ĳ�ʼֵusRegHoldingBufInitial
			    ���Һ�ƻᱻ�����������ظ�λ�����Ϩ�𣬳�����������*/
        uint8_t currentSwitchState = Key_Scan(KEY1_GPIO_PORT, KEY1_GPIO_PIN);
        
        if (currentSwitchState == KEY_ON && previousSwitchState == KEY_OFF)
        {
            
            memcpy(usRegHoldingBuf, usRegHoldingBufInitial, sizeof(usRegHoldingBufInitial));
            
           
            SaveUsRegHoldingBufToFlash(usRegHoldingBuf, REG_HOLDING_NREGS);
            
            
            LED1_OFF;  
            
          
            vTaskDelay(500); 
        }
      
        else if (currentSwitchState == KEY_OFF && previousSwitchState == KEY_ON)
        {
           
            LED1_ON;  
        }
        previousSwitchState = currentSwitchState;

       /*��������MODBUS_task����Ҫ��������ݣ���������FLASH��*/
        if (xQueueReceive(xFlashQueue, bufferToWrite, 0) == pdPASS)
        {
            SaveUsRegHoldingBufToFlash(bufferToWrite, REG_HOLDING_NREGS);
        }
        IWDG_Feed();
        vTaskDelay(100);  
    }
}
/**********************************************************************
  * @ ������  �� Flash_Task
  * @ ����˵���� Modbusͨ�ţ���ʵʱ���Ĳ����ʣ�usRegHoldingBuf[1]��
                 ���豸��ַ(usRegHoldingBuf[0])
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void MODBUS_Task(void* parameter)
{

    uint16_t currentSlaveAddress = usRegHoldingBuf[0];
    uint16_t previousBaudRateIndex = usRegHoldingBuf[1];
    uint16_t previousUsartIndex = usRegHoldingBuf[10]; 
    uint32_t baudRate = 9600;
    

    switch (usRegHoldingBuf[1])
    {
        case 1: baudRate = 1200; break;
        case 2: baudRate = 2400; break;
        case 3: baudRate = 4800; break;
        case 4: baudRate = 9600; break;
        case 5: baudRate = 19200; break;
        case 6: baudRate = 38400; break;
        case 7: baudRate = 57600; break;
        case 8: baudRate = 76800; break;
        case 9: baudRate = 115200; break;
        default: baudRate = 9600; break;
    }


    eMBInit(MB_RTU, usRegHoldingBuf[0], previousUsartIndex, baudRate, MB_PAR_NONE);
    eMBEnable();

    while (1)
    {
 
        eMBPoll();
			/*��ʱ200ms��Ϊ�������λ������λ����Ҫ���޸Ĳ����ʺ�Ҫ�������*/
        vTaskDelay(200);

      /*����豸��ַ�Ƿ����ı䣬����仯��ʵʱ����*/
        if (currentSlaveAddress != usRegHoldingBuf[0])
        {
            currentSlaveAddress = usRegHoldingBuf[0];
            eMBDisable();
            eMBInit(MB_RTU, currentSlaveAddress, previousUsartIndex, baudRate, MB_PAR_NONE);
            eMBEnable();
        }

        /*��鲨�����Ƿ����ı䣬����仯��ʵʱ����*/
        if (previousBaudRateIndex != usRegHoldingBuf[1])
        {
            previousBaudRateIndex = usRegHoldingBuf[1];
            switch (usRegHoldingBuf[1])
            {
                case 1: baudRate = 1200; break;
                case 2: baudRate = 2400; break;
                case 3: baudRate = 4800; break;
                case 4: baudRate = 9600; break;
                case 5: baudRate = 19200; break;
                case 6: baudRate = 38400; break;
                case 7: baudRate = 57600; break;
                case 8: baudRate = 76800; break;
                case 9: baudRate = 115200; break;
                default: baudRate = 9600; break;
            }
            eMBDisable();
            eMBInit(MB_RTU, currentSlaveAddress, previousUsartIndex, baudRate, MB_PAR_NONE);
            eMBEnable();
        }

        /*����Ƿ�������ڣ�������ڷ����仯��ʵʱ����*/
        if (previousUsartIndex != usRegHoldingBuf[10])
        {
            previousUsartIndex = usRegHoldingBuf[10];
            eMBDisable(); 

            eMBInit(MB_RTU, currentSlaveAddress, previousUsartIndex, baudRate, MB_PAR_NONE);
            eMBEnable();
        }

        /*�Ա�usRegHoldingBuf�Ĵ�����ֵ�Ƿ����仯����������仯ͨ�����з��͸�flash���񱣴棬������usRegHoldingBuf����*/
        if (memcmp(previousUsRegHoldingBuf, usRegHoldingBuf, sizeof(usRegHoldingBuf)) != 0)
        {
            if (xQueueSend(xFlashQueue, usRegHoldingBuf, portMAX_DELAY) == pdPASS)
            {
                memcpy(previousUsRegHoldingBuf, usRegHoldingBuf, sizeof(usRegHoldingBuf));
            }
        }


        IWDG_Feed();
        vTaskDelay(100);
    }
}
/**********************************************************************
  * @ ������  �� SHT20_Task
  * @ ����˵���� ʵ��sht20�ɼ���ʪ�����ݣ���ʾ����ʾ�ȹ���
                
  * @ ����    �� ��
  * @ ����ֵ  �� ��
  ********************************************************************/
//static void SHT20_Task(void* parameter)
//{
//	/*LCD��ʼ������������û�з���BSP_Init����Ϊ�����õ���vTaskDelay������BSP_Init��ʼ��
//	�ڵ���֮ǰ���Ѿ�����ˣ�vTaskDelay������Ҫ���Ⱥ����ʹ��*/
//	  LCD_init();
//    while (1)
//    {
//			/*�ɼ�����������ֵΪ��ʪ������*/
//        SHT2X_TEST(); 

//        xQueueSend(xTempQueue, &temperatureC, portMAX_DELAY);
//        xQueueSend(xHumQueue, &humidityRH, portMAX_DELAY);
//	      IWDG_Feed();
//        vTaskDelay(100);  

//       /*��ʪ�ȵ�������С���ֱ�����usRegInputBuf�У�С�����ֱ�����λ*/
//        int16_t temp_integer_part = (int16_t)temperatureC;             
//        int16_t temp_fractional_part = (int16_t)((temperatureC - temp_integer_part) * 100); 
//        
//        int16_t hum_integer_part = (int16_t)humidityRH;             
//        int16_t hum_fractional_part = (int16_t)((humidityRH - hum_integer_part) * 100); 

//     
//        usRegInputBuf[2] = (uint16_t)temp_integer_part;
//        usRegInputBuf[3] = (uint16_t)temp_fractional_part;
//        
//       
//        usRegInputBuf[4] = (uint16_t)hum_integer_part;
//        usRegInputBuf[5] = (uint16_t)hum_fractional_part;

//      /*��ʾ����ʾ����*/
//			  lcd_display_TEMP(temperatureC*10);  /**10����Ϊlcd_display_TEMP����Ϊuint16_t,����С�����1λ*/
//        lcd_display_HUMI(humidityRH*10);   /**10����Ϊlcd_display_TEMP����Ϊuint16_t,����С�����1λ*/
//			 	IWDG_Feed();
//			  vTaskDelay(200);
//        }

//       
//				
//				
//    }
static void SHT20_Task(void* parameter)
{
	/*LCD��ʼ������������û�з���BSP_Init����Ϊ�����õ���vTaskDelay������BSP_Init��ʼ��
	�ڵ���֮ǰ���Ѿ�����ˣ�vTaskDelay������Ҫ���Ⱥ����ʹ��*/
	  LCD_init();
	  i2c_CheckDevice(&I2C3_Config,EEPROM_DEV_ADDR1);
	  IWDG_Feed();
	  NSA2862_E2PROM_Init();
    while (1)
    {
			/*�ɼ�����������ֵΪ��ʪ������*/
         presure = NSA2862_GetPData();
			 vTaskDelay(2000);
					IWDG_Feed();
			presure1=NSA2862_GetPData1();
//			usRegInputBuf[0] = (uint16_t)(presure >> 16);


//      usRegInputBuf[1] = (uint16_t)(presure & 0xFFFF);  
			 vTaskDelay(2000);
				IWDG_Feed();

//        xQueueSend(xTempQueue, &temperatureC, portMAX_DELAY);
//        xQueueSend(xHumQueue, &humidityRH, portMAX_DELAY);
//	      IWDG_Feed();
//        vTaskDelay(100);  

//       /*��ʪ�ȵ�������С���ֱ�����usRegInputBuf�У�С�����ֱ�����λ*/
//        int16_t temp_integer_part = (int16_t)temperatureC;             
//        int16_t temp_fractional_part = (int16_t)((temperatureC - temp_integer_part) * 100); 
//        
//        int16_t hum_integer_part = (int16_t)humidityRH;             
//        int16_t hum_fractional_part = (int16_t)((humidityRH - hum_integer_part) * 100); 

//     
//        usRegInputBuf[2] = (uint16_t)temp_integer_part;
//        usRegInputBuf[3] = (uint16_t)temp_fractional_part;
//        
//       
//        usRegInputBuf[4] = (uint16_t)hum_integer_part;
//        usRegInputBuf[5] = (uint16_t)hum_fractional_part;

//      /*��ʾ����ʾ����*/
//			  lcd_display_TEMP(temperatureC*10);  /**10����Ϊlcd_display_TEMP����Ϊuint16_t,����С�����1λ*/
//        lcd_display_HUMI(humidityRH*10);   /**10����Ϊlcd_display_TEMP����Ϊuint16_t,����С�����1λ*/
//			 	IWDG_Feed();
//			  vTaskDelay(200);
        }

       
				
				
    }
/**********************************************************************
  * @ ������  �� HardWare_Task
  * @ ����˵���� ��Ҫ����Ϊ4-20mA����ͼ̵���������        
  * @ ����    �� ��
  * @ ����ֵ  �� ��
  ********************************************************************/
static void HardWare_Task(void* parameter)
{
    float receivedTemp = 0;
    float receivedHum = 0;
    /*usRegHoldingBuf 6-9 ��������ʪ�ȵ���������ֵ���ǽ�����С������ֵ������100��������Ҫ/100�����Եõ�����С������ֵ*/
    float temp_zero = (float)usRegHoldingBuf[6] / 100.0f; 
    float temp_full = (float)usRegHoldingBuf[7] / 100.0f; 
    float hum_zero = (float)usRegHoldingBuf[8] / 100.0f;  
    float hum_full = (float)usRegHoldingBuf[9] / 100.0f;  
    
    while (1)
    {
       /*ͨ�����н�����ʪ������*/
        if (xQueueReceive(xTempQueue, &receivedTemp, portMAX_DELAY) == pdPASS)
        {
             IWDG_Feed();
        }
        
       
        if (xQueueReceive(xHumQueue, &receivedHum, portMAX_DELAY) == pdPASS)
        {
            IWDG_Feed();
        }
				/*ͨ����������ֵ���ɼ�����ʵʱ��ʪ��ֵ�����Զ�������У׼*/
        uint16_t dac_value1 = (uint16_t)(((receivedTemp - temp_zero) / (temp_full - temp_zero)) * (usRegHoldingBuf[12] - usRegHoldingBuf[11]) + usRegHoldingBuf[11]);
        uint16_t dac_value2 = (uint16_t)(((receivedHum - hum_zero) / (hum_full - hum_zero)) * (usRegHoldingBuf[14] - usRegHoldingBuf[13]) + usRegHoldingBuf[13]);

        /*usRegHoldingBuf 15-18 ��������ʪ�ȵ����ֵ�����ֵ���ǽ�����С������ֵ������100��������Ҫ/100�����Եõ�����С������ֵ*/
				float temp_low_alarm = (float)usRegHoldingBuf[15] / 100.0f; 
        float temp_high_alarm = (float)usRegHoldingBuf[16] / 100.0f;
        float hum_low_alarm = (float)usRegHoldingBuf[17] / 100.0f;   
        float hum_high_alarm = (float)usRegHoldingBuf[18] / 100.0f; 
				
        /*usRegHoldingBuf 19-20 ������ʪ�ȵı���ģʽѡ��*/			
        uint16_t temp_alarm_mode = usRegHoldingBuf[19];    
        uint16_t hum_alarm_mode = usRegHoldingBuf[20];    

				/*���ݱ���ģʽʵ�̵ּ���1����2�ı���*/
        if (temp_alarm_mode == 0)  
        {
            if (receivedTemp < temp_low_alarm)
            {
                RELAY1(Bit_SET); 
            }
            else
            {
                RELAY1(Bit_RESET); 
            }
        }
        else if (temp_alarm_mode == 1)  
        {
            if (receivedTemp > temp_high_alarm)
            {
                RELAY1(Bit_SET); 
            }
            else
            {
                RELAY1(Bit_RESET); 
            }
        }
        else if (temp_alarm_mode == 2)  
        {
            if (receivedTemp > temp_low_alarm && receivedTemp < temp_high_alarm)
            {
                RELAY1(Bit_SET); 
            }
            else
            {
                RELAY1(Bit_RESET); 
            }
        }


        if (hum_alarm_mode == 0)  
        {
            if (receivedHum < hum_low_alarm)
            {
                RELAY2(Bit_SET); 
            }
            else
            {
                RELAY2(Bit_RESET); 
            }
        }
        else if (hum_alarm_mode == 1)  
        {
            if (receivedHum > hum_high_alarm)
            {
                RELAY2(Bit_SET);
            }
            else
            {
                RELAY2(Bit_RESET); 
            }
        }
        else if (hum_alarm_mode == 2)  
        {
            if (receivedHum > hum_low_alarm && receivedHum < hum_high_alarm)
            {
                RELAY2(Bit_SET);
            }
            else
            {
                RELAY2(Bit_RESET); 
            }
        }
        IWDG_Feed();
        vTaskDelay(100);
        
      /*����λ����ϣ�ѡ�������׼�¶Ⱥ�ʪ�ȵ�4mA-20mA�����������*/
        if (usRegHoldingBuf[21] == 1 && usRegHoldingBuf[23] == 1)
        {
            set_4_20mA_output(usRegHoldingBuf[11], usRegHoldingBuf[13]);
        }
        else if (usRegHoldingBuf[22] == 1 && usRegHoldingBuf[24] == 1)
        {
           set_4_20mA_output(usRegHoldingBuf[12], usRegHoldingBuf[14]);
        }
        else
       {
          set_4_20mA_output(dac_value1, dac_value2);
        }
        IWDG_Feed();
        vTaskDelay(500);
    }
}


/***********************************************************************
  * @ ������  �� BSP_Init
  * @ ����˵���� ��������ĳ�ʼ��
  * @ ����    ��   
  * @ ����ֵ  �� ��
  *********************************************************************/
static void BSP_Init(void)
{
	   NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	   LED_GPIO_Config();
	   Key_GPIO_Config();
//     I2C_Configuration();  //SHT20��I2C��ʼ��
//     SHT2X_Init();
     init_I2C_modules(); //4-20mA��I2C��ʼ��
	   Relay_GPIO_Init();
	   IWDG_Config(IWDG_Prescaler_64 ,1875); /*���ÿ��Ź����ʱ��Ϊ2s*/
	 
     xTempQueue = xQueueCreate(1, sizeof(float));
     xHumQueue = xQueueCreate(1, sizeof(float));
     xFlashQueue = xQueueCreate(5, sizeof(uint16_t) * REG_HOLDING_NREGS);
     /*���ر�����Flash�е�usRegHoldingBuf��ֵ���������led��*/
		if (LoadUsRegHoldingBufFromFlash(usRegHoldingBuf, REG_HOLDING_NREGS) == 0) {
        LED1_OFF;
    }
	   /*���ص�usRegHoldingBuf��ֵ���Ƶ�previousUsRegHoldingBuf����MODBUS_task�����Ƚ�*/
		 memcpy(previousUsRegHoldingBuf, usRegHoldingBuf, sizeof(usRegHoldingBuf));
		
}


/********************************END OF FILE****************************/
