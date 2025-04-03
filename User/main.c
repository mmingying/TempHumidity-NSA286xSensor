/**
  *********************************************************************
  * @file    温湿度-sht20
  * @author  ChenMing
  * @version V1.0
  * @date    2024-12-24
  * @brief   FreeRTOS-sht20-显示屏-4-20mA输出-modbus485-4G
  *********************************************************************
  * @attention
  *
  * 公司：纽克智造(南京)有限公司
  * 联系方式：CYMJ17203730@163.com
  *
  *
  **********************************************************************
  */ 
	
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* 开发板硬件bsp头文件 */
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

/*队列、信号量声明*/
QueueHandle_t xTempQueue;
QueueHandle_t xHumQueue;
QueueHandle_t xFlashQueue;
SemaphoreHandle_t xFlashLoadedSemaphore;  

/*采集的温湿度变量，在sht20.c中定义*/
//extern float temperatureC;
//extern float humidityRH;
float presure;
uint32_t presure1;
/*I2C的SDA和SCL引脚定义，后期使用I2C，可直接添加参数设置*/
I2C_PinConfig_t I2C1_Config = {GPIOA, GPIO_Pin_2, GPIO_Pin_1}; 
I2C_PinConfig_t I2C2_Config = {GPIOA, GPIO_Pin_4, GPIO_Pin_3}; 
I2C_PinConfig_t I2C3_Config = {GPIOA, GPIO_Pin_12, GPIO_Pin_11};

/*任务句柄*/
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t Flash_Task_Handle = NULL;
static TaskHandle_t MODBUS_Task_Handle = NULL;
static TaskHandle_t SHT20_Task_Handle = NULL;
static TaskHandle_t HardWare_Task_Handle = NULL;

/*相关函数定义*/
static void AppTaskCreate(void);
static void Flash_Task(void* pvParameters);
static void BSP_Init(void);
static void MODBUS_Task(void* pvParameters);
static void SHT20_Task(void* pvParameters);
static void HardWare_Task(void* pvParameters);

/*Flash保存起始-结束地址*/
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800C000)    //起始地址
#define FLASH_MAGIC_NUMBER    0x1234                      //自定义的有效数据标记
#define FLASH_MAGIC_ADDR      FLASH_USER_START_ADDR       //数据标记起始地址
#define FLASH_USER_DATA_ADDR  (FLASH_USER_START_ADDR + 4) //自定义数据起始地址 

/*保持寄存器初始值定义*/
static uint16_t previousUsRegHoldingBuf[REG_HOLDING_NREGS];
static const uint16_t usRegHoldingBufInitial[REG_HOLDING_NREGS] = {
    0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1388,
    0x00, 0x2710, 0x01, 0x1450, 0x663A, 0x1478, 0x6680,
    0x00, 0x1388, 0x00, 0x2710, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};

/*I2C初始化函数*/
void init_I2C_modules(void)
{
    i2c_Init(&I2C1_Config);  
    i2c_Init(&I2C2_Config);
    i2c_Init(&I2C3_Config);  
}
/*4-20mA输出函数*/
void set_4_20mA_output(uint16_t dac_value1, uint16_t dac_value2)
{
    GP8212S_write(&I2C1_Config, 0xB0, dac_value1);  
    GP8212S_write(&I2C2_Config, 0xB0, dac_value2);  
}
/*FLASH存入函数*/
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
/*FLASH重载函数*/
int LoadUsRegHoldingBufFromFlash(uint16_t* buf, uint16_t length)
{
    uint16_t magicNumber = *(__IO uint16_t*)FLASH_MAGIC_ADDR;

    /*数据标记错误返回0*/
    if (magicNumber != FLASH_MAGIC_NUMBER) {
       
        return 0;
    }
   /*数据读取正常返回1*/
    for (uint32_t i = 0; i < length; i++) {
        buf[i] = *(__IO uint16_t*)(FLASH_USER_DATA_ADDR + i * 2);
    }

   
    return 1;
}

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   硬件初始化-创建开始函数
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;
	
/* 请注意班级初始化函数中不要涉及到vTaskDelay函数，因为还没有
	启动任务调度*/
  BSP_Init();
	
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate, 
                        (const char*    )"AppTaskCreate",
                        (uint16_t       )512,  
                        (void*          )NULL,
                        (UBaseType_t    )1, 
                        (TaskHandle_t*  )&AppTaskCreate_Handle);
          
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* 启动任务，开启调度 */
  else
    return -1;  
  
  while(1);   /* 正常不会执行到这里 */    
}


/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  
  
  taskENTER_CRITICAL();           //进入临界区
  
  /* 创建LED_Task任务 */
  xTaskCreate((TaskFunction_t )Flash_Task, /* 任务入口函数 */
                        (const char*    )"Flash_Task",/* 任务名字 */
                        (uint16_t       )128,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )2,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&Flash_Task_Handle);/* 任务控制块指针 */
  
   
  xTaskCreate((TaskFunction_t )MODBUS_Task, /* 任务入口函数 */
                        (const char*    )"MODBUS_Task",/* 任务名字 */
                        (uint16_t       )128,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )3,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&MODBUS_Task_Handle);/* 任务控制块指针 */
  xTaskCreate((TaskFunction_t )SHT20_Task, /* 任务入口函数 */
                        (const char*    )"SHT20_Task",/* 任务名字 */
                        (uint16_t       )512,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )2,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&SHT20_Task_Handle);/* 任务控制块指针 */
	xTaskCreate((TaskFunction_t )HardWare_Task, /* 任务入口函数 */
                        (const char*    )"HardWare_Task",/* 任务名字 */
                        (uint16_t       )512,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )2,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&HardWare_Task_Handle);/* 任务控制块指针 */
  
  IWDG_Feed();
   
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}



/**********************************************************************
  * @ 函数名  ： Flash_Task
  * @ 功能说明： Flash_Task实现Flash存储保持寄存器数据，以及复位开关的设置，
                实现复位保持寄存器为初始数据
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/

static void Flash_Task(void* parameter)
{
    uint16_t bufferToWrite[REG_HOLDING_NREGS];
    uint8_t previousSwitchState = KEY_OFF;  

    while (1)
    {    
        /*当波动拨码开关时，会将usRegHoldingBuf寄存器的值重载为定义的初始值usRegHoldingBufInitial
			    并且红灯会被点亮，当开关复位，红灯熄灭，程序正常运行*/
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

       /*接受来自MODBUS_task任务要保存的数据，并保存在FLASH中*/
        if (xQueueReceive(xFlashQueue, bufferToWrite, 0) == pdPASS)
        {
            SaveUsRegHoldingBufToFlash(bufferToWrite, REG_HOLDING_NREGS);
        }
        IWDG_Feed();
        vTaskDelay(100);  
    }
}
/**********************************************************************
  * @ 函数名  ： Flash_Task
  * @ 功能说明： Modbus通信，可实时更改波特率（usRegHoldingBuf[1]）
                 和设备地址(usRegHoldingBuf[0])
  * @ 参数    ：   
  * @ 返回值  ： 无
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
			/*延时200ms是为了配合上位机，上位机需要当修改波特率后不要立马更改*/
        vTaskDelay(200);

      /*检查设备地址是否发生改变，如果变化则实时更新*/
        if (currentSlaveAddress != usRegHoldingBuf[0])
        {
            currentSlaveAddress = usRegHoldingBuf[0];
            eMBDisable();
            eMBInit(MB_RTU, currentSlaveAddress, previousUsartIndex, baudRate, MB_PAR_NONE);
            eMBEnable();
        }

        /*检查波特率是否发生改变，如果变化则实时更新*/
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

        /*检查是否更换串口，如果串口发生变化，实时更改*/
        if (previousUsartIndex != usRegHoldingBuf[10])
        {
            previousUsartIndex = usRegHoldingBuf[10];
            eMBDisable(); 

            eMBInit(MB_RTU, currentSlaveAddress, previousUsartIndex, baudRate, MB_PAR_NONE);
            eMBEnable();
        }

        /*对比usRegHoldingBuf寄存器的值是否发生变化，如果发生变化通过队列发送给flash任务保存，并更新usRegHoldingBuf内容*/
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
  * @ 函数名  ： SHT20_Task
  * @ 功能说明： 实现sht20采集温湿度数据，显示屏显示等功能
                
  * @ 参数    ： 无
  * @ 返回值  ： 无
  ********************************************************************/
//static void SHT20_Task(void* parameter)
//{
//	/*LCD初始化放在任务中没有放在BSP_Init，因为其中用到了vTaskDelay函数，BSP_Init初始化
//	在调度之前就已经完成了，vTaskDelay函数需要调度后才能使用*/
//	  LCD_init();
//    while (1)
//    {
//			/*采集函数，返回值为温湿度数据*/
//        SHT2X_TEST(); 

//        xQueueSend(xTempQueue, &temperatureC, portMAX_DELAY);
//        xQueueSend(xHumQueue, &humidityRH, portMAX_DELAY);
//	      IWDG_Feed();
//        vTaskDelay(100);  

//       /*温湿度的整数和小数分别存放在usRegInputBuf中，小数部分保留两位*/
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

//      /*显示屏显示函数*/
//			  lcd_display_TEMP(temperatureC*10);  /**10是因为lcd_display_TEMP参数为uint16_t,保留小数点后1位*/
//        lcd_display_HUMI(humidityRH*10);   /**10是因为lcd_display_TEMP参数为uint16_t,保留小数点后1位*/
//			 	IWDG_Feed();
//			  vTaskDelay(200);
//        }

//       
//				
//				
//    }
static void SHT20_Task(void* parameter)
{
	/*LCD初始化放在任务中没有放在BSP_Init，因为其中用到了vTaskDelay函数，BSP_Init初始化
	在调度之前就已经完成了，vTaskDelay函数需要调度后才能使用*/
	  LCD_init();
	  i2c_CheckDevice(&I2C3_Config,EEPROM_DEV_ADDR1);
	  IWDG_Feed();
	  NSA2862_E2PROM_Init();
    while (1)
    {
			/*采集函数，返回值为温湿度数据*/
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

//       /*温湿度的整数和小数分别存放在usRegInputBuf中，小数部分保留两位*/
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

//      /*显示屏显示函数*/
//			  lcd_display_TEMP(temperatureC*10);  /**10是因为lcd_display_TEMP参数为uint16_t,保留小数点后1位*/
//        lcd_display_HUMI(humidityRH*10);   /**10是因为lcd_display_TEMP参数为uint16_t,保留小数点后1位*/
//			 	IWDG_Feed();
//			  vTaskDelay(200);
        }

       
				
				
    }
/**********************************************************************
  * @ 函数名  ： HardWare_Task
  * @ 功能说明： 主要功能为4-20mA输出和继电器波报警        
  * @ 参数    ： 无
  * @ 返回值  ： 无
  ********************************************************************/
static void HardWare_Task(void* parameter)
{
    float receivedTemp = 0;
    float receivedHum = 0;
    /*usRegHoldingBuf 6-9 保存了温湿度的零点和满程值，是将带有小数的数值扩大了100，所以这要/100，可以得到带有小数的数值*/
    float temp_zero = (float)usRegHoldingBuf[6] / 100.0f; 
    float temp_full = (float)usRegHoldingBuf[7] / 100.0f; 
    float hum_zero = (float)usRegHoldingBuf[8] / 100.0f;  
    float hum_full = (float)usRegHoldingBuf[9] / 100.0f;  
    
    while (1)
    {
       /*通过队列接收温湿度数据*/
        if (xQueueReceive(xTempQueue, &receivedTemp, portMAX_DELAY) == pdPASS)
        {
             IWDG_Feed();
        }
        
       
        if (xQueueReceive(xHumQueue, &receivedHum, portMAX_DELAY) == pdPASS)
        {
            IWDG_Feed();
        }
				/*通过零点和满程值将采集到的实时温湿度值进行自定义线性校准*/
        uint16_t dac_value1 = (uint16_t)(((receivedTemp - temp_zero) / (temp_full - temp_zero)) * (usRegHoldingBuf[12] - usRegHoldingBuf[11]) + usRegHoldingBuf[11]);
        uint16_t dac_value2 = (uint16_t)(((receivedHum - hum_zero) / (hum_full - hum_zero)) * (usRegHoldingBuf[14] - usRegHoldingBuf[13]) + usRegHoldingBuf[13]);

        /*usRegHoldingBuf 15-18 保存了温湿度的最高值和最低值，是将带有小数的数值扩大了100，所以这要/100，可以得到带有小数的数值*/
				float temp_low_alarm = (float)usRegHoldingBuf[15] / 100.0f; 
        float temp_high_alarm = (float)usRegHoldingBuf[16] / 100.0f;
        float hum_low_alarm = (float)usRegHoldingBuf[17] / 100.0f;   
        float hum_high_alarm = (float)usRegHoldingBuf[18] / 100.0f; 
				
        /*usRegHoldingBuf 19-20 保存温湿度的报警模式选择*/			
        uint16_t temp_alarm_mode = usRegHoldingBuf[19];    
        uint16_t hum_alarm_mode = usRegHoldingBuf[20];    

				/*根据报警模式实现继电器1或者2的报警*/
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
        
      /*与上位机配合，选择输出标准温度和湿度的4mA-20mA或者正常输出*/
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
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 各种外设的初始化
  * @ 参数    ：   
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
	   NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	   LED_GPIO_Config();
	   Key_GPIO_Config();
//     I2C_Configuration();  //SHT20的I2C初始化
//     SHT2X_Init();
     init_I2C_modules(); //4-20mA的I2C初始化
	   Relay_GPIO_Init();
	   IWDG_Config(IWDG_Prescaler_64 ,1875); /*设置看门狗溢出时间为2s*/
	 
     xTempQueue = xQueueCreate(1, sizeof(float));
     xHumQueue = xQueueCreate(1, sizeof(float));
     xFlashQueue = xQueueCreate(5, sizeof(uint16_t) * REG_HOLDING_NREGS);
     /*重载保存在Flash中的usRegHoldingBuf的值，如果错误，led亮*/
		if (LoadUsRegHoldingBufFromFlash(usRegHoldingBuf, REG_HOLDING_NREGS) == 0) {
        LED1_OFF;
    }
	   /*重载的usRegHoldingBuf的值复制到previousUsRegHoldingBuf，在MODBUS_task中作比较*/
		 memcpy(previousUsRegHoldingBuf, usRegHoldingBuf, sizeof(usRegHoldingBuf));
		
}


/********************************END OF FILE****************************/
