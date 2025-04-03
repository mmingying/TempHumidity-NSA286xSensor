#include "FreeRTOS.h"
#include "task.h"
#include "bsp_nsa2862.h"
#include "bsp_i2c_gpio.h"
#include "bsp_usart.h" 
#include "bsp_iwdg.h" 

#define NSA2862_VOLTAGE_VERSION (0x95)
#define NSA2862_CURRENT_SVERSION (0x85)
extern I2C_PinConfig_t I2C3_Config;

static uint8_t status = 0x01;
//jiegouti xinxi zai lainheti zhong ,shizixia xiangshang de 
static struct NSA2862_Config Config = {
    .SYS_CONFIG1.SYS_CONFIG1_VALUE = 0,
    .SYS_CONFIG2.JFET_DIS = 1,             //jinyong tiaojieqi
    .SYS_CONFIG2.JFET_LVL = 1,             //taiojieqi shuchu 3.3v danshangbian yijingjinyongle 
    .SYS_CONFIG2.VREF_DIS = 0,             //meiyou jinyong caokaodianya
	.SYS_CONFIG2.VREF_LVL = 1,             //cankaodianya :2.45V
    .SYS_CONFIG2.T_OUT_EN = 0,            //buqiyong geshishuchu
	.SYS_CONFIG2.RSV1 = 7,                 //baoliuwei :7wei
    .Current_EXC.Current_EXC_VALUE = 0xF0,      // jinyong waibudianliuyuan 
    .PCH_Config2.SYS_CHOP_EN = 1,            //qiyong xitongzhnbo
    .PCH_Config1.GAIN_P = 0xD,              //1010,fangda 64bei   
	.PCH_Config1.ODR_P = 0x09,                 //0111:18.75HZ
    .PCH_Config2.RAW_P = 1,               //jiang yuan shi shuzhi baocun zai jicunqi zhong 
};


/*
static struct NSA2862_Config Config =
{
    .SYS_CONFIG1.SYS_CONFIG1_VALUE = 0,
    .SYS_CONFIG2.JFET_DIS = 1,
    .SYS_CONFIG2.JFET_LVL = 1,
    .SYS_CONFIG2.VREF_DIS = 0,
    .SYS_CONFIG2.VREF_LVL = 1,
    .SYS_CONFIG2.T_OUT_EN = 0,
    .SYS_CONFIG2.RSV1 = 7,
    .Current_EXC.Current_EXC_VALUE = 0,
    .PCH_Config2.SYS_CHOP_EN = 1,
    .PCH_Config1.GAIN_P = 0,
    .PCH_Config1.ODR_P = 7,
    .PCH_Config2.RAW_P = 1,
};
*/

uint8_t ee_ReadBytes(I2C_PinConfig_t *pinConfig, uint8_t *_pReadBuf, uint8_t _usAddress, uint8_t _usSize)
{
    uint8_t i;
    i2c_Start(pinConfig);
    
    i2c_SendByte(pinConfig, EEPROM_DEV_ADDR1 | EEPROM_I2C_WR);	

    if (i2c_WaitAck(pinConfig) != 0)
    {
        goto cmd_fail;	
    }
    i2c_SendByte(pinConfig, _usAddress);
    if (i2c_WaitAck(pinConfig) != 0)
    {
        goto cmd_fail;	
    }
    
    i2c_Start(pinConfig);	
    i2c_SendByte(pinConfig, EEPROM_DEV_ADDR1 | EEPROM_I2C_RD);	
    
    if (i2c_WaitAck(pinConfig) != 0)
    {
        goto cmd_fail;	
    }	
	
    for (i = 0; i < _usSize; i++)
    {
        _pReadBuf[i] = i2c_ReadByte(pinConfig);	
        
        if (i != _usSize - 1)
        {
            i2c_Ack(pinConfig);
        }
        else
        {
            i2c_NAck(pinConfig);	
        }
    }
    i2c_Stop(pinConfig);
    return 1;	

cmd_fail: 
    i2c_Stop(pinConfig);
    return 0;
}


uint8_t ee_WriteBytes(I2C_PinConfig_t *pinConfig, uint8_t *_pWriteBuf, uint8_t _usAddress, uint8_t _usSize)
{
    uint8_t i;
    
    i2c_Start(pinConfig);
    i2c_SendByte(pinConfig, EEPROM_DEV_ADDR1 | EEPROM_I2C_WR);	
    
    if (i2c_WaitAck(pinConfig) != 0)
    {
        goto cmd_fail;
    }
    
    i2c_SendByte(pinConfig, _usAddress);
    
    if (i2c_WaitAck(pinConfig) != 0)
    {
        goto cmd_fail;	
    }
    
    for (i = 0; i < _usSize; i++) {
        i2c_SendByte(pinConfig, _pWriteBuf[i]);
        if (i2c_WaitAck(pinConfig) != 0) {
            goto cmd_fail;
        }
    }
    
    i2c_Stop(pinConfig);
    return 1;

cmd_fail: 
    i2c_Stop(pinConfig);
    return 0;
}

//void NSA2862_E2PROM_Init(void) {
//  NSA2862_CommandMode();
//  uint8_t magic_version = 0;
//  NSA2862_ReadReg(0xb9, &magic_version, 1);
//  if (magic_version != NSA2862_VOLTAGE_VERSION) {
//    NSA2862_WriteReg(SYS_CONFIG1_ADDRESS, Config.SYS_CONFIG1.SYS_CONFIG1_VALUE);
//    NSA2862_WriteReg(SYS_CONFIG2_ADDRESS, Config.SYS_CONFIG2.SYS_CONFIG2_VALUE);
//    NSA2862_WriteReg(Current_EXC_ADDRESS, Config.Current_EXC.Current_EXC_VALUE);
//    NSA2862_WriteReg(PCH_Config1_ADDRESS, Config.PCH_Config1.PCH_Config1_VALUE);
//    NSA2862_WriteReg(PCH_Config2_ADDRESS, Config.PCH_Config2.PCH_Config2_VALUE);
//    NSA2862_WriteReg(0xb9, NSA2862_VOLTAGE_VERSION);
//    NSA2862_E2PROM_Flash();
//    vTaskDelay(100);
//    NSA2862_CommandMode();
//  }
//}
void NSA2862_E2PROM_Init(void) {
  NSA2862_CommandMode();
    NSA2862_WriteReg(SYS_CONFIG1_ADDRESS, Config.SYS_CONFIG1.SYS_CONFIG1_VALUE);
    NSA2862_WriteReg(SYS_CONFIG2_ADDRESS, Config.SYS_CONFIG2.SYS_CONFIG2_VALUE);
    NSA2862_WriteReg(Current_EXC_ADDRESS, Config.Current_EXC.Current_EXC_VALUE);
    NSA2862_WriteReg(PCH_Config1_ADDRESS, Config.PCH_Config1.PCH_Config1_VALUE);

    NSA2862_WriteReg(PCH_Config2_ADDRESS, Config.PCH_Config2.PCH_Config2_VALUE);
   
    NSA2862_E2PROM_Flash();
	IWDG_Feed();
    vTaskDelay(100);
    NSA2862_CommandMode();
  
}

uint32_t NSA2862_GetPData1(void)
{
	NSA2862_NormalMode();
	vTaskDelay(100);
	uint8_t Data[4] = {0};
  uint32_t conv_data = 0;
  uint8_t i = 0;
  for (i = 0; i < 4; i++) {
    Data[i] = 0;
  }
	uint8_t cmd = NSA2862_PDATA_REG;
	NSA2862_ReadReg(NSA2862_PDATA_REG,&Data[1],3);
	conv_data = 0;
  for (i = 0; i < 4; i++) {
    conv_data |= (Data[i] << ((3 - i) * 8));
  }
	vTaskDelay(100);
	NSA2862_CommandMode();
	return conv_data;
}
float NSA2862_GetPData(void)
{
    NSA2862_NormalMode();
    vTaskDelay(100);

    uint8_t Data[4] = {0};
    uint32_t conv_data = 0;
    uint8_t i = 0;

    for (i = 0; i < 4; i++) {
        Data[i] = 0;
    }

    uint8_t cmd = NSA2862_PDATA_REG;
    NSA2862_ReadReg(NSA2862_PDATA_REG, &Data[1], 3);

    conv_data = 0;
    for (i = 0; i < 4; i++) {
        conv_data |= (Data[i] << ((3 - i) * 8));
    }

    vTaskDelay(100);
    NSA2862_CommandMode();

 
    const uint32_t adcMin = 8388608;
    const uint32_t adcMax = 16777216;
    const float pressureMax = 1.6f;

   
    if (conv_data < adcMin) {
        return 0.0f; 
    } else if (conv_data > adcMax) {
        return pressureMax; 
    }

    float pressure = ((float)(conv_data - adcMin) / (adcMax - adcMin)) * pressureMax;

    return pressure;
}


void NSA2862_ReadReg(uint8_t reg, uint8_t *data, uint8_t len) {
    ee_ReadBytes(&I2C3_Config, data, reg, len); 
}


void NSA2862_WriteReg(uint8_t reg, uint8_t data) {
    ee_WriteBytes(&I2C3_Config, &data, reg, 1);  
    vTaskDelay(10);
}


void NSA2862_CommandMode(void) {
    uint8_t dataValue = 0x00;  
    uint8_t regAddress = 0x30; 
    uint8_t readValue;
    ee_WriteBytes(&I2C3_Config, &dataValue, regAddress, 1);
    vTaskDelay(10);
}


void NSA2862_NormalMode(void) {
    uint8_t dataValue = 0x03;  
    uint8_t regAddress = 0x30; 
    uint8_t readValue;
    ee_WriteBytes(&I2C3_Config, &dataValue, regAddress, 1);
    vTaskDelay(20);
}


void NSA2862_E2PROM_Flash(void) {
    uint8_t dataValue1 = 0x33;  
    uint8_t regAddress1 = 0x30; 
    uint8_t dataValue2 = 0x7E;  
    uint8_t regAddress2 = 0x6A; 

    ee_WriteBytes(&I2C3_Config, &dataValue1, regAddress1, 1);
    ee_WriteBytes(&I2C3_Config, &dataValue2, regAddress2, 1);
    ee_ReadBytes(&I2C3_Config, &status, regAddress2, 1);
    while (status != 0x00) {
        ee_ReadBytes(&I2C3_Config, &status, regAddress2, 1);
    }
		IWDG_Feed();
    vTaskDelay(1500);
}
