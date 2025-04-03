#ifndef __I2C_EE_H
#define	__I2C_EE_H


#include "stm32f10x.h"
#include "bsp_i2c_gpio.h"
#pragma anon_unions


	

#define EEPROM_DEV_ADDR1			0xDA		/* 24xx02的设备地址 */
#define EEPROM_PAGE_SIZE		  8			  /* 24xx02的页面大小 */
#define EEPROM_SIZE				  256			  /* 24xx02总容量 */

#define NSA2862_DPH (0xD1)
#define NSA2862_DPL (0xD4)
#define NSA2862_PDATA_REG (0x06)

#define SYS_CONFIG1_ADDRESS (0xA1)
#define SYS_CONFIG2_ADDRESS (0xA2)
#define Current_EXC_ADDRESS (0xA3)
#define PCH_Config1_ADDRESS (0xA4)
#define PCH_Config2_ADDRESS (0xA5)

struct NSA2862_Config {
  union
  {
    uint8_t SYS_CONFIG1_VALUE;
    struct {
      uint8_t RSV1:1;
      uint8_t OWI_DIS:1;
      uint8_t OWI_WINDOW:1;
      uint8_t OWI_AC_EN:1;
      uint8_t RSV2:2;
      uint8_t BURNOUT_EN:1;
      uint8_t CAL_MODE:1;
    };
  } SYS_CONFIG1;
  union
  {
    uint8_t SYS_CONFIG2_VALUE;
    struct {
      uint8_t RSV1:3;
      uint8_t T_OUT_EN:1;
      uint8_t VREF_LVL:1;
      uint8_t VREF_DIS:1;
      uint8_t JFET_LVL:1;
      uint8_t JFET_DIS:1;
    };
  } SYS_CONFIG2;
  union
  {
    uint8_t Current_EXC_VALUE;
    struct {
      uint8_t IEXC1:4;
      uint8_t IEXC2:4;
    };
  } Current_EXC;
  union
  {
    uint8_t PCH_Config1_VALUE;
    struct {
      uint8_t ODR_P:4;
      uint8_t GAIN_P:4;
    };
  } PCH_Config1;
  union
  {
    uint8_t PCH_Config2_VALUE;
    struct {
      uint8_t RAW_P:1;
      uint8_t INPUT_SWAP:1;
      uint8_t SYS_CHOP_EN:1;
      uint8_t RSV:5;
    };
  } PCH_Config2;
};
//typedef struct
//{
//    float32_t P_Data;
//} NSA2862_Struct;



uint8_t ee_ReadBytes(I2C_PinConfig_t *pinConfig, uint8_t *_pReadBuf, uint8_t _usAddress, uint8_t _usSize);
uint8_t ee_WriteBytes(I2C_PinConfig_t *pinConfig, uint8_t *_pWriteBuf, uint8_t _usAddress, uint8_t _usSize);
void ee_Erase(void);
uint8_t ee_Test(void);
void NSA2862_E2PROM_Init( void );
void NSA2862_CommandMode(void);
void NSA2862_NormalMode(void);
void NSA2862_WriteReg(uint8_t reg, uint8_t data);
void NSA2862_ReadReg(uint8_t reg, uint8_t *data, uint8_t len);
uint32_t NSA2862_GetPData1(void);
float NSA2862_GetPData(void);
void NSA2862_E2PROM_Flash(void);

#endif /* __I2C_EE_H */

