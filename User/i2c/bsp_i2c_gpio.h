#ifndef _BSP_I2C_GPIO_H
#define _BSP_I2C_GPIO_H

#include <inttypes.h>
#include "stm32f10x.h"  // ??STM32??

#define EEPROM_I2C_WR    0  /* ???bit */
#define EEPROM_I2C_RD    1  /* ???bit */
#define EEPROM_DEV_ADDR  0xB0

/* ??I2C??????? */
typedef struct {
    GPIO_TypeDef *GPIO_Port;   // GPIO??
    uint16_t SCL_Pin;          // SCL??
    uint16_t SDA_Pin;          // SDA??
} I2C_PinConfig_t;

/* ???? */
void i2c_Init(I2C_PinConfig_t *pinConfig);
void i2c_Start(I2C_PinConfig_t *pinConfig);
void i2c_Stop(I2C_PinConfig_t *pinConfig);
void i2c_SendByte(I2C_PinConfig_t *pinConfig, uint8_t data);
uint8_t i2c_ReadByte(I2C_PinConfig_t *pinConfig);
uint8_t i2c_WaitAck(I2C_PinConfig_t *pinConfig);
void i2c_Ack(I2C_PinConfig_t *pinConfig);
void i2c_NAck(I2C_PinConfig_t *pinConfig);
void GP8212S_write(I2C_PinConfig_t *pinConfig, uint8_t devAddr, uint16_t DAC_data);
uint8_t ee_CheckOk(I2C_PinConfig_t *pinConfig, uint8_t address);
uint8_t i2c_CheckDevice(I2C_PinConfig_t *pinConfig, uint8_t _Address);
#endif




