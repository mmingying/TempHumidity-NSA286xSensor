#ifndef __MODBUS_H
#define	__MODBUS_H

#include "stm32f10x.h"
#include "mb.h"
#include "mbutils.h"

//输入寄存器起始地址
#define REG_INPUT_START       0x0000

//输入寄存器数量
#define REG_INPUT_NREGS       29

//保持寄存器起始地址
#define REG_HOLDING_START     0x0000

//保持寄存器数量
#define REG_HOLDING_NREGS     25

//线圈起始地址
#define REG_COILS_START       0x0000

//线圈数量
#define REG_COILS_SIZE        16

//开关寄存器起始地址
#define REG_DISCRETE_START    0x0000

//开关寄存器数量
#define REG_DISCRETE_SIZE     16


extern uint16_t usRegInputBuf[REG_INPUT_NREGS];

extern uint16_t usRegInputStart;

extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];

extern uint16_t usRegHoldingStart;

extern uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8];

extern uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];

#endif

