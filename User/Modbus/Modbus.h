#ifndef __MODBUS_H
#define	__MODBUS_H

#include "stm32f10x.h"
#include "mb.h"
#include "mbutils.h"

//����Ĵ�����ʼ��ַ
#define REG_INPUT_START       0x0000

//����Ĵ�������
#define REG_INPUT_NREGS       29

//���ּĴ�����ʼ��ַ
#define REG_HOLDING_START     0x0000

//���ּĴ�������
#define REG_HOLDING_NREGS     25

//��Ȧ��ʼ��ַ
#define REG_COILS_START       0x0000

//��Ȧ����
#define REG_COILS_SIZE        16

//���ؼĴ�����ʼ��ַ
#define REG_DISCRETE_START    0x0000

//���ؼĴ�������
#define REG_DISCRETE_SIZE     16


extern uint16_t usRegInputBuf[REG_INPUT_NREGS];

extern uint16_t usRegInputStart;

extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];

extern uint16_t usRegHoldingStart;

extern uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8];

extern uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];

#endif

