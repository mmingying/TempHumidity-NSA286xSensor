/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */


#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern USHORT usRegHoldingBuf[];
extern SemaphoreHandle_t xFlashLoadedSemaphore;

static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);
uint16_t min=1;

static USART_TypeDef* current_usart;
static GPIO_TypeDef* current_gpio;
static uint16_t current_tx_pin;
static uint16_t current_rx_pin;
static uint8_t current_irq_channel;

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    if (xRxEnable)
    {
        USART_ITConfig(current_usart, USART_IT_RXNE, ENABLE);
        if (current_usart == USART1)
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_8); 
        }
    }
    else
    {
        USART_ITConfig(current_usart, USART_IT_RXNE, DISABLE);
        if (current_usart == USART1)
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_8); 
        }
    }

    if (xTxEnable)
    {
        USART_ITConfig(current_usart, USART_IT_TC, ENABLE);
    }
    else
    {
        USART_ITConfig(current_usart, USART_IT_TC, DISABLE);
    }
}

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    (void)ucPORT;
    (void)ucDataBits;
    (void)eParity;
   if (usRegHoldingBuf[10] == 1) {
    current_usart = USART1;
    current_gpio = GPIOA;
    current_tx_pin = GPIO_Pin_9;
    current_rx_pin = GPIO_Pin_10;
    current_irq_channel = USART1_IRQn;

    USART_Config(ulBaudRate);  

    } else if (usRegHoldingBuf[10] == 3) {
        current_usart = USART3;
        current_gpio = GPIOB;
        current_tx_pin = GPIO_Pin_10;
        current_rx_pin = GPIO_Pin_11;
        current_irq_channel = USART3_IRQn;

     
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

        GPIO_InitStructure.GPIO_Pin = current_tx_pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(current_gpio, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = current_rx_pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(current_gpio, &GPIO_InitStructure);

        USART_InitStructure.USART_BaudRate = ulBaudRate;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(current_usart, &USART_InitStructure);
        USART_Cmd(current_usart, ENABLE);
    } else {
        return FALSE;  
    }

    NVIC_InitStructure.NVIC_IRQChannel = current_irq_channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    USART_SendData(current_usart, ucByte);
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
    *pucByte = USART_ReceiveData(current_usart);
    return TRUE;
}

static void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

static void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}

void USART1_IRQHandler(void)
{
    if (current_usart == USART1)
    {
        if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
        {
            prvvUARTRxISR();
            USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        }
        if (USART_GetITStatus(USART1, USART_IT_TC) == SET)
        {
            prvvUARTTxReadyISR();
            USART_ClearITPendingBit(USART1, USART_IT_TC);
        }
    }
}

void USART3_IRQHandler(void)
{
    if (current_usart == USART3)
    {
        if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
        {
            prvvUARTRxISR();
            USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        }
        if (USART_GetITStatus(USART3, USART_IT_TC) == SET)
        {
            prvvUARTTxReadyISR();
            USART_ClearITPendingBit(USART3, USART_IT_TC);
        }
    }
}

