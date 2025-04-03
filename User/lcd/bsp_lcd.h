
#ifndef __BSP_LCD_H
#define	__BSP_LCD_H

#include "stm32f10x.h"
/*����ĵ�ַ0x09-0x15��ÿ����ַ��Ӧ�������ʾ��*/
#define COOL_ADDR        0x15 
#define HOT_ADDR         0x15 
#define HUMI_SYMBOL_ADDR 0x14 
#define SIX_8_ADDR       0x13
#define HUMI_POINT_ADDR  0x12 
#define FIVE_8_ADDR      0x11
#define HUMI_FONT_ADDR   0x10 
#define FORE_8_ADDR      0x0F
#define TEMP_SYMBOL_ADDR 0x0E 
#define THREE_8_ADDR     0x0D
#define TEMP_POINT_ADDR  0x0C 
#define SECOND_8_ADDR    0x0B
#define TEMP_FONT_ADDR   0x0A 
#define FIRST_8_ADDR     0x09

/*ÿ���ε�����Ҫ��������ֵ*/
#define DISPLAY_COOL         0x02   //���� ѩ�� ������
#define DISPLAY_HOT          0x01   //���� ̫�� ������
#define DISPLAY_HUMI_SYMBOL  0x01   //���� ʪ�ȷ��� % ������
#define DISPLAY_HUMI_POINT   0x01   //���� ʪ��С���� . ������
#define DISPLAY_HUMI_FONT    0x08   //���� ʪ������ ʵ��ʪ�� ������
#define DISPLAY_TEMP_SYMBOL  0x08   //���� �¶ȷ��� �� ������
#define DISPLAY_TEMP_POINT   0x08   //���� �¶�С���� . ������
#define DISPLAY_TEMP_FONT    0x01   //���� �¶����� ʵ���¶� ������

/*�¶Ⱥ�ʪ�ȣ��ֱ��Ϊ���������֣�ÿ�����ֶ���ĵ�����ͬ���ֵ�����*/
#define DISPLAY_TEMP_FIR_8_0 0xFA | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_1 0x0A | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_2 0xD6 | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_3 0x9E | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_4 0x2E | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_5 0xBC | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_6 0xFC | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_7 0x1A | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_8 0xFE | DISPLAY_TEMP_FONT
#define DISPLAY_TEMP_FIR_8_9 0xBE | DISPLAY_TEMP_FONT

#define DISPLAY_TEMP_SEC_8_0 0xF5 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_1 0x05 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_2 0xD3 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_3 0x97 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_4 0x27 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_5 0xB6 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_6 0xF6 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_7 0x15 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_8 0xF7 | DISPLAY_TEMP_POINT
#define DISPLAY_TEMP_SEC_8_9 0xB7 | DISPLAY_TEMP_POINT

#define DISPLAY_TEMP_THD_8_0 0xF5 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_1 0x05 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_2 0xD3 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_3 0x97 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_4 0x27 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_5 0xB6 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_6 0xF6 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_7 0x15 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_8 0xF7 | DISPLAY_TEMP_SYMBOL
#define DISPLAY_TEMP_THD_8_9 0xB7 | DISPLAY_TEMP_SYMBOL

#define DISPLAY_HUMI_FIR_8_0 0xF5 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_1 0x05 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_2 0xB6 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_3 0x97 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_4 0x47 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_5 0xD3 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_6 0xF3 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_7 0x85 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_8 0xF7 | DISPLAY_HUMI_FONT
#define DISPLAY_HUMI_FIR_8_9 0xD7 | DISPLAY_HUMI_FONT

#define DISPLAY_HUMI_SEC_8_0 0xFA | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_1 0x0A | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_2 0xBC | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_3 0x9E | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_4 0x4E | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_5 0xD6 | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_6 0xF6 | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_7 0x8A | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_8 0xFE | DISPLAY_HUMI_POINT
#define DISPLAY_HUMI_SEC_8_9 0xDE | DISPLAY_HUMI_POINT

#define DISPLAY_HUMI_THD_8_0 0xFA | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_1 0x0A | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_2 0xBC | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_3 0x9E | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_4 0x4E | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_5 0xD6 | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_6 0xF6 | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_7 0x8A | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_8 0xFE | DISPLAY_HUMI_SYMBOL
#define DISPLAY_HUMI_THD_8_9 0xDE | DISPLAY_HUMI_SYMBOL

static void temp_dec_calc(uint16_t value);
static void temp_unit_calc(uint16_t value);
static void temp_decPlace_calc(uint16_t value);

static void humi_dec_calc(uint16_t value);
static void humi_unit_calc(uint16_t value);
static void humi_decPlace_calc(uint16_t value);

static void lcd_clear(void);
 void LCD_init(void);
 void LCD_display_hot(void);
 void LCD_display_cool(void);
 void lcd_display_TEMP(uint16_t tempValue);
 void lcd_display_HUMI(uint16_t humiValue);
#endif 
