#include "bsp_lcd.h"
#include "driver_ht1621.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_lcd.h"
#include "driver_ht1621.h"
#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  ��ʼ��LCD�������ʾ����ʾ��ʪ�ȷ���
  * @retval ��
  */
void LCD_init(void)
{
    HT1621_Init();                  // ��ʼ��HT1621 LCD��ʾ����
    lcd_clear();                    // ���LCD��Ļ��ʾ����
    vTaskDelay(200);                // ��ʱ200ms��ȷ����ʾ��ʼ�����
    HT1621_WriteData4Bit(TEMP_SYMBOL_ADDR, DISPLAY_TEMP_SYMBOL); // ��ʾ�¶ȷ���
    HT1621_WriteData4Bit(TEMP_POINT_ADDR, DISPLAY_TEMP_POINT);   // ��ʾ�¶�С����
    HT1621_WriteData4Bit(TEMP_FONT_ADDR, DISPLAY_TEMP_FONT);     // ��ʾ�¶����壺ʵ���¶�

    HT1621_WriteData4Bit(HUMI_SYMBOL_ADDR, DISPLAY_HUMI_SYMBOL); // ��ʾʪ�ȷ���
    HT1621_WriteData4Bit(HUMI_POINT_ADDR, DISPLAY_HUMI_POINT);   // ��ʾʪ��С����
    HT1621_WriteData4Bit(HUMI_FONT_ADDR, DISPLAY_HUMI_FONT);     // ��ʾʪ�����壺ʵ��ʪ��
}

/**
  * @brief  ��ʾ"̫��"
  * @retval ��
  */
void LCD_display_hot(void)
{
    HT1621_WriteData4Bit(HOT_ADDR, DISPLAY_HOT);  // ��ʾ"��"��
}

/**
  * @brief  ��ʾ"ѩ��"
  * @retval ��
  */
void LCD_display_cool(void)
{
    HT1621_WriteData4Bit(COOL_ADDR, DISPLAY_COOL);  // ��ʾ"��"��
}

/**
  * @brief  �����¶�ֵ��ʾ�¶�
  * @param  tempValue �¶�ֵ��16λ�޷���������
  * @retval ��
  */
void lcd_display_TEMP(uint16_t tempValue)
{
    temp_dec_calc(tempValue);      // �����¶ȵİ�λ������ʾ
    temp_unit_calc(tempValue);     // �����¶ȵ�ʮλ������ʾ
    temp_decPlace_calc(tempValue); // �����¶ȵĸ�λ������ʾ
}

/**
  * @brief  ����ʪ��ֵ��ʾʪ��
  * @param  humiValue ʪ��ֵ��16λ�޷���������
  * @retval ��
  */
void lcd_display_HUMI(uint16_t humiValue)
{
    humi_dec_calc(humiValue);      // ����ʪ�ȵİ�λ������ʾ
    humi_unit_calc(humiValue);     // ����ʪ�ȵ�ʮλ������ʾ
    humi_decPlace_calc(humiValue); // ����ʪ�ȵĸ�λ������ʾ
}

/**
  * @brief  ���㲢��ʾ�¶ȵİ�λ����
  * @param  value �¶�ֵ��16λ�޷���������
  * @retval ��
  */
static void temp_dec_calc(uint16_t value)
{
    uint8_t temp = value / 100;   // ��ȡ�¶ȵİ�λ����

    // ���ݰ�λ������ʾ��Ӧ���ַ�
    if (temp == 0) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_0);
    } else if (temp == 1) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_1);
    } else if (temp == 2) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_2);
    } else if (temp == 3) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_3);
    } else if (temp == 4) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_4);
    } else if (temp == 5) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_5);
    } else if (temp == 6) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_6);
    } else if (temp == 7) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_7);
    } else if (temp == 8) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_8);
    } else if (temp == 9) {
        HT1621_WriteData8Bits(FIRST_8_ADDR, DISPLAY_TEMP_FIR_8_9);
    }
}

/**
  * @brief  ���㲢��ʾ�¶ȵ�ʮλ����
  * @param  value �¶�ֵ��16λ�޷���������
  * @retval ��
  */
static void temp_unit_calc(uint16_t value)
{
    uint8_t temp = (value % 100) / 10;  // ��ȡ�¶ȵ�ʮλ����

    // ����ʮλ������ʾ��Ӧ���ַ�
    if (temp == 0) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_0);
    } else if (temp == 1) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_1);
    } else if (temp == 2) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_2);
    } else if (temp == 3) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_3);
    } else if (temp == 4) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_4);
    } else if (temp == 5) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_5);
    } else if (temp == 6) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_6);
    } else if (temp == 7) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_7);
    } else if (temp == 8) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_8);
    } else if (temp == 9) {
        HT1621_WriteData8Bits(SECOND_8_ADDR, DISPLAY_TEMP_SEC_8_9);
    }
}

/**
  * @brief  ���㲢��ʾ�¶ȵĸ�λ����
  * @param  value �¶�ֵ��16λ�޷���������
  * @retval ��
  */
static void temp_decPlace_calc(uint16_t value)
{
    uint8_t temp = value % 10;    // ��ȡ�¶ȵĸ�λ����

    // ���ݸ�λ������ʾ��Ӧ���ַ�
    if (temp == 0) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_0);
    } else if (temp == 1) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_1);
    } else if (temp == 2) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_2);
    } else if (temp == 3) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_3);
    } else if (temp == 4) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_4);
    } else if (temp == 5) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_5);
    } else if (temp == 6) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_6);
    } else if (temp == 7) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_7);
    } else if (temp == 8) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_8);
    } else if (temp == 9) {
        HT1621_WriteData8Bits(THREE_8_ADDR, DISPLAY_TEMP_THD_8_9);
    }
}

/**
  * @brief  ���㲢��ʾʪ�ȵİ�λ����
  * @param  value ʪ��ֵ��16λ�޷���������
  * @retval ��
  */
static void humi_dec_calc(uint16_t value)
{
    uint8_t temp = value / 100;   // ��ȡʪ�ȵİ�λ����

    // ���ݰ�λ������ʾ��Ӧ���ַ�
    if (temp == 0) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_0);
    } else if (temp == 1) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_1);
    } else if (temp == 2) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_2);
    } else if (temp == 3) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_3);
    } else if (temp == 4) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_4);
    } else if (temp == 5) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_5);
    } else if (temp == 6) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_6);
    } else if (temp == 7) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_7);
    } else if (temp == 8) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_8);
    } else if (temp == 9) {
        HT1621_WriteData8Bits(FORE_8_ADDR, DISPLAY_HUMI_FIR_8_9);
    }
}

/**
  * @brief  ���㲢��ʾʪ�ȵ�ʮλ����
  * @param  value ʪ��ֵ��16λ�޷���������
  * @retval ��
  */
static void humi_unit_calc(uint16_t value)
{
    uint8_t temp = (value % 100) / 10;  // ��ȡʪ�ȵ�ʮλ����

    // ����ʮλ������ʾ��Ӧ���ַ�
    if (temp == 0) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_0);
    } else if (temp == 1) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_1);
    } else if (temp == 2) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_2);
    } else if (temp == 3) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_3);
    } else if (temp == 4) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_4);
    } else if (temp == 5) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_5);
    } else if (temp == 6) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_6);
    } else if (temp == 7) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_7);
    } else if (temp == 8) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_8);
    } else if (temp == 9) {
        HT1621_WriteData8Bits(FIVE_8_ADDR, DISPLAY_HUMI_SEC_8_9);
    }
}

/**
  * @brief  ���㲢��ʾʪ�ȵĸ�λ����
  * @param  value ʪ��ֵ��16λ�޷���������
  * @retval ��
  */
static void humi_decPlace_calc(uint16_t value)
{
    uint8_t temp = value % 10;    // ��ȡʪ�ȵĸ�λ����

    // ���ݸ�λ������ʾ��Ӧ���ַ�
    if (temp == 0) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_0);
    } else if (temp == 1) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_1);
    } else if (temp == 2) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_2);
    } else if (temp == 3) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_3);
    } else if (temp == 4) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_4);
    } else if (temp == 5) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_5);
    } else if (temp == 6) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_6);
    } else if (temp == 7) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_7);
    } else if (temp == 8) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_8);
    } else if (temp == 9) {
        HT1621_WriteData8Bits(SIX_8_ADDR, DISPLAY_HUMI_THD_8_9);
    }
}

/**
  * @brief  ���LCD��ʾ
  * @retval ��
  */
static void lcd_clear(void)
{
    // �ӵ�ַ0x09��0x15�����������
    for (uint8_t i = 0x09; i < 0x16; i++) {
        HT1621_WriteData4Bit(i, 0);  // �����Ӧ��ַ����ʾ����
    }
}
