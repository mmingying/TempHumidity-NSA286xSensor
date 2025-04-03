#include "bsp_lcd.h"
#include "driver_ht1621.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_lcd.h"
#include "driver_ht1621.h"
#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  初始化LCD，清除显示并显示温湿度符号
  * @retval 无
  */
void LCD_init(void)
{
    HT1621_Init();                  // 初始化HT1621 LCD显示驱动
    lcd_clear();                    // 清除LCD屏幕显示内容
    vTaskDelay(200);                // 延时200ms，确保显示初始化完成
    HT1621_WriteData4Bit(TEMP_SYMBOL_ADDR, DISPLAY_TEMP_SYMBOL); // 显示温度符号
    HT1621_WriteData4Bit(TEMP_POINT_ADDR, DISPLAY_TEMP_POINT);   // 显示温度小数点
    HT1621_WriteData4Bit(TEMP_FONT_ADDR, DISPLAY_TEMP_FONT);     // 显示温度字体：实测温度

    HT1621_WriteData4Bit(HUMI_SYMBOL_ADDR, DISPLAY_HUMI_SYMBOL); // 显示湿度符号
    HT1621_WriteData4Bit(HUMI_POINT_ADDR, DISPLAY_HUMI_POINT);   // 显示湿度小数点
    HT1621_WriteData4Bit(HUMI_FONT_ADDR, DISPLAY_HUMI_FONT);     // 显示湿度字体：实测湿度
}

/**
  * @brief  显示"太阳"
  * @retval 无
  */
void LCD_display_hot(void)
{
    HT1621_WriteData4Bit(HOT_ADDR, DISPLAY_HOT);  // 显示"热"字
}

/**
  * @brief  显示"雪花"
  * @retval 无
  */
void LCD_display_cool(void)
{
    HT1621_WriteData4Bit(COOL_ADDR, DISPLAY_COOL);  // 显示"凉"字
}

/**
  * @brief  根据温度值显示温度
  * @param  tempValue 温度值（16位无符号整数）
  * @retval 无
  */
void lcd_display_TEMP(uint16_t tempValue)
{
    temp_dec_calc(tempValue);      // 计算温度的百位数并显示
    temp_unit_calc(tempValue);     // 计算温度的十位数并显示
    temp_decPlace_calc(tempValue); // 计算温度的个位数并显示
}

/**
  * @brief  根据湿度值显示湿度
  * @param  humiValue 湿度值（16位无符号整数）
  * @retval 无
  */
void lcd_display_HUMI(uint16_t humiValue)
{
    humi_dec_calc(humiValue);      // 计算湿度的百位数并显示
    humi_unit_calc(humiValue);     // 计算湿度的十位数并显示
    humi_decPlace_calc(humiValue); // 计算湿度的个位数并显示
}

/**
  * @brief  计算并显示温度的百位数字
  * @param  value 温度值（16位无符号整数）
  * @retval 无
  */
static void temp_dec_calc(uint16_t value)
{
    uint8_t temp = value / 100;   // 获取温度的百位数字

    // 根据百位数字显示相应的字符
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
  * @brief  计算并显示温度的十位数字
  * @param  value 温度值（16位无符号整数）
  * @retval 无
  */
static void temp_unit_calc(uint16_t value)
{
    uint8_t temp = (value % 100) / 10;  // 获取温度的十位数字

    // 根据十位数字显示相应的字符
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
  * @brief  计算并显示温度的个位数字
  * @param  value 温度值（16位无符号整数）
  * @retval 无
  */
static void temp_decPlace_calc(uint16_t value)
{
    uint8_t temp = value % 10;    // 获取温度的个位数字

    // 根据个位数字显示相应的字符
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
  * @brief  计算并显示湿度的百位数字
  * @param  value 湿度值（16位无符号整数）
  * @retval 无
  */
static void humi_dec_calc(uint16_t value)
{
    uint8_t temp = value / 100;   // 获取湿度的百位数字

    // 根据百位数字显示相应的字符
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
  * @brief  计算并显示湿度的十位数字
  * @param  value 湿度值（16位无符号整数）
  * @retval 无
  */
static void humi_unit_calc(uint16_t value)
{
    uint8_t temp = (value % 100) / 10;  // 获取湿度的十位数字

    // 根据十位数字显示相应的字符
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
  * @brief  计算并显示湿度的个位数字
  * @param  value 湿度值（16位无符号整数）
  * @retval 无
  */
static void humi_decPlace_calc(uint16_t value)
{
    uint8_t temp = value % 10;    // 获取湿度的个位数字

    // 根据个位数字显示相应的字符
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
  * @brief  清除LCD显示
  * @retval 无
  */
static void lcd_clear(void)
{
    // 从地址0x09到0x15清除所有数据
    for (uint8_t i = 0x09; i < 0x16; i++) {
        HT1621_WriteData4Bit(i, 0);  // 清除对应地址的显示内容
    }
}
