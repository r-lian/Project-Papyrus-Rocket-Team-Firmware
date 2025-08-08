
#include "lcd_nhd_c0220biz.h"
#include "stm32c0xx_hal_i2c.h"
const uint8_t lcd_addr = 0x78;

void lcd_i2c_send(uint8_t *pData, uint16_t size, I2C_HandleTypeDef *i2c) {
  if (HAL_I2C_Master_Transmit(i2c, (uint16_t)lcd_addr, pData, size, 5000) !=
      HAL_OK) {

    if (HAL_I2C_GetError(i2c) != HAL_I2C_ERROR_AF) {
      Error_Handler();
    }
  }
}
void lcd_i2c_sendbyte(uint8_t commandChar, I2C_HandleTypeDef *i2c) {
  uint8_t command[2];

  command[0] = 0x00;
  command[1] = commandChar;
  lcd_i2c_send(command, 2, i2c);
}

void lcd_clear(I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(0x01, i2c);
  HAL_Delay(2);
}
void lcd_write_ch(char ch, I2C_HandleTypeDef *i2c) {
  uint8_t command[2];
  command[0] = 0x40;
  command[1] = (uint8_t)ch;
  lcd_i2c_send(command, 2, i2c);
}
void lcd_return_home(I2C_HandleTypeDef *i2c) { lcd_i2c_sendbyte(0x02, i2c); }
const uint8_t LCD_FLAG_8BITS = 0x10;
const uint8_t LCD_FLAG_2LINES = 0x08;
const uint8_t LCD_FLAG_DOUBLE_HEIGHT = 0x04;
const uint8_t LCD_FLAG_ITBL1 = 0x01;
void lcd_function_set(uint8_t flags, I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(0x20 | flags, i2c);
  HAL_Delay(20);
}
void lcd_bias_set(bool high_bias, I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(((int)high_bias ? 0x1c : 0x14), i2c);
  HAL_Delay(2);
}
void lcd_contrast_set(uint8_t contrast, I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(0x70 | contrast, i2c);
  HAL_Delay(2);
}
const uint8_t LCD_PWR_ICON = 0x08;
const uint8_t LCD_PWR_BOOST = 0x04;
void lcd_power_set(uint8_t high_contrast, uint8_t flags,
                   I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(0x50 | flags | high_contrast, i2c);
  HAL_Delay(2);
}
const uint8_t LCD_FOLLOW_ON = 0x08;
void lcd_follow_set(uint8_t amp, uint8_t flags, I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(0x60 | flags | amp, i2c);
  HAL_Delay(2);
}
const uint8_t LCD_DISPLAY_ON = 0x04;
const uint8_t LCD_CURSOR_ON = 0x04;
const uint8_t LCD_CURPOS_ON = 0x04;
void lcd_display_set(uint8_t flags, I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(0x08 | flags, i2c);
  HAL_Delay(2);
}
const uint8_t LCD_ENTRY_INC = 0x02;
const uint8_t LCD_SHIFT_ON = 0x01;
void lcd_entry_set(uint8_t flags, I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(0x04 | flags, i2c);
  HAL_Delay(2);
}
void lcd_set_addr(uint8_t addr, I2C_HandleTypeDef *i2c) {
  lcd_i2c_sendbyte(addr | 0x80, i2c);
  HAL_Delay(2);
}

void nhd_init(PapyrusI2C *display) {
  // 38 -> 2 lines, 8 bit interface
  // 14 -> display shift, S/C
  // 78
  // 5e
  // 6d
  // 0c
  // 01
  // 06
  I2C_HandleTypeDef *i2c = &display->handle;
  lcd_function_set(LCD_FLAG_8BITS | LCD_FLAG_2LINES, i2c);
  lcd_function_set(LCD_FLAG_8BITS | LCD_FLAG_2LINES | LCD_FLAG_ITBL1, i2c);
  lcd_bias_set(false, i2c);
  lcd_contrast_set(0x8, i2c);
  lcd_power_set(2, LCD_PWR_ICON | LCD_PWR_BOOST, i2c);
  lcd_follow_set(5, LCD_FOLLOW_ON, i2c);
  lcd_display_set(LCD_DISPLAY_ON, i2c);
  lcd_clear(i2c);
  lcd_entry_set(LCD_ENTRY_INC, i2c);
}
void nhd_set_pos(uint8_t row, uint8_t col, PapyrusI2C *display) {
  lcd_set_addr((row << 6) | col, &display->handle);
}
void nhd_write_char(char ch, PapyrusI2C *display) {
  lcd_write_ch(ch, &display->handle);
}
void nhd_clear(PapyrusI2C *display) { lcd_clear(&display->handle); }
void nhd_write_str(const char *str, PapyrusI2C *display) {
  for (const char *p = str; *p; p++) {
    nhd_write_char(*p, display);
  }
}
void nhd_shutdown(PapyrusI2C *display) {
  I2C_HandleTypeDef *i2c = &display->handle;
  lcd_power_set(0, 0, i2c);
  HAL_Delay(60);
}
