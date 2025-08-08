#pragma once
#include "papyrus_hardware.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_i2c.h"

extern const uint8_t lcd_addr;

extern void Error_Handler();

void nhd_init(PapyrusI2C *display);
void nhd_set_pos(uint8_t row, uint8_t col, PapyrusI2C *display);
void nhd_write_char(char ch, PapyrusI2C *display);
void nhd_write_str(const char *str, PapyrusI2C *display);
void nhd_clear(PapyrusI2C *display);
void nhd_shutdown(PapyrusI2C *display);
