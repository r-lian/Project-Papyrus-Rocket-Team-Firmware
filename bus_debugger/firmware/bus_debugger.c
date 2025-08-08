/**
 * @file tc_controller.c
 * @brief Thermocouple Controller Source
 * @author Papyrus Avionics Team
 * @date 2024
 */
#include "bus_debugger.h"
#include "lcd_nhd_c0220biz.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

BusDebugger this;

UART_HandleTypeDef *stdio_uart;

int is_onebit(uint8_t x) { return (x != 0) && (x & (x - 1)) == 0; }

void read_buttons() {
  this.newly_pressed = this.buttons_pressed;
  this.buttons_pressed = 0;
  for (uint8_t i = 0; i < 7; i++) {
    if (HAL_GPIO_ReadPin(GPIO(this.button_pins[i])) == GPIO_PIN_SET) {
      this.buttons_pressed |= (1 << i);
    }
  }
  this.newly_pressed = this.buttons_pressed & (~this.newly_pressed);
  // Key repeat algorithm: if jjust one button is held, increment timer if it's
  // the same as before Past a base point of 500ms, every 100ms trigger a
  // keypress if it's pressed but not newly pressed
  if (is_onebit(this.buttons_pressed) &&
      this.keyrep_mask == this.buttons_pressed) {
    this.keyrep_timer++;
    if (this.keyrep_timer > 16 && ((this.keyrep_timer & 3) == 0)) {
      this.newly_pressed |= this.buttons_pressed;
    }
  } else {
    this.keyrep_timer = 0;
    if (is_onebit(this.buttons_pressed))
      this.keyrep_mask = this.buttons_pressed;
  }
}
bool cursor_flash() { return (bool)!((HAL_GetTick() & 1023) > 512); }
void buttons_mentry(MenuEntry *entry) {
  switch (entry->type) {
  case MENT_LABEL:
    if (entry->label.is_scrollable) {
      if ((this.newly_pressed & BUTTON_LEFT) && entry->label.scroll_pos > 0) {
        entry->label.scroll_pos--;
      }
      if ((this.newly_pressed & BUTTON_RIGHT) &&
          entry->label.scroll_pos < strlen(entry->label.text) - 19) {
        entry->label.scroll_pos++;
      }
      if (this.newly_pressed & BUTTON_NO) {
        entry->label.scroll_pos = 0;
      }
    }
  default:;
  }
}
void render_mentry(MenuEntry *entry, uint8_t row, bool is_selected) {
  nhd_set_pos(row, 0, &this.display);
  if ((int)is_selected && (int)cursor_flash()) {
    nhd_write_char('>', &this.display);
  } else {
    nhd_write_char(' ', &this.display);
  }
  uint8_t line_length = 19;
  char line_buf[20];
  switch (entry->type) {
  case MENT_LABEL:
    strncpy(line_buf, entry->label.text + entry->label.scroll_pos, line_length);
    line_buf[line_length] = 0;
    memset(line_buf + strlen(line_buf), ' ', line_length - strlen(line_buf));
    nhd_write_str(line_buf, &this.display);
    break;
  case MENT_PRESSABLE:
    strncpy(line_buf, entry->pressable.text, line_length);
    line_buf[line_length] = 0;
    memset(line_buf + strlen(line_buf), ' ', line_length - strlen(line_buf));
    nhd_write_str(line_buf, &this.display);
    break;
  case MENT_CHOICE:
    render_mentry(&entry->choice.choices[entry->choice.cur_choice], row,
                  is_selected);
    break;
  case MENT_VARIABLE:
    nhd_write_str("placeholder", &this.display);
  default:;
  }
  if (is_selected)
    buttons_mentry(entry);
}

void buttons_menu(Menu *menu) {
  if (menu->cur_entry < menu->num_entries - 1 &&
      (this.newly_pressed & BUTTON_DOWN)) {
    menu->cur_entry++;
  }
  if (menu->cur_entry > 0 && (this.newly_pressed & BUTTON_UP)) {
    menu->cur_entry--;
  }
}
void render_menu(Menu *menu) {
  render_mentry(&menu->entries[menu->scroll_pos], 0,
                menu->cur_entry == menu->scroll_pos);
  if (menu->scroll_pos + 1 < menu->num_entries) {
    render_mentry(&menu->entries[menu->scroll_pos + 1], 1,
                  menu->cur_entry == menu->scroll_pos + 1);
  }
  buttons_menu(menu);
  if (menu->scroll_pos < menu->cur_entry - 1) {
    menu->scroll_pos = menu->cur_entry - 1;
  } else if (menu->cur_entry < menu->scroll_pos) {
    menu->scroll_pos = menu->cur_entry;
  }
}

bool need_reinit = false;
int main() {
  if (bus_debugger_init(&this) != PAPYRUS_OK) {
    Error_Handler();
  }
  this.backlight_sleep = 0;
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIO(this.backlight), GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO(this.lcd_reset), GPIO_PIN_SET);
  HAL_Delay(50);
  nhd_init(&this.display);
  Menu testmenu;
  testmenu.prev_menu = nullptr;
  testmenu.num_entries = 3;

  MenuEntry testm1;
  testm1.type = MENT_LABEL;
  testm1.label.text = "Hello world!";
  testm1.label.is_scrollable = false;
  testm1.label.owned_text = false;
  testm1.label.update_text = nullptr;
  testm1.label.scroll_pos = 0;

  MenuEntry testm2;
  testm2.type = MENT_LABEL;
  testm2.label.text = "Hieroglyph says hi";
  testm2.label.is_scrollable = false;
  testm2.label.owned_text = false;
  testm2.label.update_text = nullptr;
  testm2.label.scroll_pos = 0;
  MenuEntry testm3;
  testm3.type = MENT_LABEL;
  testm3.label.text =
      "This is an incredibly long string but it can still be displayed!!!";
  testm3.label.is_scrollable = true;
  testm3.label.owned_text = false;
  testm3.label.update_text = nullptr;
  testm3.label.scroll_pos = 0;

  testmenu.scroll_pos = 0;
  testmenu.cur_entry = 0;
  testmenu.num_entries = 3;
  testmenu.entries = calloc(3, sizeof(MenuEntry));
  if (testmenu.entries == NULL) {
    Error_Handler();
    return 1;
  }
  testmenu.entries[0] = testm1;
  testmenu.entries[1] = testm2;
  testmenu.entries[2] = testm3;

  while (true) {
    read_buttons();
    // nhd_clear(&this.display);
    // nhd_write_str("HELLO", &this.display);
    render_menu(&testmenu);
    if (this.buttons_pressed)
      this.backlight_sleep = 0;
    this.backlight_sleep++;
    if (this.backlight_sleep > 1500) {
      HAL_GPIO_WritePin(GPIO(this.backlight), GPIO_PIN_RESET);
      if (this.backlight_sleep > 3000) {
        need_reinit = true;
        nhd_shutdown(&this.display);
      }
    } else {
      if (need_reinit) {
        need_reinit = false;
        nhd_init(&this.display);
      }
      HAL_GPIO_WritePin(GPIO(this.backlight), GPIO_PIN_SET);
    }
    while (!next_tick)
      ;
    next_tick = false;
  }
}
