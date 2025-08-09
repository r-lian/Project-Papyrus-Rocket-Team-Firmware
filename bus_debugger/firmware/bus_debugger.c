/**
 * @file tc_controller.c
 * @brief Thermocouple Controller Source
 * @author Papyrus Avionics Team
 * @date 2024
 */
#include "bus_debugger.h"
#include "lcd_nhd_c0220biz.h"
#include "menus.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_adc.h"
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

Menu *cur_menu;

void allocation_failed() { Error_Handler(); }

extern void Reset_Handler();

void reset_board(void *unused) {
  UNUSED(unused);
  NVIC_SystemReset();
}

void read_buttons() {
  this.newly_pressed = this.buttons_pressed;
  this.buttons_pressed = 0;
  for (uint8_t i = 0; i < 7; i++) {
    if (HAL_GPIO_ReadPin(GPIO(this.button_pins[i])) == GPIO_PIN_SET) {
      this.buttons_pressed |= (1 << i);
    }
  }
  this.newly_pressed = this.buttons_pressed & (~this.newly_pressed);
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
    break;
  case MENT_PRESSABLE:
    if (entry->pressable.on_left != NULL && this.newly_pressed & BUTTON_LEFT) {
      entry->pressable.on_left(entry->pressable.argument);
    }
    if (entry->pressable.on_right != NULL &&
        this.newly_pressed & BUTTON_RIGHT) {
      entry->pressable.on_right(entry->pressable.argument);
    }
    if (entry->pressable.on_yes != NULL && this.newly_pressed & BUTTON_YES) {
      entry->pressable.on_yes(entry->pressable.argument);
    }
    if (entry->pressable.on_no != NULL && this.newly_pressed & BUTTON_NO) {
      entry->pressable.on_no(entry->pressable.argument);
    }
    if (entry->pressable.on_menu != NULL && this.newly_pressed & BUTTON_MENU) {
      entry->pressable.on_menu(entry->pressable.argument);
    }
    break;
  default:;
  }
}

float get_bat_voltage() {
  HAL_ADC_Start(&this.batread.handle);
  HAL_ADC_PollForConversion(&this.batread.handle, HAL_MAX_DELAY);
  uint32_t raw = HAL_ADC_GetValue(&this.batread.handle);
  // return (fixed32)(((uint64_t)raw) * 0xe30a3);
  return 14.19 * ((float)raw / 4096.0);
}
void render_mentry(MenuEntry *entry, uint8_t row, bool is_selected) {
  nhd_set_pos(row, 0, &this.display);
  if ((int)is_selected && (int)cursor_flash()) {
    nhd_write_char('*', &this.display);
  } else {
    nhd_write_char(' ', &this.display);
  }
  uint8_t line_length = 19;
  char line_buf[20];
  switch (entry->type) {
  case MENT_LABEL:
    if (entry->label.update_text != NULL && entry->label.update_timer == 0) {
      entry->label.update_text(&entry->label.text);
      entry->label.update_timer = entry->label.update_reload;
    } else if (entry->label.update_text != NULL) {
      entry->label.update_timer--;
    }
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

void free_mentry(MenuEntry *entry) {
  /*if (entry->type == MENT_LABEL && (int)entry->label.owned_text)
    free(entry->label.text);
  if (entry->type == MENT_PRESSABLE && (int)entry->pressable.owned_data)
    free(entry->pressable.data);
  if (entry->type == MENT_CHOICE && (int)entry->choice.owned_choices) {
    for (uint16_t i = 0; i < entry->choice.num_choices; i++) {
      free_mentry(&entry->choice.choices[i]);
    }
    free(entry->choice.choices);
  }*/
  UNUSED(entry);
}

void free_menu(Menu *menu, bool all_prev) {
  for (uint16_t i = 0; i < menu->num_entries; i++) {
    free_mentry(&menu->entries[i]);
  }
  if ((int)all_prev && menu->prev_menu != NULL)
    free_menu(menu->prev_menu, true);
  free(menu);
}

void buttons_menu(Menu *menu) {
  if (this.newly_pressed & BUTTON_DOWN) {
    uint16_t orig = menu->cur_entry;
    while (menu->cur_entry < menu->num_entries - 1) {
      menu->cur_entry++;
      if (menu->entries[menu->cur_entry].is_selectable)
        break;
    }
    if (!menu->entries[menu->cur_entry].is_selectable) {
      menu->cur_entry = orig;
      if (menu->scroll_pos == menu->cur_entry - 1)
        menu->scroll_pos++;
    }
  }
  if (this.newly_pressed & BUTTON_UP) {
    uint16_t orig = menu->cur_entry;
    while (menu->cur_entry > 0) {
      menu->cur_entry--;
      if (menu->entries[menu->cur_entry].is_selectable)
        break;
    }
    if (!menu->entries[menu->cur_entry].is_selectable) {
      menu->cur_entry = orig;
      if (menu->scroll_pos == menu->cur_entry)
        menu->scroll_pos--;
    }
  }
  if (this.newly_pressed & BUTTON_NO && menu->prev_menu != NULL) {
    MenuEntry *chk = &menu->entries[menu->cur_entry];
    bool did = false;
    if (chk->type == MENT_LABEL && (int)chk->label.is_scrollable &&
        chk->label.scroll_pos) {
      did = true;
    }
    if (chk->type == MENT_CHOICE &&
        chk->choice.cur_choice != chk->choice.default_choice) {
      did = true;
    }
    if (chk->type == MENT_VARIABLE) {
      ;
    }
    if (!did) {
      cur_menu = menu->prev_menu;
      free_menu(menu, false);
    }
  }
}
/* Menu buttons
 * NO (if it doesn't do something else) - pop menu stack
 * MENU - push main menu to menu stack. check backwards; delete any menus after
 * the second copy of main menu found.
 *
 */
void render_menu(Menu *menu) {
  buttons_menu(menu);
  menu = cur_menu;
  render_mentry(&menu->entries[menu->scroll_pos], 0,
                menu->cur_entry == menu->scroll_pos);
  if (menu->scroll_pos + 1 < menu->num_entries) {
    render_mentry(&menu->entries[menu->scroll_pos + 1], 1,
                  menu->cur_entry == menu->scroll_pos + 1);
  }
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
  HAL_GPIO_WritePin(GPIO(this.backlight), GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO(this.lcd_reset), GPIO_PIN_SET);
  HAL_Delay(50);
  nhd_init(&this.display);
  init_all_menus();
  cur_menu = nullptr;
  push_new_menu(main_menu);

  while (true) {
    read_buttons();
    // nhd_clear(&this.display);
    // nhd_write_str("HELLO", &this.display);
    render_menu(cur_menu);
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
      HAL_GPIO_WritePin(GPIO(this.backlight), GPIO_PIN_RESET);
    }
    while (!next_tick)
      ;
    next_tick = false;
  }
}
