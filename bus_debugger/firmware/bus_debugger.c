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
#include "printf.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_adc.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include "sys_menus.h"
#include <stdlib.h>
#include <string.h>

BusDebugger this;

UART_HandleTypeDef *stdio_uart;

int is_onebit(uint8_t x) { return (x != 0) && (x & (x - 1)) == 0; }

Menu *cur_menu;
uint32_t num_bus_devices;
BusDevice *bus_devices;

MenuEntry sticky_notifs[16];
uint8_t num_sticky_notifs;
void (*on_notif_dismiss)(uint8_t button);

void allocation_failed() { Error_Handler(); }

extern void Reset_Handler();

void reset_board(void *unused) {
  UNUSED(unused);
  NVIC_SystemReset();
}

uint32_t beep_timer = 0;
void play_beep(uint8_t kind, uint32_t length) {
  if (!this.settings.sound)
    return;
  update_prescaler(kind);
  if (!beep_timer)
    start_pwm();
  beep_timer = length;
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
  if ((int)this.settings.keyrepeat && is_onebit(this.buttons_pressed) &&
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
bool cursor_flash() {
  if (num_sticky_notifs)
    return false;
  return (bool)!((HAL_GetTick() & 1023) > 512);
}
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
      play_beep(1, 5);
    }
    if (entry->pressable.on_right != NULL &&
        this.newly_pressed & BUTTON_RIGHT) {
      entry->pressable.on_right(entry->pressable.argument);
      play_beep(1, 5);
    }
    if (entry->pressable.on_yes != NULL && this.newly_pressed & BUTTON_YES) {
      entry->pressable.on_yes(entry->pressable.argument);
      play_beep(1, 5);
    }
    if (entry->pressable.on_no != NULL && this.newly_pressed & BUTTON_NO) {
      entry->pressable.on_no(entry->pressable.argument);
      play_beep(1, 5);
    }
    if (entry->pressable.on_menu != NULL && this.newly_pressed & BUTTON_MENU) {
      entry->pressable.on_menu(entry->pressable.argument);
      play_beep(1, 5);
    }
    break;
  case MENT_VARIABLE:
    if (this.newly_pressed & (BUTTON_RIGHT | BUTTON_LEFT | BUTTON_NO))
      play_beep(1, 3);
    switch (entry->variable.kind) {
    case MVAR_INT:

      if (this.newly_pressed & BUTTON_RIGHT)
        *(int *)entry->variable.value += entry->variable.int_step;
      if (this.newly_pressed & BUTTON_LEFT)
        *(int *)entry->variable.value -= entry->variable.int_step;
      *(int *)entry->variable.value =
          CLAMP(*(int *)entry->variable.value, entry->variable.int_min,
                entry->variable.int_max);
      if (this.newly_pressed & BUTTON_NO)
        *(int *)entry->variable.value = entry->variable.default_int;
      break;
    case MVAR_ENUM:
      if (this.newly_pressed & BUTTON_RIGHT)
        (*(uint32_t *)entry->variable.value)++;
      if (this.newly_pressed & BUTTON_LEFT)
        (*(uint32_t *)entry->variable.value)--;
      if (*(uint32_t *)entry->variable.value == (uint32_t)-1) {
        *(uint32_t *)entry->variable.value =
            (int)entry->variable.do_wrap ? entry->variable.num_options - 1 : 0;
      }
      if (*(uint32_t *)entry->variable.value >= entry->variable.num_options) {
        *(uint32_t *)entry->variable.value =
            (int)entry->variable.do_wrap ? 0 : entry->variable.num_options - 1;
      }
      if (this.newly_pressed & BUTTON_NO)
        *(uint32_t *)entry->variable.value = entry->variable.default_enum;
      break;
    case MVAR_BOOL:
      if (this.newly_pressed & (BUTTON_LEFT | BUTTON_RIGHT))
        (*(bool *)entry->variable.value) =
            (bool)!(*(bool *)entry->variable.value);

      if (this.newly_pressed & (BUTTON_YES))
        (*(bool *)entry->variable.value) = true;

      if (this.newly_pressed & (BUTTON_NO))
        (*(bool *)entry->variable.value) = false;

      break;
    default:
      break;
    }
  default:;
  }
}

float get_bat_voltage() {
  HAL_ADC_Start(&this.batread.handle);
  HAL_ADC_PollForConversion(&this.batread.handle, HAL_MAX_DELAY);
  uint32_t raw = HAL_ADC_GetValue(&this.batread.handle);
  // return (fixed32)(((uint64_t)raw) * 0xe30a3);
  return 14.19 * ((float)raw / 4096.0) + 0.3;
}
void render_mentry(MenuEntry *entry, uint8_t row, bool is_selected,
                   bool ignore_sticky) {
  if (!ignore_sticky && !is_selected &&
      ((int)cur_menu->sticky_enabled || num_sticky_notifs)) {
    if (num_sticky_notifs) {
      render_mentry(&sticky_notifs[num_sticky_notifs - 1], row, false, true);
    } else {
      render_mentry(&cur_menu->sticky, row, false, true);
    }
    return;
  }
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
                  is_selected, false);
    break;
  case MENT_VARIABLE:
    switch (entry->variable.kind) {
    case MVAR_BOOL:
      sprintf(line_buf, entry->variable.format,
              (int)*(bool *)entry->variable.value ? entry->variable.true_str
                                                  : entry->variable.false_str);
      break;
    case MVAR_INT:
      sprintf(line_buf, entry->variable.format, *(int *)entry->variable.value);
      break;
    case MVAR_FLOAT:
      sprintf(line_buf, entry->variable.format,
              *(float *)entry->variable.value);
      break;
    case MVAR_ENUM:
      sprintf(line_buf, entry->variable.format,
              entry->variable.enum_options[*(int *)entry->variable.value]);
      break;
    }
    memset(line_buf + strlen(line_buf), ' ', line_length - strlen(line_buf));
    nhd_write_str(line_buf, &this.display);
    break;
  default:;
  }
  if (is_selected)
    buttons_mentry(entry);
}
// 01234567890123456789
// Backlight: AUTO-SLEEP
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
    play_beep(3, 7);
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
      if (chk->variable.kind == MVAR_INT &&
          *(int *)chk->variable.value != chk->variable.default_int)
        did = true;
      if (chk->variable.kind == MVAR_BOOL)
        did = true;
      if (chk->variable.kind == MVAR_FLOAT &&
          *(float *)chk->variable.value != chk->variable.default_float)
        did = true;
      if (chk->variable.kind == MVAR_ENUM &&
          *(uint32_t *)chk->variable.value != chk->variable.default_enum)
        did = true;
    }
    if (!did) {
      cur_menu = menu->prev_menu;
      free_menu(menu, false);
    }
  }
  if (this.newly_pressed & BUTTON_MENU) {
    play_beep(2, 7);
    push_new_menu(main_menu);
    bool found_once = false;
    Menu *ptr = main_menu->prev_menu;
    Menu **tonull = nullptr;
    while (ptr != NULL) {
      if (ptr->special == SPECIAL_MAINMENU) {
        if (found_once)
          break;
        found_once = true;
      }
      tonull = &ptr->prev_menu;
      ptr = ptr->prev_menu;
    }
    if (ptr != NULL) {
      *tonull = nullptr;
      free_menu(ptr, true);
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
                menu->cur_entry == menu->scroll_pos, false);
  if (menu->scroll_pos + 1 < menu->num_entries) {
    render_mentry(&menu->entries[menu->scroll_pos + 1], 1,
                  menu->cur_entry == menu->scroll_pos + 1, false);
  }
  if (menu->scroll_pos < menu->cur_entry - 1) {
    menu->scroll_pos = menu->cur_entry - 1;
  } else if (menu->cur_entry < menu->scroll_pos) {
    menu->scroll_pos = menu->cur_entry;
  }
}
uint8_t lastRxData[8];
FDCAN_RxHeaderTypeDef lastRxHeader;

void push_sticky_notif(char *text) {
  sticky_notifs[num_sticky_notifs].type = MENT_LABEL;
  strncpy(sticky_notifs[num_sticky_notifs].label.text, text, 19);
  num_sticky_notifs++;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &lastRxHeader, lastRxData);
    if (this.resp_mode == RESPONSE_SCANNING &&
        CAN_MESSAGE_TYPE(lastRxHeader.Identifier) == MSG_TYPE_RESPONSE) {
      num_bus_devices++;
      bus_devices = realloc(bus_devices, sizeof(BusDevice) * num_bus_devices);
      bus_devices[num_bus_devices - 1].board_id =
          CAN_CONTROLLER_ID(lastRxHeader.Identifier);
      bus_devices[num_bus_devices - 1].kind = lastRxData[0];
      bus_devices[num_bus_devices - 1].board_revision = lastRxData[1];
      bus_devices[num_bus_devices - 1].firmware_revision = lastRxData[2];
    } else if (this.resp_mode == RESPONSE_DIRECT) {
      on_response(lastRxData);
    } else {
      ;
    }
  }
}

bool need_reinit = false;
uint32_t scan_timeout = 0;
int main() {
  if (bus_debugger_init(&this) != PAPYRUS_OK) {
    Error_Handler();
  }
  this.backlight_sleep = 0;
  this.settings.backlight = BACKLIGHT_OFF;
  this.settings.sound = true;
  this.settings.keyrepeat = true;
  num_bus_devices = 0;
  on_notif_dismiss = nullptr;
  for (uint8_t i = 0; i < 16; i++) {
    sticky_notifs[i].label.text = malloc(19);
  }
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIO(this.backlight), GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO(this.lcd_reset), GPIO_PIN_SET);
  HAL_Delay(50);
  nhd_init(&this.display);
  init_all_menus();
  cur_menu = nullptr;
  push_new_menu(main_menu);

  num_bus_devices = 0;
  bus_devices = malloc(sizeof(BusDevice));

  update_prescaler(0);

  this.resp_mode = RESPONSE_IGNORE;

  play_beep(0, 12);

  rescan_can_bus(nullptr);

  while (true) {
    read_buttons();
    if ((this.newly_pressed & (BUTTON_YES | BUTTON_NO)) && num_sticky_notifs) {
      if (on_notif_dismiss != NULL)
        on_notif_dismiss(this.newly_pressed);
      on_notif_dismiss = nullptr;
      this.newly_pressed = 0;
      num_sticky_notifs--;
      play_beep(1, 5);
    }
    // nhd_clear(&this.display);
    // nhd_write_str("HELLO", &this.display);
    render_menu(cur_menu);
    if (this.buttons_pressed)
      this.backlight_sleep = 0;
    this.backlight_sleep++;
    if (this.backlight_sleep > 1500) {
      HAL_GPIO_WritePin(GPIO(this.backlight),
                        (this.settings.backlight != BACKLIGHT_ON)
                            ? GPIO_PIN_RESET
                            : GPIO_PIN_SET);
      if (this.backlight_sleep > 3000) {
        need_reinit = true;
        nhd_shutdown(&this.display);
      }
    } else {
      if (need_reinit) {
        need_reinit = false;
        nhd_init(&this.display);
      }
      HAL_GPIO_WritePin(GPIO(this.backlight),
                        (this.settings.backlight == BACKLIGHT_OFF)
                            ? GPIO_PIN_RESET
                            : GPIO_PIN_SET);
    }
    if (scan_timeout) {
      scan_timeout--;
      if (scan_timeout == 0) {
        this.resp_mode = RESPONSE_IGNORE;
        update_bus_listing();
      }
    }
    if (beep_timer) {
      beep_timer--;
      if (beep_timer == 0)
        stop_pwm();
    }
    while (!next_tick)
      ;
    next_tick = false;
  }
}
