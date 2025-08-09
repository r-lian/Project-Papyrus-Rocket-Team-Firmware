#include "menus.h"
#include "bus_debugger.h"
#include "printf.h"
#include <stdlib.h>
#include <string.h>

Menu *main_menu;
Menu *overall_settings_menu;
Menu *board_settings_menu;
Menu *notify_settings_menu;
Menu *bus_settings_menu;
Menu *memory_settings_menu;
Menu *board_info_menu;

Menu *alloc_copy(Menu *old) {
  Menu *new = malloc(sizeof(Menu));
  while (new == NULL) {
    allocation_failed();
    new = malloc(sizeof(Menu));
  }
  memcpy(new, old, sizeof(Menu));
  new->entries = calloc(new->num_entries, sizeof(MenuEntry));
  while (new->entries == NULL) {
    allocation_failed();
    new->entries = calloc(new->num_entries, sizeof(MenuEntry));
  }
  memcpy(new->entries, old->entries, new->num_entries * sizeof(MenuEntry));
  return new;
}

void push_new_menu(Menu *menu) {
  Menu *prev = cur_menu;
  cur_menu = alloc_copy(menu);
  cur_menu->prev_menu = prev;
}

Menu *new_menu(uint16_t num_entries) {
  Menu *new = malloc(sizeof(Menu));
  while (new == NULL) {
    allocation_failed();
    new = malloc(sizeof(Menu));
  }

  new->cur_entry = 0;
  new->scroll_pos = 0;
  new->sticky_enabled = false;
  new->num_entries = num_entries;
  new->entries = calloc(num_entries, sizeof(MenuEntry));
  while (new->entries == NULL) {
    allocation_failed();
    new->entries = calloc(num_entries, sizeof(MenuEntry));
  }
  return new;
}

void new_label_basic(MenuEntry *ent, char *text, bool is_selectable) {
  ent->type = MENT_LABEL;
  ent->is_selectable = is_selectable;
  ent->label.is_scrollable = true;
  ent->label.scroll_pos = 0;
  ent->label.text = text;
  ent->label.update_text = nullptr;
}
void new_label_dynamic(MenuEntry *ent, void (*func)(char **),
                       bool is_selectable, uint8_t reload) {
  ent->type = MENT_LABEL;
  ent->is_selectable = is_selectable;
  ent->label.is_scrollable = true;
  ent->label.scroll_pos = 0;
  ent->label.text = malloc(20);
  while (ent->label.text == NULL) {
    allocation_failed();
    ent->label.text = malloc(20);
  }
  ent->label.update_text = func;
  ent->label.update_timer = 0;
  ent->label.update_reload = reload;
}
void new_submenu_press(MenuEntry *ent, char *label, Menu *submenu) {
  ent->type = MENT_PRESSABLE;
  ent->is_selectable = true;
  ent->pressable.on_menu = nullptr;
  ent->pressable.on_no = nullptr;
  ent->pressable.on_left = nullptr;
  ent->pressable.on_right = nullptr;
  ent->pressable.on_yes = (void (*)(void *))push_new_menu;
  ent->pressable.text = label;
  ent->pressable.argument = submenu;
}
void new_press_func(MenuEntry *ent, char *label, void (*func)(void *),
                    void *argument) {
  ent->type = MENT_PRESSABLE;
  ent->is_selectable = true;
  ent->pressable.on_menu = nullptr;
  ent->pressable.on_no = nullptr;
  ent->pressable.on_left = nullptr;
  ent->pressable.on_right = nullptr;
  ent->pressable.on_yes = func;
  ent->pressable.text = label;
  ent->pressable.argument = argument;
}

void update_bat_voltage(char **voltage_buf) {
  const char *fmt = "Batt Voltage: %.2fV";
  float voltage = get_bat_voltage();
  sprintf(*voltage_buf, fmt, voltage);
}

void reset_all_settings(void *data) { UNUSED(data); }

void init_all_menus() {
  board_info_menu = new_menu(5);
  new_label_basic(&board_info_menu->entries[0], "-Board Info-", false);
  new_label_basic(&board_info_menu->entries[1], "Board Rev: 1", true);
  new_label_basic(&board_info_menu->entries[2], "Firmware Rev: 1", true);
  new_label_dynamic(&board_info_menu->entries[3], update_bat_voltage, true, 5);
  new_label_basic(&board_info_menu->entries[4], "View Ctrlr Support", true);
  board_info_menu->cur_entry = 1;

  board_settings_menu = new_menu(3);
  new_label_basic(&board_settings_menu->entries[0], "Backlight: ", true);
  new_label_basic(&board_settings_menu->entries[1], "Sound: ", true);
  new_label_basic(&board_settings_menu->entries[2], "Key Repeat: ", true);

  notify_settings_menu = new_menu(1);
  new_label_basic(&notify_settings_menu->entries[0], "Not Implemented", true);

  bus_settings_menu = new_menu(1);
  new_label_basic(&bus_settings_menu->entries[0], "Not Implemented", true);

  memory_settings_menu = new_menu(1);
  new_label_basic(&memory_settings_menu->entries[0], "Not Implemented", true);

  overall_settings_menu = new_menu(6);
  new_label_basic(&overall_settings_menu->entries[0], "-Settings Menu-", false);
  new_submenu_press(&overall_settings_menu->entries[1], "Hieroglyph Settings",
                    board_settings_menu);
  new_submenu_press(&overall_settings_menu->entries[2], "Notify Settings",
                    notify_settings_menu);
  new_submenu_press(&overall_settings_menu->entries[3], "Bus Settings",
                    bus_settings_menu);
  new_submenu_press(&overall_settings_menu->entries[4], "Memory Settings",
                    memory_settings_menu);
  new_press_func(&overall_settings_menu->entries[5], "RESET SETTINGS",
                 reset_all_settings, nullptr);
  overall_settings_menu->cur_entry = 1;

  main_menu = new_menu(4);
  new_label_basic(&main_menu->entries[0], "-Hieroglyph Menu-", false);
  new_submenu_press(&main_menu->entries[1], "Settings", overall_settings_menu);
  new_submenu_press(&main_menu->entries[2], "Board Info", board_info_menu);
  new_press_func(&main_menu->entries[3], "Reset Board", reset_board, nullptr);
  main_menu->cur_entry = 1;
}
