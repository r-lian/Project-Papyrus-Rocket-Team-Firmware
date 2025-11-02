#include "menus.h"
#include "bus_debugger.h"
#include "papyrus_can.h"
#include "printf.h"
#include "stm32c0xx_hal_fdcan.h"
#include "sys_menus.h"
#include "tc_controller.h"
#include <stdlib.h>
#include <string.h>

Menu *main_menu;
Menu *overall_settings_menu;
Menu *board_settings_menu;
Menu *notify_settings_menu;
Menu *bus_settings_menu;
Menu *supported_controllers_menu;
Menu *memory_settings_menu;
Menu *board_info_menu;
Menu *can_bus_menu;

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

void new_int_var(MenuEntry *ent, char *format, int *loc, int defv, int minv,
                 int maxv, int stepv) {
  ent->type = MENT_VARIABLE;
  ent->is_selectable = true;
  ent->variable.kind = MVAR_INT;
  ent->variable.format = format;
  ent->variable.default_int = defv;
  ent->variable.value = loc;
  ent->variable.int_min = minv;
  ent->variable.int_max = maxv;
  ent->variable.int_step = stepv;
}
void new_bool_var(MenuEntry *ent, char *format, bool *loc, char *trues,
                  char *falses) {
  ent->type = MENT_VARIABLE;
  ent->is_selectable = true;
  ent->variable.kind = MVAR_BOOL;
  ent->variable.format = format;
  ent->variable.value = loc;
  ent->variable.true_str = trues;
  ent->variable.false_str = falses;
}
void new_enum_var(MenuEntry *ent, char *format, void *loc, char **keys,
                  uint32_t num_keys, uint32_t defv) {
  ent->type = MENT_VARIABLE;
  ent->is_selectable = true;
  ent->variable.kind = MVAR_ENUM;
  ent->variable.format = format;
  ent->variable.value = loc;
  ent->variable.default_enum = defv;
  ent->variable.enum_options = keys;
  ent->variable.num_options = num_keys;
  ent->variable.do_wrap = false;
}

void pop_menu_stack(void *unused) {
  Menu *menu = cur_menu;
  UNUSED(unused);
  cur_menu = menu->prev_menu;
  free_menu(menu, false);
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
  new->special = SPECIAL_NONE;
  new->num_entries = num_entries + 1;
  new->entries = calloc(num_entries + 1, sizeof(MenuEntry));
  while (new->entries == NULL) {
    allocation_failed();
    new->entries = calloc(num_entries + 1, sizeof(MenuEntry));
  }
  new_press_func(&new->entries[num_entries], "(Back)", pop_menu_stack, nullptr);
  return new;
}

void update_bat_voltage(char **voltage_buf) {
  const char *fmt = "Batt Voltage: %.2fV";
  float voltage = get_bat_voltage();
  if (voltage < 3.3) {
    sprintf(*voltage_buf, "Batt Voltage: N/A");
  } else {
    sprintf(*voltage_buf, fmt, voltage);
  }
}
void update_can_label(char **can_buf) {
  if (this.resp_mode == RESPONSE_SCANNING) {
    sprintf(*can_buf, "-Bus / scanning...-");
  } else {
    sprintf(*can_buf, "-Bus / %3d devices-", num_bus_devices);
  }
}

void reset_all_settings(void *data) { UNUSED(data); }
void rescan_can_bus(void *unused) {
  UNUSED(unused);
  FDCAN_TxHeaderTypeDef tHeader = {0};
  tHeader.IdType = FDCAN_STANDARD_ID;
  tHeader.TxFrameType = FDCAN_DATA_FRAME;
  tHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  tHeader.BitRateSwitch = FDCAN_BRS_OFF;
  tHeader.FDFormat = FDCAN_CLASSIC_CAN;
  tHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tHeader.MessageMarker = 0;
  tHeader.DataLength = 1;
  uint8_t data[2] = {0x1};
  HAL_FDCAN_AddMessageToTxFifoQ(&this.can.handle, &tHeader, data);
  num_bus_devices = 0;
  bus_devices = realloc(bus_devices, 0);
  this.resp_mode = RESPONSE_SCANNING;
  scan_timeout = 40;
}

int cansort_key_recent(const void *d1, const void *d2) {
  return ((BusDevice *)d1)->last_used - ((BusDevice *)d2)->last_used;
}
int cansort_key_id(const void *d1, const void *d2) {
  return ((BusDevice *)d1)->board_id - ((BusDevice *)d2)->board_id;
}
int cansort_key_type(const void *d1, const void *d2) {
  return ((BusDevice *)d1)->kind - ((BusDevice *)d2)->kind;
}

int (*cansort_funcs[])(const void *, const void *) = {
    cansort_key_recent, cansort_key_id, cansort_key_type};

const char *controller_type_names[] = {"Servo", "Thermocouple", "Unknown"};
Menu *controller_submenus[3];
void load_controller_submenus() {
  controller_submenus[CONTROLLER_TYPE_THERMOCOUPLE] = tc_controller_menu;
}
void enter_controller_menu(void *device) {
  bus_target = device;
  push_new_menu(controller_submenus[bus_target->kind]);
}
void update_bus_listing() {
  qsort(bus_devices, num_bus_devices, sizeof(BusDevice),
        cansort_funcs[this.settings.cansort]);
  can_bus_menu->num_entries = 5 + num_bus_devices;
  can_bus_menu->entries = realloc(
      can_bus_menu->entries, sizeof(MenuEntry) * can_bus_menu->num_entries);
  for (uint32_t i = 0; i < num_bus_devices; i++) {
    sprintf(bus_devices[i].name_buf, "%2d:%s v%d", bus_devices[i].board_id,
            controller_type_names[bus_devices[i].kind],
            bus_devices[i].board_revision);
    new_press_func(&can_bus_menu->entries[4 + i], bus_devices[i].name_buf,
                   enter_controller_menu, &bus_devices[i]);
  }
  new_press_func(&can_bus_menu->entries[4 + num_bus_devices], "(Back)",
                 pop_menu_stack, nullptr);
  if (cur_menu->special == SPECIAL_CANLIST) {
    uint16_t cur = cur_menu->cur_entry;
    uint16_t pos = cur_menu->scroll_pos;
    pop_menu_stack(nullptr);
    push_new_menu(can_bus_menu);
    cur_menu->cur_entry = cur;
    cur_menu->scroll_pos = pos;
  }
}

const char *backlight_keys[] = {"ON", "SLEEP", "OFF"};
const char *cansort_keys[] = {"Recent", "ID", "Type"};

void init_all_menus() {
  supported_controllers_menu = new_menu(2);
  new_label_basic(&supported_controllers_menu->entries[0], "-Supported Ctrlrs-",
                  false);
  new_label_basic(&supported_controllers_menu->entries[1], "Thermocouple v1",
                  true);
  supported_controllers_menu->cur_entry = 1;

  board_info_menu = new_menu(5);
  new_label_basic(&board_info_menu->entries[0], "-Board Info-", false);
  new_label_basic(&board_info_menu->entries[1], "Board Rev: 1", true);
  new_label_basic(&board_info_menu->entries[2], "Firmware Rev: 1", true);
  new_label_dynamic(&board_info_menu->entries[3], update_bat_voltage, true, 5);
  new_submenu_press(&board_info_menu->entries[4], "View Ctrlr Support",
                    supported_controllers_menu);
  board_info_menu->cur_entry = 1;

  board_settings_menu = new_menu(3);
  // new_label_basic(&board_settings_menu->entries[0], "Backlight: ", true);
  new_enum_var(&board_settings_menu->entries[0], "Backlight: %s",
               &this.settings.backlight, (char **)backlight_keys, 3, 2);
  new_bool_var(&board_settings_menu->entries[1], "Sound: %s",
               &this.settings.sound, "ON", "OFF");
  new_bool_var(&board_settings_menu->entries[2], "Key Repeat: %s",
               &this.settings.keyrepeat, "ON", "OFF");

  // 0123456789012345678
  // Key Repeat: OFF

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

  // 0123456789012345678
  // -Bus / %%% devices-

  can_bus_menu = new_menu(4);
  new_label_dynamic(&can_bus_menu->entries[0], update_can_label, false, 5);
  new_press_func(&can_bus_menu->entries[1], "Rescan Bus", rescan_can_bus,
                 nullptr);
  new_enum_var(&can_bus_menu->entries[2], "Sort: %s", &this.settings.cansort,
               (char **)cansort_keys, 3, 0);
  new_label_basic(&can_bus_menu->entries[3], "-------------------", false);
  can_bus_menu->cur_entry = 1;
  can_bus_menu->special = SPECIAL_CANLIST;

  main_menu = new_menu(5);
  new_label_basic(&main_menu->entries[0], "-Hieroglyph Menu-", false);
  new_submenu_press(&main_menu->entries[1], "CAN Bus", can_bus_menu);
  new_submenu_press(&main_menu->entries[2], "Settings", overall_settings_menu);
  new_submenu_press(&main_menu->entries[3], "Board Info", board_info_menu);
  new_press_func(&main_menu->entries[4], "Reset Board", reset_board, nullptr);
  main_menu->cur_entry = 1;
  main_menu->special = SPECIAL_MAINMENU;
  main_menu->num_entries--;

  init_system_menu();
  init_tc_controller_menu();
  load_controller_submenus();
}
