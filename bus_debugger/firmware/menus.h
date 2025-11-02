#pragma once
#include "bus_debugger.h"

extern Menu *main_menu;
extern Menu *overall_settings_menu;
extern Menu *board_info_menu;

extern const char *controller_type_names[];

void init_all_menus();
void push_new_menu(Menu *menu);
void rescan_can_bus(void *unused);
void update_bus_listing();
void new_label_basic(MenuEntry *ent, char *text, bool is_selectable);
void new_label_dynamic(MenuEntry *ent, void (*func)(char **),
                       bool is_selectable, uint8_t reload);
void new_submenu_press(MenuEntry *ent, char *label, Menu *submenu);
void new_press_func(MenuEntry *ent, char *label, void (*func)(void *),
                    void *argument);
void new_int_var(MenuEntry *ent, char *format, int *loc, int defv, int minv,
                 int maxv, int stepv);
void new_bool_var(MenuEntry *ent, char *format, bool *loc, char *trues,
                  char *falses);
void new_enum_var(MenuEntry *ent, char *format, void *loc, char **keys,
                  uint32_t num_keys, uint32_t defv);
Menu *new_menu(uint16_t num_entries);
