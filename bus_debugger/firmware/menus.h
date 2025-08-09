#pragma once
#include "bus_debugger.h"

extern Menu *main_menu;
extern Menu *overall_settings_menu;
extern Menu *board_info_menu;

void init_all_menus();
void push_new_menu(Menu *menu);
