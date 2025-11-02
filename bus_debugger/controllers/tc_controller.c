#include "tc_controller.h"
#include "menus.h"
#include "printf.h"
#include "sys_menus.h"
Menu *tc_controller_menu;
Menu *tc_commands_menu;
int num_thermocouples;

void init_tc_controller_menu() {

  tc_commands_menu = new_menu(1);
  // new_label_basic(&tc_commands_menu->entries[0], "todo", true);

  tc_controller_menu = new_menu(3);
  new_submenu_press(&tc_controller_menu->entries[0], "System",
                    sys_commands_menu);
  new_submenu_press(&tc_controller_menu->entries[1], "Thermocouples",
                    tc_commands_menu);
  new_submenu_press(&tc_controller_menu->entries[2], "Raw I/O", raw_io_menu);
  tc_controller_menu->special = SPECIAL_CONTROLLER;
}
