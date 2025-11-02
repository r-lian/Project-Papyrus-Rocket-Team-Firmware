#pragma once
#include "bus_debugger.h"
#include "menus.h"

extern Menu *sys_commands_menu;
extern Menu *raw_io_menu;
extern BusDevice *bus_target;
extern void (*on_response)(uint8_t *data);

void init_system_menu();
void ack_formatter(uint8_t *data);
