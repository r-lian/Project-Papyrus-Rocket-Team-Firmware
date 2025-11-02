#include "sys_menus.h"
#include "papyrus_can.h"
#include "printf.h"
#include "stm32c0xx_hal_fdcan.h"
#include <string.h>

Menu *sys_commands_menu;
Menu *ctrlr_info_menu;
Menu *ping_cmd_menu;
Menu *reassign_cmd_menu;
Menu *raw_io_menu;
BusDevice *bus_target;
void (*on_response)(uint8_t *data);
int temp_data_byte;
/*
void send_command_qs(void *quicksend_void) {
  QuickSendDef *quicksend = quicksend_void;
  FDCAN_TxHeaderTypeDef tHeader = {0};
  tHeader.IdType = FDCAN_STANDARD_ID;
  tHeader.TxFrameType = FDCAN_DATA_FRAME;
  tHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  tHeader.BitRateSwitch = FDCAN_BRS_OFF;
  tHeader.FDFormat = FDCAN_CLASSIC_CAN;
  tHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tHeader.MessageMarker = 0;
  tHeader.DataLength = quicksend->len;
  tHeader.Identifier = CAN_GENERATE_ID(bus_target->board_id, MSG_TYPE_COMMAND);
  HAL_FDCAN_AddMessageToTxFifoQ(&this.can.handle, &tHeader, &quicksend->msg[0]);
  on_response = quicksend->on_response;
  num_response_stages = quicksend->num_stages;
  this.resp_mode = RESPONSE_DIRECT;
}*/
void send_command(uint8_t data_len, uint8_t *data) {
  FDCAN_TxHeaderTypeDef tHeader = {0};
  tHeader.IdType = FDCAN_STANDARD_ID;
  tHeader.TxFrameType = FDCAN_DATA_FRAME;
  tHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  tHeader.BitRateSwitch = FDCAN_BRS_OFF;
  tHeader.FDFormat = FDCAN_CLASSIC_CAN;
  tHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tHeader.MessageMarker = 0;
  tHeader.DataLength = data_len;
  tHeader.Identifier = CAN_GENERATE_ID(bus_target->board_id, MSG_TYPE_COMMAND);
  HAL_FDCAN_AddMessageToTxFifoQ(&this.can.handle, &tHeader, data);
}
void ack_formatter(uint8_t *data) {
  push_sticky_notif("Acknowledged!");
  UNUSED(data);
}
void ping_formatter(uint8_t *data) {
  char buf[20];
  sprintf(buf, "Received ping (%02X)!", data[0]);
  push_sticky_notif(buf);
}
void fmt_ctrl_name(char **fmt) {
  sprintf(*fmt, "%2d: %s", bus_target->board_id,
          controller_type_names[bus_target->kind]);
}
void fmt_ctrl_rev(char **fmt) {
  sprintf(*fmt, "HWRev.%d   FWRev.%d", bus_target->board_revision,
          bus_target->firmware_revision);
}
void send_cmd_ping(void *unused) {
  UNUSED(unused);
  uint8_t buf[2];
  buf[0] = '\x00';
  buf[1] = temp_data_byte;
  send_command(2, buf);
  this.resp_mode = RESPONSE_DIRECT;
  on_response = ping_formatter;
}
void send_reassign_id(uint8_t button) {
  if (button != BUTTON_YES)
    return;
  uint8_t buf[2];
  buf[0] = '\x02';
  buf[1] = temp_data_byte;
  send_command(2, buf);
  bus_target->board_id = temp_data_byte;
  this.resp_mode = RESPONSE_DIRECT;
  on_response = ack_formatter;
}
void try_reassign_id(void *unused) {
  UNUSED(unused);
  on_notif_dismiss = send_reassign_id;
  push_sticky_notif("Are you sure?");
}
void send_reset(uint8_t button) {
  if (button != BUTTON_YES)
    return;
  uint8_t buf;
  buf = '\x04';
  send_command(1, &buf);
  this.resp_mode = RESPONSE_DIRECT;
  on_response = ack_formatter;
}
void try_reset(void *unused) {
  UNUSED(unused);
  on_notif_dismiss = send_reset;
  push_sticky_notif("Are you sure?");
}
void init_system_menu() {
  ping_cmd_menu = new_menu(2);
  new_int_var(&ping_cmd_menu->entries[0], "Data byte: %02X", &temp_data_byte, 1,
              0, 255, 1);
  new_press_func(&ping_cmd_menu->entries[1], "Send Ping", send_cmd_ping,
                 nullptr);

  reassign_cmd_menu = new_menu(2);
  new_int_var(&reassign_cmd_menu->entries[0], "New ID: %d", &temp_data_byte, 1,
              1, 127, 1);
  new_press_func(&reassign_cmd_menu->entries[1], "Reassign ID", try_reassign_id,
                 nullptr);

  sys_commands_menu = new_menu(7);
  new_label_dynamic(&sys_commands_menu->entries[0], fmt_ctrl_name, true, 40);
  new_label_dynamic(&sys_commands_menu->entries[1], fmt_ctrl_rev, true, 40);
  new_submenu_press(&sys_commands_menu->entries[2], "Ping", ping_cmd_menu);
  new_submenu_press(&sys_commands_menu->entries[3], "Reassign ID",
                    reassign_cmd_menu);
  new_press_func(&sys_commands_menu->entries[4], "Reset", try_reset, nullptr);
  new_label_basic(&sys_commands_menu->entries[5], "Notif. Config", true);
  new_label_basic(&sys_commands_menu->entries[6], "Error Config", true);

  raw_io_menu = new_menu(1);
  new_label_basic(&raw_io_menu->entries[0], "todo", true);
  /*sys_commands_menu = new_menu(2);
  new_press_func(&sys_commands_menu->entries[0], "SYS/PING", send_command_qs,
                 (void *)&sysping_qs);
  new_press_func(&sys_commands_menu->entries[1], "SYS/QUERY_TYPE",
                 send_command_qs, (void *)&sysquerytype_qs);

  ctrlr_info_menu = new_menu(3);
  new_label_dynamic(&ctrlr_info_menu->entries[0], fmt_ctrl_name, true, 40);
  new_label_dynamic(&ctrlr_info_menu->entries[1], fmt_ctrl_brev, true, 40);
  new_label_dynamic(&ctrlr_info_menu->entries[2], fmt_ctrl_frev, true, 40);*/
}
