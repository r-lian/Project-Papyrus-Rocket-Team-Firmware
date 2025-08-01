#include "commands_sys0.h"
#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal_gpio.h"

void load_ctable_sys0(CommandRoutine *runs, CommandRoutine *resps,
                      CANMsgLen *lens) {
  runs[0] = &run_sys0_ping;
  resps[0] = &resp_sys0_ping;
  lens[0] = MSGLEN_SHORT;
  runs[1] = &run_sys0_query_type;
  resps[1] = &resp_sys0_query_type;
  lens[1] = MSGLEN_SHORT;
}

uint8_t ping_data_byte;

PapyrusStatus run_sys0_ping(CANMessage *msg, ControllerBase *controller,
                            ErrorEntry *err) {
  if (msg->rHeader.DataLength != 2) {
    err->err = ERROR_COMMAND;
    err->target = 0;
    return PAPYRUS_ERROR_PARAM;
  }
  ping_data_byte = msg->msg.short_data[0];
  if (ping_data_byte == 0xFF) {
    HAL_GPIO_TogglePin(GPIO(controller->status_indicator));
  }
  UNUSED(controller);
  err->err = ERROR_NONE;
  return PAPYRUS_OK;
}
PapyrusStatus resp_sys0_ping(CANMessage *msg, ControllerBase *controller,
                             ErrorEntry *err) {
  papyrus_prep_theader(&msg->tHeader);
  msg->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  msg->tHeader.DataLength = 1;
  msg->msg.short_data[0] = ping_data_byte;
  UNUSED(err);
  return PAPYRUS_OK;
}

PapyrusStatus run_sys0_query_type(CANMessage *msg, ControllerBase *controller,
                                  ErrorEntry *err) {
  if (msg->rHeader.DataLength != 1) {
    err->err = ERROR_COMMAND;
    err->target = 0;
    return PAPYRUS_ERROR_PARAM;
  }
  UNUSED(controller);
  UNUSED(msg);
  err->err = ERROR_NONE;
  return PAPYRUS_OK;
}
PapyrusStatus resp_sys0_query_type(CANMessage *msg, ControllerBase *controller,
                                   ErrorEntry *err) {
  papyrus_prep_theader(&msg->tHeader);
  msg->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  msg->tHeader.DataLength = 4;
  msg->msg.short_data[0] = controller->controller_type;
  msg->msg.short_data[1] = controller->board_revision;
  msg->msg.short_data[2] = controller->firmware_revision;
  msg->msg.short_data[3] = controller->status.num_subdevices;
  UNUSED(err);
  return PAPYRUS_OK;
}

PapyrusStatus gen_sys0_ping(CANMessage *msg, uint8_t data_byte) {
  papyrus_prep_theader(&msg->tHeader);
  msg->tHeader.DataLength = 2;
  msg->msg.command_id = SYSCMD_PING;
  msg->msg.short_args[0] = data_byte;
  return PAPYRUS_OK;
}
PapyrusStatus gen_sys0_query_type(CANMessage *msg) {
  papyrus_prep_theader(&msg->tHeader);
  msg->tHeader.DataLength = 1;
  msg->msg.command_id = SYSCMD_QUERY_TYPE;
  return PAPYRUS_OK;
}
