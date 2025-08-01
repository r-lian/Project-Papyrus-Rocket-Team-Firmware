

#ifndef COMMANDS_SYS0_H
#define COMMANDS_SYS0_H

#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"
/* System Commands (Block 0)*/
typedef enum {
  SYSCMD_PING = 0x00,
  SYSCMD_QUERY_TYPE = 0x01,
  SYSCMD_REASSIGN_ID = 0x02,
  SYSCMD_RAW_IO = 0x03,
  SYSCMD_RESET = 0x04,
  SYSCMD_NAME = 0x05,
  SYSCMD_NOTIFICATION = 0x06,
  SYSCMD_SUPPORTED = 0x07,
  SYSCMD_GET_ERROR = 0x08,
  SYSCMD_STATE = 0x09,
} SystemCommand;

/* Command Constructors & Executors (Block 0) */

COMMAND_PROTO(sys0_ping);
PapyrusStatus gen_sys0_ping(CANMessage *msg, uint8_t data_byte);
COMMAND_PROTO(sys0_query_type);
PapyrusStatus gen_sys0_query_type(CANMessage *msg);

void load_ctable_sys0(CommandRoutine *runs, CommandRoutine *resps,
                      CANMsgLen *lens);
#endif
