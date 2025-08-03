#pragma once

#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"

typedef enum {
  TCCMD_READ = 0x10,
  TCCMD_STREAM = 0x11,
  TCCMD_STATUS = 0x12,
  TCCMD_FMT = 0x13,
  TCCMD_COUNT = 0x18,
  TCCMD_TYPE = 0x19,
  TCCMD_ALARM = 0x1A
} TCCommand;

COMMAND_PROTO(tc1_read);
COMMAND_PROTO(tc1_stream);
COMMAND_PROTO(tc1_status);
COMMAND_PROTO(tc1_fmt);
COMMAND_PROTO(tc1_count);
COMMAND_PROTO(tc1_type);
COMMAND_PROTO(tc1_alarm);

void load_ctable_tc1(CommandRunner *runs, CommandResponder *resps,
                     CANMsgLen *lens);
