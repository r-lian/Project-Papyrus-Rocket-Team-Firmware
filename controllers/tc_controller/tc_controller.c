/**
 * @file tc_controller.c
 * @brief Thermocouple Controller Source
 * @author Papyrus Avionics Team
 * @date 2024
 */
#include "tc_controller.h"
#include "commands_sys0.h"
#include "commands_tc1.h"
#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include "tc_amplifier_max31855.h"
#include <string.h>

TCController this;
CommandRunner papyrus_cmdrun_table[256];
CommandResponder papyrus_cmdresp_table[256];
CANMsgLen papyrus_cmd_lens[256];

extern void uart_debugger(TCController *this);

int32_t to_tc_format(TCTempRead internal, TCReadFormat target) {
  switch (target) {
  case TC_FMT_INT8_SHIFT:
    return internal >> 20;
  case TC_FMT_INT16:
    return internal >> 16;
  case TC_FMT_FIXED88:
    return (internal >> 8) & 0xFFFF;
  case TC_FMT_FIXED1616:
    return internal;
  }
  return 0;
}
TCTempRead from_tc_format(int32_t external, TCReadFormat target) {
  switch (target) {
  case TC_FMT_INT8_SHIFT:
    return external << 20;
  case TC_FMT_INT16:
    return external << 16;
  case TC_FMT_FIXED88:
    return external << 8;
  case TC_FMT_FIXED1616:
    return external;
  }
  return 0;
}

int main() {
  if (tc_controller_init(&this) != PAPYRUS_OK) {
    Error_Handler();
  }
  load_ctable_sys0(papyrus_cmdrun_table, papyrus_cmdresp_table,
                   papyrus_cmd_lens);
  load_ctable_tc1(papyrus_cmdrun_table, papyrus_cmdresp_table,
                  papyrus_cmd_lens);
  uart_debugger(&this);
  /*
    CANMessage prepmsg;
    papyrus_prep_theader(&prepmsg.tHeader);
    prepmsg.msg.command_id = TCCMD_STREAM;
    prepmsg.tHeader.DataLength = 4;
    prepmsg.tHeader.Identifier = CAN_GENERATE_ID(1, MSG_TYPE_COMMAND);
    prepmsg.msg.short_args[0] = 0x01;
    prepmsg.msg.short_args[1] = 0xe8;
    prepmsg.msg.short_args[2] = 0x03;
    HAL_FDCAN_AddMessageToTxFifoQ(&(this.base.can.handle), &prepmsg.tHeader,
                                  prepmsg.msg.raw_data);
    for (;;) {
      HAL_Delay(500);
    }
    */
}
