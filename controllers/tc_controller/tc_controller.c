/**
 * @file tc_controller.c
 * @brief Thermocouple Controller Source
 * @author Papyrus Avionics Team
 * @date 2024
 */
#include "tc_controller.h"
#include "commands_sys0.h"
#include "controller_base.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include <stdio.h>
void SystemClock_Config();

TCController this;
CommandRoutine papyrus_cmdrun_table[256];
CommandRoutine papyrus_cmdresp_table[256];
CANMsgLen papyrus_cmd_lens[256];

int main() {
  if (tc_controller_init(&this) != PAPYRUS_OK) {
    Error_Handler();
  }
  load_ctable_sys0(papyrus_cmdrun_table, papyrus_cmdresp_table,
                   papyrus_cmd_lens);
  for (;;) {
    if (papyrus_todo_flags) {
      ErrorEntry last_err;
      papyrus_handle_todos(&this.base, papyrus_todo_flags, &last_err);
      // TODO: parse the error that results
    }
  }
}
