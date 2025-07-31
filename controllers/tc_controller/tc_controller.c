/**
 * @file tc_controller.c
 * @brief Thermocouple Controller Source
 * @author Papyrus Avionics Team
 * @date 2024
 */
#include "tc_controller.h"
#include "commands_sys0.h"
#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

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
  setvbuf(stdin, nullptr, _IONBF, 0);
  __io_getchar();

  for (;;) {
    printf("HElloworld\r\n");
    HAL_GPIO_TogglePin(GPIO(TC_STATUS_LED));
    HAL_Delay(500);
  }
}
