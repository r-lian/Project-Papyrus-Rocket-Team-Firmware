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
#include "papyrus_utils.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include "tc_amplifier_max31855.h"
#include <stdio.h>
#include <string.h>

TCController this;
CommandRoutine papyrus_cmdrun_table[256];
CommandRoutine papyrus_cmdresp_table[256];
CANMsgLen papyrus_cmd_lens[256];

extern void uart_debugger(TCController *this);

int main() {
  if (tc_controller_init(&this) != PAPYRUS_OK) {
    Error_Handler();
  }
  // load_ctable_sys0(papyrus_cmdrun_table, papyrus_cmdresp_table,
  //                  papyrus_cmd_lens);
  uart_debugger(&this);
  /*
  MAX31855Output tmp;
  for (;;) {
    if (max31855_read_tc(&this.tc_spis[0], &tmp) != PAPYRUS_OK) {
      printf("TC read error: ");
      if (tmp.err_flags & TC_FAULT_OPEN)
        printf("OPEN ");
      if (tmp.err_flags & TC_FAULT_SHORT_GND)
        printf("SHORT_GND ");
      if (tmp.err_flags & TC_FAULT_SHORT_VCC)
        printf("SHORT_VCC ");
      if (tmp.err_flags & TC_FAULT_SPI_FAILED) {
        if (tmp.last_hw_err == HAL_BUSY) {
          printf("SPI_FAILED(BUSY) ");
        } else if (tmp.last_hw_err == HAL_ERROR) {
          printf("SPI_FAILED(ERROR) ");
        } else if (tmp.last_hw_err == HAL_TIMEOUT) {
          printf("SPI_FAILED(TIMEOUT) ");
        } else {
          printf("SPI_FAILED(");
          printf("%02x%02x%02x%02x) ", tmp.read_bytes[0], tmp.read_bytes[1],
                 tmp.read_bytes[2], tmp.read_bytes[3]);
        }
      }
      printf("\r\n");
    } else {
      printf("TC read results: tc=%.2f cjc=%.2f\r\n",
             fixed_to_float(tmp.tc_fixed), fixed_to_float(tmp.cjc_fixed));
    }
    HAL_Delay(400);
  }*/
}
