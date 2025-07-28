#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_gpio.h"
#include <stdlib.h>

uint32_t papyrus_todo_flags = 0;
PapyrusStatus controller_base_init(ControllerBase *controller) {
  controller->status.state = CONTROLLER_STATE_INIT;
  controller->status.error_queue_len = 0;
  controller->status.persistent_errors_len = 0;
  controller->status.statistics.board_errors = 0;
  controller->status.statistics.can_errors = 0;
  controller->status.statistics.can_messages_received = 0;
  controller->status.statistics.can_messages_sent = 0;
  controller->status.statistics.notifications_sent = 0;
  controller->status.statistics.timestamp_start = HAL_GetTick();
  controller->num_transactions = 0;
  for (int i = 0; i < 16; i++) {
    controller->status.subdev_state[i] = SUBDEV_STATE_ABSENT;
  }
  return PAPYRUS_OK;
}
PapyrusStatus controller_hardware_init(ControllerBase *controller) {
  if (HAL_Init() != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }

  // Initialize CAN
  // TODO

  // Initialize UART
  if (controller->uart.enabled) {
    controller->uart.handle.Instance = controller->uart.instance;
    controller->uart.handle.Init.BaudRate = controller->uart.uart_baudrate;
    controller->uart.handle.Init.WordLength = UART_WORDLENGTH_8B;
    controller->uart.handle.Init.StopBits = UART_STOPBITS_1;
    controller->uart.handle.Init.Parity = UART_PARITY_NONE;
    controller->uart.handle.Init.Mode = UART_MODE_TX_RX;
    controller->uart.handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    controller->uart.handle.Init.OverSampling = UART_OVERSAMPLING_16;
    controller->uart.handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    controller->uart.handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    controller->uart.handle.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&controller->uart.handle) != HAL_OK) {
      return PAPYRUS_ERROR_HARDWARE;
    }
  }

  // Initialize LEDs
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = controller->status_indicator.pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(controller->status_indicator.grp, &gpio_init);
  gpio_init.Pin = controller->fault_indicator.pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(controller->fault_indicator.grp, &gpio_init);
  return PAPYRUS_OK;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  UNUSED(hfdcan);
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
    papyrus_todo_flags |= PAPYRUS_TODO_CAN_RXFIFO;
  }
}

void papyrus_handle_todos(ControllerBase *base, uint32_t flags,
                          ErrorEntry *err) {
  if (flags & PAPYRUS_TODO_CAN_RXFIFO) {
    CANMessage newest;
    if (HAL_FDCAN_GetRxMessage(&base->can.handle, FDCAN_RX_FIFO0,
                               &newest.rHeader,
                               newest.msg.raw_data) != HAL_OK) {
      err->err = ERROR_CAN_BUS;
      err->target = 0;
      return;
    }
    MsgType mtype = CAN_MESSAGE_TYPE(newest.rHeader.Identifier);
    if (mtype != MSG_TYPE_COMMAND && mtype != MSG_TYPE_EMERGENCY &&
        mtype != MSG_TYPE_PRIORITY) {
      err->err = ERROR_CAN_BUS;
      err->target = 0;
      return;
    }
    if (papyrus_cmd_lens[newest.msg.command_id] == MSGLEN_SHORT) {
      uint8_t command_id = newest.msg.command_id;
      papyrus_cmdrun_table[command_id](&newest, base, err);
      CANMessage response;
      papyrus_cmdresp_table[command_id](&response, base, err);
      // TODO: better error checking here
      if (HAL_FDCAN_AddMessageToTxFifoQ(&base->can.handle, &response.tHeader,
                                        response.msg.raw_data) != HAL_OK) {
        err->err = ERROR_CAN_BUS;
        err->target = 0;
      }
    } else if (papyrus_cmd_lens[newest.msg.command_id] == MSGLEN_LONG) {
      bool found = false;
      CANMessage *trans;
      for (int i = 0; i < base->num_transactions; i++) {
        if (matches_transaction(&base->transactions[i], &newest)) {
          append_transaction(&base->transactions[i], &newest);
          trans = &base->transactions[i];
          found = true;
        }
      }
      if (!found) {
        if (base->num_transactions == 8) {
          err->err = ERROR_CAN_BUS;
          err->target = 0;
          return;
        }
        base->transactions[base->num_transactions] = newest;
        trans = &base->transactions[base->num_transactions];
        base->num_transactions++;
      }
      if (transaction_finished(trans)) {
        uint8_t command_id = trans->msg.command_id;
        papyrus_cmdrun_table[command_id](trans, base, err);
        CANMessage response;
        destroy_transaction(trans->next);
        papyrus_cmdresp_table[command_id](&response, base, err);
        // TODO: better error checking here
        if (HAL_FDCAN_AddMessageToTxFifoQ(&base->can.handle, &response.tHeader,
                                          response.msg.raw_data) != HAL_OK) {
          err->err = ERROR_CAN_BUS;
          err->target = 0;
        }
      }
    }
  }
}
