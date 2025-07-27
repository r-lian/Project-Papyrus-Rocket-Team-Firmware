#include "controller_base.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
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
