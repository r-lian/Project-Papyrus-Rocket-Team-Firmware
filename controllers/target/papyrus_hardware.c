#include "papyrus_hardware.h"
#include "stm32c0xx_hal_spi.h"

UART_HandleTypeDef stdio_uart;

int __io_putchar(char ch) {
  HAL_UART_Transmit(&stdio_uart, (uint8_t *)&ch, 1, 10);
  return ch;
}

int __io_getchar(void) {
  uint8_t ch = 0;
  __HAL_UART_CLEAR_OREFLAG(&stdio_uart);

  HAL_UART_Receive(&stdio_uart, &ch, 1, 0xFFFF);
  HAL_UART_Transmit(&stdio_uart, &ch, 1, 0xFFFF);
  return ch;
}
