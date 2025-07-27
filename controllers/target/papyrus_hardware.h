/**
 * @file papyrus_hardware.h
 * @brief Papyrus hardware helpers
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef PAPYRUS_HARDWARE_H
#define PAPYRUS_HARDWARE_H

#include "papyrus_utils.h"
#include "stm32c0xx_hal_dma.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_uart.h"

extern UART_HandleTypeDef stdio_uart;
typedef struct {
  UART_HandleTypeDef handle;
  USART_TypeDef *instance;
  uint32_t uart_baudrate;
  bool enabled;
} PapyrusUART;

typedef struct {
  GPIO_TypeDef *grp;
  uint16_t pin;
} PapyrusGPIO;

typedef struct {
  SPI_HandleTypeDef handle;
  PapyrusGPIO mosi;
  PapyrusGPIO miso;
  PapyrusGPIO sck;
  PapyrusGPIO cs;
} PapyrusSPI;

typedef struct {
  FDCAN_HandleTypeDef handle;
  uint32_t can_baudrate;
  uint8_t can_retry_count;
  uint16_t can_timeout_ms;
} PapyrusCAN;

#endif /* PAPYRUS_HARDWARE_H */
