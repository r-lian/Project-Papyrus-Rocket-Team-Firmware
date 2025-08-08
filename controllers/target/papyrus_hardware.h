/**
 * @file papyrus_hardware.h
 * @brief Papyrus hardware helpers
 * @author Papyrus Avionics Team
 * @date 2024
 */

#pragma once

#include "papyrus_can.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_adc.h"
#include "stm32c0xx_hal_dma.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_i2c.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"

extern UART_HandleTypeDef *stdio_uart;
extern FDCAN_HandleTypeDef *irq_fdcan;
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

typedef struct {
  I2C_HandleTypeDef handle;
  PapyrusGPIO sda;
  PapyrusGPIO scl;
  uint16_t addr;
} PapyrusI2C;

typedef struct {
  ADC_HandleTypeDef handle;
  bool calibrated;
} PapyrusADC;

typedef struct {
  TIM_HandleTypeDef handle;
  PapyrusGPIO output;

} PapyrusPWM;

PapyrusStatus controller_spi_init(PapyrusSPI *spi);
PapyrusStatus controller_fdcan_init(PapyrusCAN *can);
void papyrus_prep_theader(FDCAN_TxHeaderTypeDef *tHeader);
PapyrusStatus controller_update_can_filter(PapyrusCAN *can, uint8_t new_id);

int __io_getchar(void);
int __io_putchar(char ch);
