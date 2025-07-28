
#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "stm32c092xx.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_uart.h"
#include "tc_controller.h"

const PapyrusGPIO TC_FAULT_LED = {GPIOA, GPIO_PIN_1};
const PapyrusGPIO TC_STATUS_LED = {GPIOA, GPIO_PIN_0};
const PapyrusGPIO TC_SPI_MOSI = {GPIOA, GPIO_PIN_7};
const PapyrusGPIO TC_SPI_MISO = {GPIOA, GPIO_PIN_6};
const PapyrusGPIO TC_SPI_SCK = {GPIOA, GPIO_PIN_5};
const PapyrusGPIO TC_SPI_FLASH_CS = {GPIOA, GPIO_PIN_4};
const PapyrusGPIO TC_SPI_TC_CS[] = {
    {GPIOB, GPIO_PIN_7},
    {GPIOC, GPIO_PIN_14},
    {GPIOC, GPIO_PIN_15},
};

const PapyrusGPIO TC_UART_RX = {GPIOA, GPIO_PIN_13};
const PapyrusGPIO TC_UART_TX = {GPIOA, GPIO_PIN_14};

int SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    return 1;
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    return 1;
  }
  return 0;
}
void SysTick_Handler(void) { HAL_IncTick(); }

PapyrusStatus tc_controller_init(TCController *tc_ctrl) {
  PapyrusStatus err;
  tc_ctrl->base.controller_type = CONTROLLER_TYPE_THERMOCOUPLE;
  tc_ctrl->base.board_revision = 0;
  FORWARD_ERR(controller_base_init(&tc_ctrl->base));
  FORWARD_ERR(tc_hardware_init(tc_ctrl));
  return PAPYRUS_OK;
}

PapyrusStatus tc_hardware_init(TCController *tc_ctrl) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  PapyrusStatus err;
  // Initialize base controller hardware
  tc_ctrl->base.fault_indicator = TC_FAULT_LED;
  tc_ctrl->base.status_indicator = TC_STATUS_LED;
  FORWARD_ERR(controller_hardware_init(&tc_ctrl->base));
  if (SystemClock_Config())
    return PAPYRUS_ERROR_HARDWARE;

  // Configure TC SPI bus
  tc_ctrl->flash_spi.mosi = TC_SPI_MOSI;
  tc_ctrl->flash_spi.miso = TC_SPI_MISO;
  tc_ctrl->flash_spi.sck = TC_SPI_SCK;
  tc_ctrl->flash_spi.cs = TC_SPI_FLASH_CS;
  FORWARD_ERR(controller_spi_init(&tc_ctrl->flash_spi));
  for (int i = 0; i < TC_MAX_CHANNELS; i++) {
    tc_ctrl->tc_spis[i].mosi = TC_SPI_MOSI;
    tc_ctrl->tc_spis[i].miso = TC_SPI_MISO;
    tc_ctrl->tc_spis[i].sck = TC_SPI_SCK;
    tc_ctrl->tc_spis[i].cs = TC_SPI_TC_CS[i];
    FORWARD_ERR(controller_spi_init(&tc_ctrl->tc_spis[i]));
  }

  // Configure CAN bus
  FORWARD_ERR(controller_fdcan_init(&tc_ctrl->base.can));

  return PAPYRUS_OK;
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (huart->Instance == USART2) {

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = TC_UART_RX.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(TC_UART_RX.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TC_UART_TX.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(TC_UART_TX.grp, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {

    __HAL_RCC_USART2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, TC_UART_RX.pin | TC_UART_TX.pin);
  }
}
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hspi->Instance == SPI1) {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S1;
    PeriphClkInit.I2s1ClockSelection = RCC_I2S1CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      Error_Handler();
    }
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = TC_SPI_MISO.pin | TC_SPI_MOSI.pin | TC_SPI_SCK.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(TC_SPI_MISO.grp, &GPIO_InitStruct);
  }
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param tc_ctrl->flash_spi.handle: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, TC_SPI_MISO.pin | TC_SPI_MOSI.pin | TC_SPI_SCK.pin);
  }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hfdcan->Instance == FDCAN1) {
    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN1;
    PeriphClkInit.Fdcan1ClockSelection = RCC_FDCAN1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      Error_Handler();
    }
    /* Peripheral clock enable */
    __HAL_RCC_FDCAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan->Instance == FDCAN1) {
    __HAL_RCC_FDCAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);
  }
}

void Error_Handler() {

  for (;;)
    ;
}

void NMI_Handler(void) { Error_Handler(); }

void HardFault_Handler(void) { Error_Handler(); }

void SVC_Handler(void) {}

void PendSV_Handler(void) {}
