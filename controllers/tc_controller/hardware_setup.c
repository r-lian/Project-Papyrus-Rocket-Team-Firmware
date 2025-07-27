
#include "controller_base.h"
#include "papyrus_utils.h"
#include "stm32c092xx.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include "tc_controller.h"

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

PapyrusStatus tc_hardware_init(TCController *tc_ctrl) {
  if (SystemClock_Config())
    return PAPYRUS_ERROR_HARDWARE;
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure TC SPI bus
  tc_ctrl->flash_spi.mosi = TC_SPI_MOSI;
  tc_ctrl->flash_spi.miso = TC_SPI_MISO;
  tc_ctrl->flash_spi.sck = TC_SPI_SCK;
  tc_ctrl->flash_spi.cs = TC_SPI_FLASH_CS;

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
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {

    __HAL_RCC_USART2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_13 | GPIO_PIN_14);
  }
}
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hspi->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */

    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S1;
    PeriphClkInit.I2s1ClockSelection = RCC_I2S1CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
     *    PA5     ------> SPI1_SCK
     *    PA6     ------> SPI1_MISO
     *    PA7     ------> SPI1_MOSI
     */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
     *    PA5     ------> SPI1_SCK
     *    PA6     ------> SPI1_MISO
     *    PA7     ------> SPI1_MOSI
     */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base) {
  if (htim_base->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (htim->Instance == TIM1) {

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base) {
  if (htim_base->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
}
void Error_Handler() {
  HAL_GPIO_WritePin(PG_FAULT_LED, PIN_FAULT_LED, GPIO_PIN_SET);
  for (;;)
    ;
}

void NMI_Handler(void) {

  while (1) {
  }
}

void HardFault_Handler(void) {

  while (1) {
  }
}

void SVC_Handler(void) {}

void PendSV_Handler(void) {}
