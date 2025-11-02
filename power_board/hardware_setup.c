
#include "papyrus_hardware.h"
#include "power_board.h"
#include "stm32c092xx.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_adc.h"
#include "stm32c0xx_hal_adc_ex.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_i2c.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_uart.h"
#include <string.h>

const PapyrusGPIO PB_UART_RX = {GPIOA, GPIO_PIN_13};
const PapyrusGPIO PB_UART_TX = {GPIOA, GPIO_PIN_14};
const PapyrusGPIO PB_FAULT_LED = {GPIOB, GPIO_PIN_4};
const PapyrusGPIO PB_24V_LED = {GPIOB, GPIO_PIN_7};
const PapyrusGPIO PB_8V_LED = {GPIOB, GPIO_PIN_5};
const PapyrusGPIO PB_5V_LED = {GPIOB, GPIO_PIN_6};
const PapyrusGPIO PB_24A_SW = {GPIOB, GPIO_PIN_3};
const PapyrusGPIO PB_24B_SW = {GPIOA, GPIO_PIN_15};
const PapyrusGPIO PB_8VBUCK_EN = {GPIOA, GPIO_PIN_12};
const PapyrusGPIO PB_5VBUCK_EN = {GPIOA, GPIO_PIN_11};

const PapyrusGPIO PB_5VREAD = {GPIOA, GPIO_PIN_2};
const PapyrusGPIO PB_8VREAD = {GPIOA, GPIO_PIN_1};
const PapyrusGPIO PB_24VREAD = {GPIOA, GPIO_PIN_0};
const PapyrusGPIO PB_I3V3 = {GPIOA, GPIO_PIN_8};
const PapyrusGPIO PB_I5V = {GPIOA, GPIO_PIN_7};
const PapyrusGPIO PB_I8V = {GPIOA, GPIO_PIN_6};
const PapyrusGPIO PB_I24VA = {GPIOA, GPIO_PIN_3};
const PapyrusGPIO PB_I24VB = {GPIOA, GPIO_PIN_4};
ADC_HandleTypeDef hadc;
UART_HandleTypeDef *stdio_uart;
UART_HandleTypeDef huart2;
void Error_Handler() {

  for (;;)
    ;
}

int __io_putchar(char ch) {
  HAL_UART_Transmit(stdio_uart, (uint8_t *)&ch, 1, 10);
  return ch;
}
int __io_getchar(void) {
  uint8_t ch = 0;
  __HAL_UART_CLEAR_OREFLAG(stdio_uart);

  HAL_UART_Receive(stdio_uart, &ch, 1, 0xFFFF);
  // HAL_UART_Transmit(stdio_uart, &ch, 1, 0xFFFF);
  return ch;
}

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

void pb_gpio_init(PapyrusGPIO gpio) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = gpio.pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(gpio.grp, &gpio_init);
}

void pb_input_init(PapyrusGPIO gpio) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = gpio.pin;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLDOWN;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(gpio.grp, &gpio_init);
}

void pb_adc_init(ADC_HandleTypeDef *hadc, uint32_t chan) {
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc->Instance = ADC1;
  hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc->Init.Resolution = ADC_RESOLUTION_12B;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc->Init.LowPowerAutoWait = DISABLE;
  hadc->Init.LowPowerAutoPowerOff = DISABLE;
  hadc->Init.ContinuousConvMode = DISABLE;
  hadc->Init.NbrOfConversion = 1;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.DMAContinuousRequests = DISABLE;
  hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc->Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc->Init.OversamplingMode = DISABLE;
  hadc->Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(hadc) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(hadc);
  while (HAL_ADCEx_Calibration_GetValue(hadc) != HAL_OK)
    ;

  sConfig.Channel = chan;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hadc->Instance == ADC1) {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      Error_Handler();
    }

    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = PB_5VREAD.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_5VREAD.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PB_8VREAD.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_8VREAD.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PB_24VREAD.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_24VREAD.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PB_I3V3.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_I3V3.grp, &GPIO_InitStruct);

    /*GPIO_InitStruct.Pin = PB_I5V.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_I5V.grp, &GPIO_InitStruct);*/

    GPIO_InitStruct.Pin = PB_I8V.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_I8V.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PB_I24VA.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_I24VA.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PB_I24VB.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PB_I24VB.grp, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIO(PB_5VREAD));
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

PapyrusStatus pb_hardware_init() {
  PapyrusStatus err;
  if (HAL_Init() != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }

  if (SystemClock_Config())
    return PAPYRUS_ERROR_HARDWARE;
  UNUSED(err);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  pb_gpio_init(PB_FAULT_LED);
  pb_gpio_init(PB_24V_LED);
  pb_gpio_init(PB_8V_LED);
  pb_gpio_init(PB_5V_LED);

  pb_gpio_init(PB_5VBUCK_EN);
  pb_gpio_init(PB_8VBUCK_EN);
  pb_gpio_init(PB_24A_SW);
  pb_gpio_init(PB_24B_SW);

  /*pb_adc_init(&PB_ADC_24VREAD, ADC_CHANNEL_0);
  pb_adc_init(&PB_ADC_8VREAD, ADC_CHANNEL_1);
  pb_adc_init(&PB_ADC_5VREAD, ADC_CHANNEL_2);
  pb_adc_init(&PB_ADC_I3V3, ADC_CHANNEL_8);
  pb_adc_init(&PB_ADC_I5V, ADC_CHANNEL_7);
  pb_adc_init(&PB_ADC_I8V, ADC_CHANNEL_6);
  pb_adc_init(&PB_ADC_I24A, ADC_CHANNEL_3);
  pb_adc_init(&PB_ADC_I24B, ADC_CHANNEL_4);*/

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(&hadc);

  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  stdio_uart = &huart2;
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
    GPIO_InitStruct.Pin = PB_UART_RX.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(PB_UART_RX.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PB_UART_TX.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(PB_UART_TX.grp, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {

    __HAL_RCC_USART2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, PB_UART_RX.pin | PB_UART_TX.pin);
  }
}
void SysTick_Handler(void) { HAL_IncTick(); }

void NMI_Handler(void) { Error_Handler(); }

void HardFault_Handler(void) { Error_Handler(); }

void SVC_Handler(void) {}

void PendSV_Handler(void) {}
