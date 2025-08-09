
#include "bus_debugger.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "stm32c092xx.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_adc_ex.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_uart.h"

const PapyrusGPIO BD_UART_RX = {GPIOA, GPIO_PIN_13};
const PapyrusGPIO BD_UART_TX = {GPIOA, GPIO_PIN_14};
const PapyrusGPIO BD_BUTTON_PINS[7] = {{GPIOB, GPIO_PIN_3}, {GPIOB, GPIO_PIN_4},
                                       {GPIOB, GPIO_PIN_5}, {GPIOB, GPIO_PIN_6},
                                       {GPIOB, GPIO_PIN_7}, {GPIOB, GPIO_PIN_8},
                                       {GPIOB, GPIO_PIN_9}};
const PapyrusGPIO BD_SPI_MISO = {GPIOA, GPIO_PIN_3};
const PapyrusGPIO BD_SPI_MOSI = {GPIOA, GPIO_PIN_4};
const PapyrusGPIO BD_SPI_SCK = {GPIOA, GPIO_PIN_0};
const PapyrusGPIO BD_PIEZO_PIN = {GPIOA, GPIO_PIN_1};
const PapyrusGPIO BD_BATREAD = {GPIOA, GPIO_PIN_2};
const PapyrusGPIO BD_I2C_SDA = {GPIOA, GPIO_PIN_12};
const PapyrusGPIO BD_I2C_SCL = {GPIOA, GPIO_PIN_11};
const PapyrusGPIO BD_BACKLIGHT_PIN = {GPIOC, GPIO_PIN_6};
const PapyrusGPIO BD_FAULT_LED = {GPIOA, GPIO_PIN_8};
const PapyrusGPIO BD_STATUS_LED = {GPIOA, GPIO_PIN_9};
const PapyrusGPIO BD_LCD_RESET = {GPIOB, GPIO_PIN_2};

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
uint8_t tick_cycle;
bool next_tick;
bool tick_too_slow;
void SysTick_Handler(void) {
  HAL_IncTick();
  tick_cycle++;
  if (tick_cycle == 20) {
    tick_cycle = 0;
    if (next_tick)
      tick_too_slow = true;
    next_tick = true;
  }
}

/*
 * GPIO_InitTypeDef gpio_init = {0};
 g pio_init.Pin = spi->cs.pin;           *
 gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
 gpio_init.Pull = GPIO_NOPULL;
 gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(spi->cs.grp, &gpio_init);
 */

void bd_gpio_init(PapyrusGPIO gpio) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = gpio.pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(gpio.grp, &gpio_init);
}

void bd_input_init(PapyrusGPIO gpio) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = gpio.pin;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLDOWN;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(gpio.grp, &gpio_init);
}

PapyrusStatus bus_debugger_init(BusDebugger *bus_dbg) {
  PapyrusStatus err;

  FORWARD_ERR(bd_hardware_init(bus_dbg));
  return PAPYRUS_OK;
}

PapyrusStatus bd_hardware_init(BusDebugger *bus_dbg) {
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

  // Init GPIOs
  bus_dbg->backlight = BD_BACKLIGHT_PIN;
  bd_gpio_init(BD_BACKLIGHT_PIN);
  bus_dbg->piezo.output = BD_PIEZO_PIN;
  bd_gpio_init(BD_PIEZO_PIN);
  bus_dbg->status_led = BD_STATUS_LED;
  bd_gpio_init(BD_STATUS_LED);
  bus_dbg->fault_led = BD_FAULT_LED;
  bd_gpio_init(BD_FAULT_LED);
  bus_dbg->lcd_reset = BD_LCD_RESET;
  bd_gpio_init(BD_LCD_RESET);
  for (uint8_t i = 0; i < 7; i++) {
    bus_dbg->button_pins[i] = BD_BUTTON_PINS[i];
    bd_input_init(BD_BUTTON_PINS[i]);
  }

  HAL_GPIO_WritePin(GPIO(BD_BACKLIGHT_PIN), GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO(BD_LCD_RESET), GPIO_PIN_RESET);

  // Init I2C

  bus_dbg->display.handle.Instance = I2C2;
  bus_dbg->display.handle.Init.Timing = 0x00402D41;
  bus_dbg->display.handle.Init.OwnAddress1 = 0;
  bus_dbg->display.handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  bus_dbg->display.handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  bus_dbg->display.handle.Init.OwnAddress2 = 0;
  bus_dbg->display.handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  bus_dbg->display.handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  bus_dbg->display.handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&bus_dbg->display.handle) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&bus_dbg->display.handle,
                                   I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&bus_dbg->display.handle, 0) != HAL_OK) {
    Error_Handler();
  }

  ADC_ChannelConfTypeDef sConfig = {0};

  bus_dbg->batread.handle.Instance = ADC1;
  bus_dbg->batread.handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  bus_dbg->batread.handle.Init.Resolution = ADC_RESOLUTION_12B;
  bus_dbg->batread.handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  bus_dbg->batread.handle.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  bus_dbg->batread.handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  bus_dbg->batread.handle.Init.LowPowerAutoWait = DISABLE;
  bus_dbg->batread.handle.Init.LowPowerAutoPowerOff = DISABLE;
  bus_dbg->batread.handle.Init.ContinuousConvMode = DISABLE;
  bus_dbg->batread.handle.Init.NbrOfConversion = 1;
  bus_dbg->batread.handle.Init.DiscontinuousConvMode = DISABLE;
  bus_dbg->batread.handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  bus_dbg->batread.handle.Init.ExternalTrigConvEdge =
      ADC_EXTERNALTRIGCONVEDGE_NONE;
  bus_dbg->batread.handle.Init.DMAContinuousRequests = DISABLE;
  bus_dbg->batread.handle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  bus_dbg->batread.handle.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  bus_dbg->batread.handle.Init.OversamplingMode = DISABLE;
  bus_dbg->batread.handle.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&bus_dbg->batread.handle) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(&bus_dbg->batread.handle);

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&bus_dbg->batread.handle, &sConfig) != HAL_OK) {
    Error_Handler();
  }

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
    GPIO_InitStruct.Pin = BD_UART_RX.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(BD_UART_RX.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BD_UART_TX.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(BD_UART_TX.grp, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {

    __HAL_RCC_USART2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, BD_UART_RX.pin | BD_UART_TX.pin);
  }
}
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hi2c->Instance == I2C2) {

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = BD_I2C_SCL.pin | BD_I2C_SDA.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(BD_I2C_SCL.grp, &GPIO_InitStruct);
    __HAL_RCC_I2C2_CLK_ENABLE();
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C2) {

    __HAL_RCC_I2C2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, BD_I2C_SCL.pin);

    HAL_GPIO_DeInit(GPIOA, BD_I2C_SDA.pin);
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

    GPIO_InitStruct.Pin = BD_BATREAD.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BD_BATREAD.grp, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIO(BD_BATREAD));
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hspi->Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = BD_SPI_SCK.pin | BD_SPI_MISO.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
    HAL_GPIO_Init(BD_SPI_SCK.grp, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BD_SPI_MOSI.pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_SPI2;
    HAL_GPIO_Init(BD_SPI_MOSI.grp, &GPIO_InitStruct);
  }
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param tc_ctrl->flash_spi.handle: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_DISABLE();

    HAL_GPIO_DeInit(BD_SPI_MOSI.grp,
                    BD_SPI_MOSI.pin | BD_SPI_MISO.pin | BD_SPI_SCK.pin);
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

void NMI_Handler(void) { Error_Handler(); }

void HardFault_Handler(void) { Error_Handler(); }

void SVC_Handler(void) {}

void PendSV_Handler(void) {}

FDCAN_HandleTypeDef *irq_fdcan;
void FDCAN_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(irq_fdcan); }
