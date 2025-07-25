/**
 * @file tc_controller.c
 * @brief Thermocouple Controller Source
 * @author Papyrus Avionics Team
 * @date 2024
 */
#include "tc_controller.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include <stdio.h>
void SystemClock_Config();

tc_controller_t this;

int main() {
  if (tc_hardware_init(&this) != PAPYRUS_OK) {
    Error_Handler();
  }
  // setvbuf(stdin, nullptr, _IONBF, 0);
  //__io_getchar();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  // uint8_t dataBuf[4];
  // dataBuf[0] = 0;
  // dataBuf[1] = 0;
  // dataBuf[2] = 0;
  // dataBuf[3] = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  for (;;) {
    /*
    dataBuf[0] = 0x9F;
    printf("Reading Flash ID:\r\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi, &dataBuf[0], 1, 1000) != HAL_OK)
      Error_Handler();
    if (HAL_SPI_Receive(&hspi, &dataBuf[0], 3, 1000) != HAL_OK)
      Error_Handler();
    printf("Got data: %02x %02x %02x\r\n", dataBuf[0], dataBuf[1], dataBuf[2]);
    HAL_GPIO_WritePin(PG_FAULT_LED, PIN_FAULT_LED, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PG_FAULT_LED, PIN_FAULT_LED, GPIO_PIN_SET);
    HAL_Delay(500);*/
  }
}
