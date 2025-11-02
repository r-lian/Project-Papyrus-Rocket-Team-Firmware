#include "power_board.h"
#include "Legacy/stm32_hal_legacy.h"
#include "stm32c0xx_hal_adc.h"
#include "stm32c0xx_hal_gpio.h"
#include <stdio.h>
uint32_t ADC_Read_VDDA_mV(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t vrefint_raw = 0;
  uint32_t vrefint_cal = 0;

  // Configure for VREFINT channel
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  // ensure a long sampling time for VREFINT
  sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    return 0;
  if (HAL_ADC_Start(&hadc) != HAL_OK)
    return 0;
  if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
    return 0;
  vrefint_raw = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  if (vrefint_raw == 0)
    return 0; // avoid div0

  vrefint_cal = (uint32_t)(*VREFINT_CAL_ADDR);

  // VDDA_mV = (VREFINT_CAL_VDD_mV * vrefint_cal) / vrefint_raw
  uint64_t tmp = (uint64_t)3000U * (uint64_t)vrefint_cal;
  uint32_t vdda_mV = (uint32_t)(tmp / (uint64_t)vrefint_raw);

  return vdda_mV;
}
uint32_t vdda_mV;
int64_t read_adc_poll(uint32_t chan) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = chan;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
    return -1;
  }
  if (HAL_ADC_Start(&hadc) != HAL_OK)
    return -1;
  if (HAL_ADC_PollForConversion(&hadc, -1) != HAL_OK)
    return 1;
  return HAL_ADC_GetValue(&hadc);
}
void debug_adc(uint32_t chan, const char *name, int num) {
  UNUSED(num);
  printf("%s = %lld mV  ", name,
         read_adc_poll(chan) * (num * vdda_mV) / (4096 * 1000));
}
int main() {
  setvbuf(stdin, nullptr, _IONBF, 0);
  pb_hardware_init();
  HAL_Delay(500);
  vdda_mV = ADC_Read_VDDA_mV();
  printf("VDDA: %lu mV\r\n", vdda_mV);
  HAL_Delay(500);
  vdda_mV = ADC_Read_VDDA_mV();
  printf("VDDA: %lu mV\r\n", vdda_mV);
  HAL_Delay(500);
  vdda_mV = ADC_Read_VDDA_mV();
  printf("VDDA: %lu mV\r\n", vdda_mV);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIO(PB_5VBUCK_EN), GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO(PB_8VBUCK_EN), GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO(PB_24A_SW), GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO(PB_24B_SW), GPIO_PIN_RESET);
  for (;;) {
    HAL_GPIO_WritePin(GPIO(PB_24A_SW), GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO(PB_24A_SW), GPIO_PIN_RESET);
    /*HAL_GPIO_TogglePin(GPIO(PB_8V_LED));
    HAL_GPIO_TogglePin(GPIO(PB_5V_LED));
    HAL_GPIO_TogglePin(GPIO(PB_FAULT_LED));
    // printf("I24A=%ld ", read_adc_poll(ADC_CHAN_I24VA));
    // printf("I24B=%ld ", read_adc_poll(ADC_CHAN_I24VB));
    // printf("I5V=%ld ", read_adc_poll(ADC_CHAN_I5V));
    // printf("I8V=%ld ", read_adc_poll(ADC_CHAN_I8V));
    // printf("I3V3=%ld\r\n", read_adc_poll(ADC_CHAN_I3V3));
    // printf("5VREAD=%ld ", read_adc_poll(ADC_CHAN_5VREAD));
    // printf("8VREAD=%ld ", read_adc_poll(ADC_CHAN_8VREAD));
    // printf("24VREAD=%ld\r\n", read_adc_poll(ADC_CHAN_24VREAD));
    // debug_adc(ADC_CHAN_5VREAD, "+5V", 3060);
    debug_adc(ADC_CHAN_8VREAD, "+8V", 3060);
    //  debug_adc(ADC_CHAN_24VREAD, "+24V", 11000);
    printf("\r\n");
    HAL_Delay(500);*/
  }
}
