#include "tc_amplifier_max31855.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_spi.h"

PapyrusStatus max31855_read_tc(PapyrusSPI *spi, MAX31855Output *out) {
  HAL_GPIO_WritePin(GPIO(spi->cs), GPIO_PIN_RESET);
  if (HAL_SPI_Receive(&spi->handle, out->read_bytes, 4, 100) != HAL_OK) {
    out->err_flags = TC_FAULT_SPI_FAILED;
    HAL_GPIO_WritePin(GPIO(spi->cs), GPIO_PIN_SET);
    return PAPYRUS_ERROR_HARDWARE;
  }
  HAL_GPIO_WritePin(GPIO(spi->cs), GPIO_PIN_SET);
  if ((out->read_bytes[1] & 2) || (out->read_bytes[3] & 8)) {
    out->err_flags = TC_FAULT_SPI_FAILED;
    return PAPYRUS_ERROR_HARDWARE;
  }
  out->tc_temperature = (out->read_bytes[0] << 6) | (out->read_bytes[1] >> 2);
  out->cjc_temperature =
      ((int16_t)((out->read_bytes[2] << 8) | (out->read_bytes[3]))) >> 4;
  out->tc_fixed = ((int32_t)out->tc_temperature) << 14;
  out->cjc_fixed = ((int32_t)out->cjc_temperature) << 12;
  out->err_flags = out->read_bytes[3] & 7;
  if (out->err_flags) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  return PAPYRUS_OK;
}
