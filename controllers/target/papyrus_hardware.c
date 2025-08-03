#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal_cortex.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_spi.h"
#include <string.h>

UART_HandleTypeDef *stdio_uart;

PapyrusStatus controller_fdcan_init(PapyrusCAN *can) {
  memset(&can->handle, 0, sizeof(FDCAN_HandleTypeDef));
  can->handle.Instance = FDCAN1;
  can->handle.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  can->handle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  can->handle.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK; // FDCAN_MODE_NORMAL;
  can->handle.Init.AutoRetransmission = DISABLE;
  can->handle.Init.TransmitPause = ENABLE;
  can->handle.Init.ProtocolException = DISABLE;
  can->handle.Init.NominalPrescaler = 20;
  can->handle.Init.NominalSyncJumpWidth = 1;
  can->handle.Init.NominalTimeSeg1 = 14;
  can->handle.Init.NominalTimeSeg2 = 2;
  can->handle.Init.DataPrescaler = 1;
  can->handle.Init.DataSyncJumpWidth = 4;
  can->handle.Init.DataTimeSeg1 = 5;
  can->handle.Init.DataTimeSeg2 = 4;
  can->handle.Init.StdFiltersNbr = 3;
  can->handle.Init.ExtFiltersNbr = 0;
  can->handle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&can->handle) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  FDCAN_FilterTypeDef sFilterConfig = {0};

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0FF;
  sFilterConfig.FilterID2 = 0x0FF;
  if (HAL_FDCAN_ConfigFilter(&can->handle, &sFilterConfig) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterID1 = 0x000;
  if (HAL_FDCAN_ConfigFilter(&can->handle, &sFilterConfig) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  sFilterConfig.FilterIndex = 2;
  if (HAL_FDCAN_ConfigFilter(&can->handle, &sFilterConfig) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  irq_fdcan = &can->handle;
  if (HAL_FDCAN_ActivateNotification(
          &can->handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  if (HAL_FDCAN_Start(&can->handle) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  return PAPYRUS_OK;
}
PapyrusStatus controller_update_can_filter(PapyrusCAN *can, uint8_t new_id) {
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 2;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = new_id;
  sFilterConfig.FilterID2 = 0x0FF;
  if (HAL_FDCAN_ConfigFilter(&can->handle, &sFilterConfig) != HAL_OK) {
    return PAPYRUS_ERROR_HARDWARE;
  }
  return PAPYRUS_OK;
}
PapyrusStatus controller_spi_init(PapyrusSPI *spi) {
  memset(&spi->handle.Instance, 0, sizeof(SPI_HandleTypeDef));
  spi->handle.Instance = SPI1;
  spi->handle.Init.Mode = SPI_MODE_MASTER;
  spi->handle.Init.Direction = SPI_DIRECTION_2LINES;
  spi->handle.Init.DataSize = SPI_DATASIZE_8BIT;
  spi->handle.Init.CLKPolarity = SPI_POLARITY_LOW;
  spi->handle.Init.CLKPhase = SPI_PHASE_1EDGE;
  spi->handle.Init.NSS = SPI_NSS_SOFT;
  spi->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  spi->handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi->handle.Init.TIMode = SPI_TIMODE_DISABLE;
  spi->handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi->handle.Init.CRCPolynomial = 7;
  spi->handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  spi->handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&spi->handle) != HAL_OK)
    return PAPYRUS_ERROR_HARDWARE;
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = spi->cs.pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(spi->cs.grp, &gpio_init);
  HAL_GPIO_WritePin(GPIO(spi->cs), GPIO_PIN_SET);
  return PAPYRUS_OK;
}

void papyrus_prep_theader(FDCAN_TxHeaderTypeDef *tHeader) {
  tHeader->IdType = FDCAN_STANDARD_ID;
  tHeader->TxFrameType = FDCAN_DATA_FRAME;
  tHeader->ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  tHeader->BitRateSwitch = FDCAN_BRS_OFF;
  tHeader->FDFormat = FDCAN_CLASSIC_CAN;
  tHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tHeader->MessageMarker = 0;
}
