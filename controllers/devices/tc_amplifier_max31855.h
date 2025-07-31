#pragma once
#include "papyrus_hardware.h"

#define TC_FAULT_OPEN 1
#define TC_FAULT_SHORT_GND 2
#define TC_FAULT_SHORT_VCC 4
#define TC_FAULT_SPI_FAILED 8

typedef struct {
  uint8_t read_bytes[4];
  int16_t tc_temperature;  // in 1/4s of a degree C
  int16_t cjc_temperature; // in 1/16s of a degree C
  int32_t tc_fixed;        // in 16.16 fixed point
  int32_t cjc_fixed;       // in 16.16 fixed point
  uint8_t err_flags;
} MAX31855Output;

PapyrusStatus max31855_read_tc(PapyrusSPI *spi, MAX31855Output *out);
