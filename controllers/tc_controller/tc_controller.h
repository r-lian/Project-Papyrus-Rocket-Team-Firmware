/**
 * @file tc_controller.h
 * @brief Thermocouple Controller Header
 * @author Papyrus Avionics Team
 * @date 2024
 */

#pragma once

#include "controller_base.h"
#include "papyrus_can.h"
#include "papyrus_config.h"
#include "papyrus_hardware.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_uart.h"
#include "tc_amplifier_max31855.h"
#include <stdbool.h>
#include <stdint.h>

void Error_Handler();

#define ENABLE_UART
extern UART_HandleTypeDef huart;

/* Thermocouple Configuration */
#define TC_MAX_CHANNELS 3  // Maximum TCs per controller
#define TC_MAX_TEMP_C 1200 // Maximum measurable temperature
#define TC_MIN_TEMP_C -200 // Minimum measurable temperature

// #define ENABLE_UART

/* Hardware Definition */
extern const PapyrusGPIO TC_FAULT_LED;
extern const PapyrusGPIO TC_STATUS_LED;
extern const PapyrusGPIO TC_SPI_MOSI;
extern const PapyrusGPIO TC_SPI_MISO;
extern const PapyrusGPIO TC_SPI_SCK;
extern const PapyrusGPIO TC_SPI_FLASH_CS;
extern const PapyrusGPIO TC_SPI_TC_CS[];

extern const PapyrusGPIO TC_UART_RX;
extern const PapyrusGPIO TC_UART_TX;

/* Thermocouple Types */
typedef enum {
  TC_TYPE_K = 0, // Type K (Chromel/Alumel)
  TC_TYPE_J,     // Type J (Iron/Constantan)
  TC_TYPE_T,     // Type T (Copper/Constantan)
  TC_TYPE_E,     // Type E (Chromel/Constantan)
  TC_TYPE_N,     // Type N (Nicrosil/Nisil)
  TC_TYPE_S,     // Type S (Platinum/Platinum-Rhodium)
  TC_TYPE_R,     // Type R (Platinum/Platinum-Rhodium)
  TC_TYPE_B      // Type B (Platinum-Rhodium/Platinum-Rhodium)
} TCType;

typedef enum {
  TC_FMT_INT8_SHIFT,
  TC_FMT_INT16,
  TC_FMT_FIXED88,
  TC_FMT_FIXED1616
} TCReadFormat;

extern const uint8_t TC_READ_FORMAT_SIZE[];

typedef int32_t
    TCTempRead; // Thermocouple temperature reading, 16.16 fixed point

/* Thermocouple Controller Specific Configuration */
typedef struct {
  /* Thermocouple Configuration */
  TCType tc_type[TC_MAX_CHANNELS];

  /* Temperature Ranges */
  TCTempRead temp_alarm_low_c[TC_MAX_CHANNELS];
  TCTempRead temp_alarm_high_c[TC_MAX_CHANNELS];
  TCTempRead cjc_diff_alarm;

  bool stream_enabled[TC_MAX_CHANNELS];
  uint16_t stream_reload[TC_MAX_CHANNELS];
  uint16_t stream_timer[TC_MAX_CHANNELS];

  TCReadFormat tc_read_format[TC_MAX_CHANNELS];

  uint8_t num_tcs_active;

} TCConfig;

/* Thermocouple Controller Class */
typedef struct {
  /* Base controller */
  ControllerBase base;

  /* TC-specific configuration and status */
  TCConfig tc_config;

  /* Hardware interfaces */
  PapyrusSPI tc_spis[TC_MAX_CHANNELS];
  PapyrusSPI flash_spi;

  TCTempRead last_tc_read[TC_MAX_CHANNELS];
  TCTempRead last_cjc_read[TC_MAX_CHANNELS];
  uint8_t last_err_flags[TC_MAX_CHANNELS];

} TCController;

extern TCController this;

/* Function Prototypes - TC Controller Main Functions */

/**
 * @brief Initialize thermocouple controller
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus tc_controller_init(TCController *tc_ctrl);
PapyrusStatus tc_hardware_init(TCController *tc_ctrl);

int32_t to_tc_format(TCTempRead internal, TCReadFormat target);
TCTempRead from_tc_format(int32_t external, TCReadFormat target);

/* Default Configuration Values */
#define TC_DEFAULT_SAMPLE_RATE_HZ 10
#define TC_DEFAULT_FILTER_SAMPLES 8
#define TC_DEFAULT_FILTER_WINDOW_MS 1000
#define TC_DEFAULT_CJC_OFFSET_C 0
#define TC_DEFAULT_DATA_REPORT_RATE_HZ 1
#define TC_DEFAULT_TEMP_MIN_C -200
#define TC_DEFAULT_TEMP_MAX_C 1200
#define TC_DEFAULT_ALARM_LOW_C -50
#define TC_DEFAULT_ALARM_HIGH_C 500
