/**
 * @file tc_controller.h
 * @brief Thermocouple Controller Header
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef TC_CONTROLLER_H
#define TC_CONTROLLER_H

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

typedef int32_t
    TCTempRead; // Thermocouple temperature reading, 16.16 fixed point
typedef int32_t
    TCCjcRead; // Thermocouple cold junction reading, 16.16 fixed point

/* Thermocouple Controller Specific Configuration */
typedef struct {
  /* Thermocouple Configuration */
  TCType tc_type[TC_MAX_CHANNELS];

  /* Temperature Ranges */
  int16_t temp_alarm_low_c[TC_MAX_CHANNELS];
  int16_t temp_alarm_high_c[TC_MAX_CHANNELS];
  int16_t cjc_diff_alarm;

  /* Sampling Configuration */
  uint16_t sample_rate_hz;
  uint8_t filter_samples;
  uint16_t filter_window_ms;

  /* Fault Detection */
  uint8_t tc_fault[TC_MAX_CHANNELS];

  /* Communication */
  uint16_t data_report_rate_hz;
  bool continuous_mode;

} TCStatus;

/* Thermocouple Controller Class */
typedef struct {
  /* Base controller */
  ControllerBase base;

  /* TC-specific configuration and status */
  TCStatus tc_status;

  /* Hardware interfaces */
  PapyrusSPI tc_spis[TC_MAX_CHANNELS];
  PapyrusSPI flash_spi;

  /* Data processing (TODO)*/
  int32_t filter_buffer[TC_MAX_CHANNELS][16];
  uint8_t filter_index[TC_MAX_CHANNELS];
  uint32_t last_conversion_time[TC_MAX_CHANNELS];

  TCTempRead last_tc_values[TC_MAX_CHANNELS];
  /* Cold Junction Compensation */
  TCCjcRead last_cjc_values[TC_MAX_CHANNELS];

} TCController;

extern TCController this;

/* Function Prototypes - TC Controller Main Functions */

/**
 * @brief Initialize thermocouple controller
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus tc_controller_init(TCController *tc_ctrl);

/**
 * @brief Main TC controller processing loop
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus tc_controller_process(TCController *tc_ctrl);

/* Function Prototypes - Temperature Reading */

/**
 * @brief Read temperature from specific thermocouple
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple index
 * @return PAPYRUS_OK on success
 */
PapyrusStatus tc_read_thermocouple(TCController *tc_ctrl, uint8_t tc_id);

/* Function Prototypes - Hardware Interface */

/**
 * @brief Initialize thermocouple hardware (SPI, ADC, etc.)
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus tc_hardware_init(TCController *tc_ctrl);

/* Function Prototypes - Data Processing */

/**
 * @brief Check temperature alarms
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier
 * @return PAPYRUS_OK on success
 */
PapyrusStatus tc_check_alarms(TCController *tc_ctrl, uint8_t tc_id);

/**
 * @brief Set sampling rate
 * @param tc_ctrl Pointer to TC controller instance
 * @param sample_rate_hz Sampling rate in Hz
 * @return PAPYRUS_OK on success
 */
PapyrusStatus tc_set_sample_rate(TCController *tc_ctrl,
                                 uint16_t sample_rate_hz);

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

#endif /* TC_CONTROLLER_H */
