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
#include <stdbool.h>
#include <stdint.h>

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
} tc_type_t;

/* Thermocouple Controller Specific Configuration */
typedef struct {
  /* Thermocouple Configuration */
  uint8_t tc_count;
  tc_type_t tc_types[TC_MAX_CHANNELS];
  bool tc_enabled[TC_MAX_CHANNELS];

  /* Temperature Ranges */
  int16_t temp_min_c[TC_MAX_CHANNELS];
  int16_t temp_max_c[TC_MAX_CHANNELS];
  int16_t temp_alarm_low_c[TC_MAX_CHANNELS];
  int16_t temp_alarm_high_c[TC_MAX_CHANNELS];

  /* Sampling Configuration */
  uint16_t sample_rate_hz;
  uint8_t filter_samples;
  uint16_t filter_window_ms;

  /* Cold Junction Compensation */
  bool cjc_enabled;
  int16_t cjc_offset_c;
  uint8_t cjc_sensor_pin;

  /* Calibration Data */
  bool calibrated[TC_MAX_CHANNELS];
  float gain[TC_MAX_CHANNELS];
  float offset[TC_MAX_CHANNELS];

  /* Fault Detection */
  bool open_circuit_detection;
  bool short_circuit_detection;
  uint16_t fault_timeout_ms;

  /* Communication */
  uint16_t data_report_rate_hz;
  bool continuous_mode;

} tc_config_t;

/* Thermocouple Controller Status */
typedef struct {
  /* Current Temperature Readings */
  int16_t temperature_c[TC_MAX_CHANNELS];
  int16_t raw_voltage_uv[TC_MAX_CHANNELS];
  int16_t cjc_temperature_c;
  bool tc_connected[TC_MAX_CHANNELS];

  /* Temperature Statistics */
  int16_t temp_min_recorded[TC_MAX_CHANNELS];
  int16_t temp_max_recorded[TC_MAX_CHANNELS];
  int16_t temp_average[TC_MAX_CHANNELS];

  /* Alarm Status */
  bool temp_alarm_low[TC_MAX_CHANNELS];
  bool temp_alarm_high[TC_MAX_CHANNELS];
  bool temp_rate_alarm[TC_MAX_CHANNELS];

  /* Fault Status */
  bool open_circuit[TC_MAX_CHANNELS];
  bool short_circuit[TC_MAX_CHANNELS];
  bool out_of_range[TC_MAX_CHANNELS];
  bool cjc_fault;
  uint16_t fault_count[TC_MAX_CHANNELS];

  /* Sampling Status */
  uint32_t samples_taken[TC_MAX_CHANNELS];
  uint32_t last_sample_time[TC_MAX_CHANNELS];
  uint16_t sample_rate_actual;

  /* Performance Metrics */
  uint16_t conversion_time_us;
  uint16_t filter_settling_time_ms;
  uint8_t data_quality[TC_MAX_CHANNELS];

} tc_status_t;

/* Thermocouple Commands */
typedef enum {
  TC_CMD_READ_TEMP = 0x20,
  TC_CMD_READ_ALL = 0x21,
  TC_CMD_SET_ALARMS = 0x22,
  TC_CMD_CLEAR_ALARMS = 0x23,
  TC_CMD_CALIBRATE = 0x24,
  TC_CMD_SET_TYPE = 0x25,
  TC_CMD_ENABLE_TC = 0x26,
  TC_CMD_DISABLE_TC = 0x27,
  TC_CMD_RESET_STATS = 0x28,
  TC_CMD_SET_RATE = 0x29
} tc_command_t;

/* Temperature Data Structure for CAN */
typedef struct {
  uint8_t tc_id;
  int16_t temperature_c;
  uint8_t quality;
  uint8_t flags;
} __attribute__((packed)) tc_data_packet_t;

/* Thermocouple Controller Class */
typedef struct {
  /* Base controller */
  controller_base_t base;

  /* TC-specific configuration and status */
  tc_config_t tc_config;
  tc_status_t tc_status;

  /* Hardware interfaces */
  SPI_HandleTypeDef *hspi;
  ADC_HandleTypeDef *hadc_cjc;
  GPIO_TypeDef *cs_gpio_port[TC_MAX_CHANNELS];
  uint16_t cs_gpio_pin[TC_MAX_CHANNELS];

  /* Data processing */
  int32_t filter_buffer[TC_MAX_CHANNELS][16];
  uint8_t filter_index[TC_MAX_CHANNELS];
  uint32_t last_conversion_time[TC_MAX_CHANNELS];

  /* Alarm tracking */
  uint32_t alarm_start_time[TC_MAX_CHANNELS];
  bool alarm_latched[TC_MAX_CHANNELS];

} tc_controller_t;

/* Function Prototypes - TC Controller Main Functions */

/**
 * @brief Initialize thermocouple controller
 * @param tc_ctrl Pointer to TC controller instance
 * @param board_id Board identifier
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_controller_init(tc_controller_t *tc_ctrl,
                                    board_id_t board_id);

/**
 * @brief Main TC controller processing loop
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_controller_process(tc_controller_t *tc_ctrl);

/**
 * @brief Deinitialize TC controller
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_controller_deinit(tc_controller_t *tc_ctrl);

/* Function Prototypes - Temperature Reading */

/**
 * @brief Read temperature from specific thermocouple
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param temperature_c Pointer to store temperature in Celsius
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_read_temperature(tc_controller_t *tc_ctrl, uint8_t tc_id,
                                     int16_t *temperature_c);

/**
 * @brief Read all thermocouple temperatures
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_read_all_temperatures(tc_controller_t *tc_ctrl);

/**
 * @brief Read raw voltage from thermocouple
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param voltage_uv Pointer to store voltage in microvolts
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_read_raw_voltage(tc_controller_t *tc_ctrl, uint8_t tc_id,
                                     int32_t *voltage_uv);

/**
 * @brief Read cold junction temperature
 * @param tc_ctrl Pointer to TC controller instance
 * @param cjc_temp_c Pointer to store CJC temperature
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_read_cjc_temperature(tc_controller_t *tc_ctrl,
                                         int16_t *cjc_temp_c);

/* Function Prototypes - Hardware Interface */

/**
 * @brief Initialize thermocouple hardware (SPI, ADC, etc.)
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_hardware_init(tc_controller_t *tc_ctrl);

/**
 * @brief Start temperature conversion for specific TC
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_start_conversion(tc_controller_t *tc_ctrl, uint8_t tc_id);

/**
 * @brief Read conversion result from TC amplifier
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param raw_data Pointer to store raw ADC data
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_read_conversion(tc_controller_t *tc_ctrl, uint8_t tc_id,
                                    uint32_t *raw_data);

/**
 * @brief Check for thermocouple faults
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK if no faults detected
 */
papyrus_status_t tc_check_faults(tc_controller_t *tc_ctrl, uint8_t tc_id);

/* Function Prototypes - Data Processing */

/**
 * @brief Convert raw voltage to temperature
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param voltage_uv Raw voltage in microvolts
 * @param temperature_c Pointer to store temperature
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_voltage_to_temperature(tc_controller_t *tc_ctrl,
                                           uint8_t tc_id, int32_t voltage_uv,
                                           int16_t *temperature_c);

/**
 * @brief Apply digital filter to temperature reading
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param raw_temp Raw temperature reading
 * @return Filtered temperature
 */
int16_t tc_apply_filter(tc_controller_t *tc_ctrl, uint8_t tc_id,
                        int16_t raw_temp);

/**
 * @brief Compensate for cold junction temperature
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param hot_junction_temp Hot junction temperature
 * @return Cold junction compensated temperature
 */
int16_t tc_compensate_cjc(tc_controller_t *tc_ctrl, uint8_t tc_id,
                          int16_t hot_junction_temp);

/**
 * @brief Update temperature statistics
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param temperature New temperature reading
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_update_statistics(tc_controller_t *tc_ctrl, uint8_t tc_id,
                                      int16_t temperature);

/* Function Prototypes - Alarm Management */

/**
 * @brief Set temperature alarm limits
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param low_limit Low temperature alarm limit
 * @param high_limit High temperature alarm limit
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_set_alarm_limits(tc_controller_t *tc_ctrl, uint8_t tc_id,
                                     int16_t low_limit, int16_t high_limit);

/**
 * @brief Check temperature alarms
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_check_alarms(tc_controller_t *tc_ctrl, uint8_t tc_id);

/**
 * @brief Clear temperature alarms
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_clear_alarms(tc_controller_t *tc_ctrl, uint8_t tc_id);

/**
 * @brief Get alarm status
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return Alarm flags bitmask
 */
uint8_t tc_get_alarm_status(tc_controller_t *tc_ctrl, uint8_t tc_id);

/* Function Prototypes - Configuration */

/**
 * @brief Set thermocouple type
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param tc_type Thermocouple type
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_set_type(tc_controller_t *tc_ctrl, uint8_t tc_id,
                             tc_type_t tc_type);

/**
 * @brief Enable thermocouple
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_enable(tc_controller_t *tc_ctrl, uint8_t tc_id);

/**
 * @brief Disable thermocouple
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_disable(tc_controller_t *tc_ctrl, uint8_t tc_id);

/**
 * @brief Set sampling rate
 * @param tc_ctrl Pointer to TC controller instance
 * @param sample_rate_hz Sampling rate in Hz
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_set_sample_rate(tc_controller_t *tc_ctrl,
                                    uint16_t sample_rate_hz);

/* Function Prototypes - Calibration */

/**
 * @brief Calibrate thermocouple
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param reference_temp_c Reference temperature for calibration
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_calibrate(tc_controller_t *tc_ctrl, uint8_t tc_id,
                              int16_t reference_temp_c);

/**
 * @brief Set calibration parameters
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param gain Calibration gain
 * @param offset Calibration offset
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_set_calibration(tc_controller_t *tc_ctrl, uint8_t tc_id,
                                    float gain, float offset);

/**
 * @brief Auto-calibrate using ice point reference
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_auto_calibrate_ice_point(tc_controller_t *tc_ctrl);

/* Function Prototypes - CAN Communication */

/**
 * @brief Process TC command from CAN
 * @param tc_ctrl Pointer to TC controller instance
 * @param msg Pointer to CAN message
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_process_can_command(tc_controller_t *tc_ctrl,
                                        const can_message_t *msg);

/**
 * @brief Send TC status via CAN
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_send_status(tc_controller_t *tc_ctrl);

/**
 * @brief Send temperature data via CAN
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_send_data(tc_controller_t *tc_ctrl);

/**
 * @brief Send specific TC reading via CAN
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_send_single_reading(tc_controller_t *tc_ctrl,
                                        uint8_t tc_id);

/* Function Prototypes - Safety and Error Handling */

/**
 * @brief Enter safe mode (disable all TCs)
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_enter_safe_mode(tc_controller_t *tc_ctrl);

/**
 * @brief Exit safe mode
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_exit_safe_mode(tc_controller_t *tc_ctrl);

/**
 * @brief Emergency stop (immediate shutdown)
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_emergency_stop(tc_controller_t *tc_ctrl);

/**
 * @brief Check TC health
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return true if TC is healthy
 */
bool tc_is_healthy(tc_controller_t *tc_ctrl, uint8_t tc_id);

/**
 * @brief Run TC self-test
 * @param tc_ctrl Pointer to TC controller instance
 * @return PAPYRUS_OK if all tests pass
 */
papyrus_status_t tc_self_test(tc_controller_t *tc_ctrl);

/* Utility Functions */

/**
 * @brief Convert thermocouple type to string
 * @param tc_type Thermocouple type
 * @return String description
 */
const char *tc_type_to_string(tc_type_t tc_type);

/**
 * @brief Get thermocouple temperature range
 * @param tc_type Thermocouple type
 * @param min_temp_c Pointer to store minimum temperature
 * @param max_temp_c Pointer to store maximum temperature
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_get_temp_range(tc_type_t tc_type, int16_t *min_temp_c,
                                   int16_t *max_temp_c);

/**
 * @brief Validate temperature reading
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @param temperature Temperature to validate
 * @return true if temperature is valid
 */
bool tc_validate_temperature(tc_controller_t *tc_ctrl, uint8_t tc_id,
                             int16_t temperature);

/**
 * @brief Reset statistics for specific TC
 * @param tc_ctrl Pointer to TC controller instance
 * @param tc_id Thermocouple identifier (0-7)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t tc_reset_statistics(tc_controller_t *tc_ctrl, uint8_t tc_id);

/* Default Configuration Values */
#define TC_DEFAULT_SAMPLE_RATE_HZ 10
#define TC_DEFAULT_FILTER_SAMPLES 8
#define TC_DEFAULT_FILTER_WINDOW_MS 1000
#define TC_DEFAULT_CJC_OFFSET_C 0
#define TC_DEFAULT_FAULT_TIMEOUT_MS 5000
#define TC_DEFAULT_DATA_REPORT_RATE_HZ 1
#define TC_DEFAULT_TEMP_MIN_C -200
#define TC_DEFAULT_TEMP_MAX_C 1200
#define TC_DEFAULT_ALARM_LOW_C -50
#define TC_DEFAULT_ALARM_HIGH_C 500

/* Thermocouple Polynomial Coefficients (Type K example) */
extern const float tc_k_coefficients[10];
extern const float tc_k_reverse_coefficients[10];

/* Alarm flags */
#define TC_ALARM_NONE 0x00
#define TC_ALARM_LOW_TEMP 0x01
#define TC_ALARM_HIGH_TEMP 0x02
#define TC_ALARM_OPEN_CIRCUIT 0x04
#define TC_ALARM_SHORT_CIRCUIT 0x08
#define TC_ALARM_OUT_OF_RANGE 0x10
#define TC_ALARM_CJC_FAULT 0x20

#endif /* TC_CONTROLLER_H */
