/**
 * @file controller_base.h
 * @brief Generic Controller Board Framework
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include "papyrus_can.h"
#include "papyrus_config.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal_fdcan.h"
#include <stdbool.h>
#include <stdint.h>

/* Controller Board Types */
typedef enum {
  CONTROLLER_TYPE_SERVO = 0,
  CONTROLLER_TYPE_THERMOCOUPLE,
  CONTROLLER_TYPE_IO_GENERAL,
  CONTROLLER_TYPE_POWER,
  CONTROLLER_TYPE_CUSTOM
} controller_type_t;

/* Controller States */
typedef enum {
  CONTROLLER_STATE_INIT = 0,
  CONTROLLER_STATE_NORMAL,
  CONTROLLER_STATE_SAFE_MODE,
  CONTROLLER_STATE_ERROR,
  CONTROLLER_STATE_MAINTENANCE,
  CONTROLLER_STATE_EMERGENCY_STOP
} controller_state_t;

/* Controller Configuration Base Structure */
typedef struct {
  /* Board Identification */
  board_id_t board_id;
  controller_type_t controller_type;
  uint8_t board_revision;
  uint32_t serial_number;

  /* CAN Configuration */
  uint32_t can_baudrate;
  uint8_t can_retry_count;
  uint16_t can_timeout_ms;

  /* Timing Configuration */
  uint16_t control_loop_rate_hz;
  uint16_t status_report_rate_hz;
  uint16_t heartbeat_rate_hz;

  /* Safety Configuration */
  uint16_t watchdog_timeout_ms;
  uint16_t safe_mode_timeout_ms;
  uint16_t emergency_timeout_ms;

  /* Hardware Configuration */
  uint8_t device_count;
  uint8_t adc_channels;
  uint8_t pwm_channels;
  uint8_t gpio_pins;

  /* Error Handling */
  uint8_t max_error_count;
  uint16_t error_escalation_time_ms;

  /* Debug Settings */
  bool debug_enabled;
  uint8_t debug_level;

  /* Controller-specific data (extended by derived types) */
  uint8_t specific_config[64];

  /* Configuration checksum */
  uint32_t config_crc;
} controller_config_t;

/* Controller Status Base Structure */
typedef struct {
  /* System Status */
  controller_state_t state;
  uint32_t startup_time;
  uint32_t uptime_sec;
  uint8_t cpu_usage_percent;
  int8_t temperature_c;

  /* Communication Status */
  uint32_t last_main_board_comm;
  uint16_t can_messages_sent;
  uint16_t can_messages_received;
  uint16_t can_errors;
  uint8_t communication_health;

  /* Device Status */
  uint8_t devices_connected;
  uint8_t devices_healthy;
  uint16_t device_error_flags;
  uint32_t total_operations;

  /* Error Status */
  uint16_t error_count;
  error_code_t last_error;
  uint32_t last_error_time;
  uint8_t error_flags;

  /* Power Status */
  uint16_t voltage_3v3_mv;
  uint16_t current_total_ma;
  uint8_t power_health;

  /* Memory Status */
  uint16_t stack_free_words;
  uint16_t heap_free_bytes;

  /* Controller-specific status (extended by derived types) */
  uint8_t specific_status[32];

} controller_status_t;

/* Device Interface Structure */
typedef struct {
  uint8_t device_id;
  uint8_t device_type;
  bool enabled;
  bool healthy;
  uint16_t error_flags;
  uint32_t last_update;
  void *device_data;
} device_interface_t;

/* Controller Base Class Structure */
typedef struct {
  /* Configuration and Status */
  controller_config_t config;
  controller_status_t status;

  /* Device Interfaces */
  device_interface_t devices[8];

  /* CAN Communication */
  FDCAN_HandleTypeDef *hcan;
  FDCAN_FilterTypeDef can_filter;

  /* Timing */
  uint32_t last_control_loop;
  uint32_t last_status_report;
  uint32_t last_heartbeat;

  /* Function Pointers for derived classes */
  papyrus_status_t (*init_hardware)(void);
  papyrus_status_t (*deinit_hardware)(void);
  papyrus_status_t (*update_devices)(void);
  papyrus_status_t (*process_command)(const can_message_t *msg);
  papyrus_status_t (*read_sensors)(void);
  papyrus_status_t (*enter_safe_mode)(void);
  papyrus_status_t (*exit_safe_mode)(void);
  papyrus_status_t (*emergency_stop)(void);
  papyrus_status_t (*self_test)(void);

} controller_base_t;

/* Function Prototypes - Base Controller Functions */

/**
 * @brief Initialize controller base system
 * @param controller Pointer to controller instance
 * @param board_id Board identifier
 * @param controller_type Type of controller
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_init(controller_base_t *controller,
                                      board_id_t board_id,
                                      controller_type_t controller_type);

/**
 * @brief Main controller processing loop
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_process(controller_base_t *controller);

/**
 * @brief Load controller configuration from flash
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_load_config(controller_base_t *controller);

/**
 * @brief Save controller configuration to flash
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_save_config(controller_base_t *controller);

/**
 * @brief Load default configuration
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_default_config(controller_base_t *controller);

/**
 * @brief Initialize CAN communication
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_can_init(controller_base_t *controller);

/**
 * @brief Send CAN message
 * @param controller Pointer to controller instance
 * @param msg Pointer to message to send
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_can_send(controller_base_t *controller,
                                          const can_message_t *msg);

/**
 * @brief Process received CAN message
 * @param controller Pointer to controller instance
 * @param msg Pointer to received message
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_can_process(controller_base_t *controller,
                                             const can_message_t *msg);

/**
 * @brief Send status report to main board
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_send_status(controller_base_t *controller);

/**
 * @brief Send heartbeat message
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_send_heartbeat(controller_base_t *controller);

/**
 * @brief Update controller status
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_update_status(controller_base_t *controller);

/**
 * @brief Set controller state
 * @param controller Pointer to controller instance
 * @param new_state Target state
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_set_state(controller_base_t *controller,
                                           controller_state_t new_state);

/**
 * @brief Get controller state
 * @param controller Pointer to controller instance
 * @return Current controller state
 */
controller_state_t
controller_base_get_state(const controller_base_t *controller);

/**
 * @brief Check if controller is healthy
 * @param controller Pointer to controller instance
 * @return true if healthy, false otherwise
 */
bool controller_base_is_healthy(const controller_base_t *controller);

/**
 * @brief Report error to main board
 * @param controller Pointer to controller instance
 * @param error_code Error code
 * @param error_data Additional error data
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_base_report_error(controller_base_t *controller,
                                              error_code_t error_code,
                                              uint16_t error_data);

/**
 * @brief Initialize device interface
 * @param device Pointer to device interface
 * @param device_id Device identifier
 * @param device_type Device type
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_device_init(device_interface_t *device,
                                        uint8_t device_id, uint8_t device_type);

/**
 * @brief Enable device
 * @param device Pointer to device interface
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_device_enable(device_interface_t *device);

/**
 * @brief Disable device
 * @param device Pointer to device interface
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_device_disable(device_interface_t *device);

/**
 * @brief Check device health
 * @param device Pointer to device interface
 * @return true if healthy, false otherwise
 */
bool controller_device_is_healthy(const device_interface_t *device);

/**
 * @brief Update device status
 * @param device Pointer to device interface
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_device_update(device_interface_t *device);

/* System Command Handlers */

/**
 * @brief Handle system reset command
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_handle_reset(controller_base_t *controller);

/**
 * @brief Handle emergency stop command
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t
controller_handle_emergency_stop(controller_base_t *controller);

/**
 * @brief Handle safe mode entry command
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t
controller_handle_safe_mode_enter(controller_base_t *controller);

/**
 * @brief Handle safe mode exit command
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t
controller_handle_safe_mode_exit(controller_base_t *controller);

/**
 * @brief Handle identification request
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_handle_identify(controller_base_t *controller);

/**
 * @brief Handle configuration save command
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_handle_config_save(controller_base_t *controller);

/**
 * @brief Handle configuration load command
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_handle_config_load(controller_base_t *controller);

/* Utility Functions */

/**
 * @brief Convert controller type to string
 * @param type Controller type
 * @return String description
 */
const char *controller_type_to_string(controller_type_t type);

/**
 * @brief Convert controller state to string
 * @param state Controller state
 * @return String description
 */
const char *controller_state_to_string(controller_state_t state);

/**
 * @brief Calculate controller uptime
 * @param controller Pointer to controller instance
 * @return Uptime in seconds
 */
uint32_t controller_get_uptime(const controller_base_t *controller);

/**
 * @brief Get free stack space
 * @return Free stack space in words
 */
uint16_t controller_get_stack_free(void);

/* Configuration Validation */

/**
 * @brief Validate controller configuration
 * @param config Pointer to configuration
 * @return PAPYRUS_OK if valid
 */
papyrus_status_t controller_validate_config(const controller_config_t *config);

/**
 * @brief Calculate configuration CRC
 * @param config Pointer to configuration
 * @return CRC32 checksum
 */
uint32_t controller_calculate_config_crc(const controller_config_t *config);

/* Debug and Testing Functions */

/**
 * @brief Enable debug mode
 * @param controller Pointer to controller instance
 * @param level Debug level (0-3)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_enable_debug(controller_base_t *controller,
                                         uint8_t level);

/**
 * @brief Disable debug mode
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t controller_disable_debug(controller_base_t *controller);

/**
 * @brief Run controller self-test
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK if all tests pass
 */
papyrus_status_t controller_run_self_test(controller_base_t *controller);

/* Default Configuration Values */
#define CONTROLLER_DEFAULT_CONTROL_LOOP_RATE_HZ 50
#define CONTROLLER_DEFAULT_STATUS_REPORT_RATE_HZ 10
#define CONTROLLER_DEFAULT_HEARTBEAT_RATE_HZ 1
#define CONTROLLER_DEFAULT_WATCHDOG_TIMEOUT_MS 5000
#define CONTROLLER_DEFAULT_SAFE_MODE_TIMEOUT_MS 30000
#define CONTROLLER_DEFAULT_EMERGENCY_TIMEOUT_MS 1000
#define CONTROLLER_DEFAULT_MAX_ERROR_COUNT 10
#define CONTROLLER_DEFAULT_ERROR_ESCALATION_MS 1000

#endif /* CONTROLLER_BASE_H */
