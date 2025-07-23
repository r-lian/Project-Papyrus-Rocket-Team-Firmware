/**
 * @file main_board.h
 * @brief Papyrus Main Board Header
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef MAIN_BOARD_H
#define MAIN_BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "papyrus_config.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"

/* Main Board Configuration Structure */
typedef struct {
    /* System Configuration */
    uint16_t system_id;
    uint8_t board_revision;
    uint32_t serial_number;
    
    /* CAN Configuration */
    uint32_t can_baudrate;
    uint8_t can_retry_count;
    uint16_t can_timeout_ms;
    
    /* Radio Configuration */
    uint32_t radio_frequency;
    uint16_t radio_power_dbm;
    uint16_t radio_timeout_ms;
    
    /* Power Management */
    uint16_t voltage_3v3_min_mv;
    uint16_t voltage_3v3_max_mv;
    uint16_t voltage_5v0_min_mv;
    uint16_t voltage_5v0_max_mv;
    uint16_t voltage_12v_min_mv;
    uint16_t voltage_12v_max_mv;
    uint16_t current_limit_ma;
    
    /* Safety Configuration */
    uint16_t emergency_timeout_ms;
    uint16_t safe_mode_timeout_ms;
    uint16_t watchdog_timeout_ms;
    
    /* Data Logging */
    bool log_enabled;
    uint16_t log_rate_hz;
    uint32_t log_file_size_kb;
    
    /* Debug Settings */
    bool debug_enabled;
    uint8_t debug_level;
    
    /* Configuration checksum */
    uint32_t config_crc;
} main_board_config_t;

/* Main Board Status Structure */
typedef struct {
    /* System Status */
    system_state_t system_state;
    uint32_t startup_time;
    uint32_t uptime_sec;
    uint8_t cpu_usage_percent;
    int8_t temperature_c;
    
    /* Power Status */
    uint16_t voltage_3v3_mv;
    uint16_t voltage_5v0_mv;
    uint16_t voltage_12v_mv;
    uint16_t current_total_ma;
    uint8_t battery_percent;
    
    /* Communication Status */
    uint16_t can_messages_sent;
    uint16_t can_messages_received;
    uint16_t can_errors;
    uint16_t radio_packets_sent;
    uint16_t radio_packets_received;
    uint16_t radio_signal_strength;
    
    /* Controller Board Status */
    uint16_t connected_boards_mask;
    uint16_t healthy_boards_mask;
    uint32_t last_board_comm[16];
    
    /* Error Status */
    uint16_t error_count;
    error_code_t last_error;
    uint32_t last_error_time;
    
    /* Data Logging Status */
    bool logging_active;
    uint32_t log_entries_written;
    uint32_t log_file_size_bytes;
    uint16_t log_errors;
    
    /* Memory Status */
    uint32_t heap_free_bytes;
    uint32_t heap_min_free_bytes;
    uint16_t stack_min_free_words[8]; /* Per task */
    
} main_board_status_t;

/* System Event Structure */
typedef struct {
    uint8_t event_type;
    uint8_t source_board;
    uint16_t event_data;
    uint32_t timestamp;
} system_event_t;

/* Data Log Entry Structure */
typedef struct {
    uint32_t timestamp;
    uint8_t data_type;
    uint8_t source_board;
    uint16_t data_length;
    uint8_t data[32];
} data_log_entry_t;

/* Event Types */
typedef enum {
    EVENT_SYSTEM_START = 0,
    EVENT_BOARD_CONNECTED,
    EVENT_BOARD_DISCONNECTED,
    EVENT_ERROR_OCCURRED,
    EVENT_ERROR_CLEARED,
    EVENT_EMERGENCY_STOP,
    EVENT_SAFE_MODE_ENTER,
    EVENT_SAFE_MODE_EXIT,
    EVENT_CONFIGURATION_CHANGED,
    EVENT_CALIBRATION_COMPLETE
} system_event_type_t;

/* Function Prototypes - Main Board Management */

/**
 * @brief Load main board configuration from flash
 * @param config Pointer to configuration structure
 * @return PAPYRUS_OK on success
 */
papyrus_status_t main_board_load_config(main_board_config_t* config);

/**
 * @brief Save main board configuration to flash
 * @param config Pointer to configuration structure
 * @return PAPYRUS_OK on success
 */
papyrus_status_t main_board_save_config(const main_board_config_t* config);

/**
 * @brief Load default configuration
 * @param config Pointer to configuration structure
 * @return PAPYRUS_OK on success
 */
papyrus_status_t main_board_default_config(main_board_config_t* config);

/**
 * @brief Validate configuration
 * @param config Pointer to configuration structure
 * @return PAPYRUS_OK if valid
 */
papyrus_status_t main_board_validate_config(const main_board_config_t* config);

/**
 * @brief Get main board status
 * @return Pointer to status structure
 */
main_board_status_t* main_board_get_status(void);

/**
 * @brief Get main board configuration
 * @return Pointer to configuration structure
 */
main_board_config_t* main_board_get_config(void);

/**
 * @brief Get CAN TX queue handle
 * @return Queue handle
 */
QueueHandle_t main_board_get_can_tx_queue(void);

/**
 * @brief Get system event queue handle
 * @return Queue handle
 */
QueueHandle_t main_board_get_system_event_queue(void);

/**
 * @brief Get data log queue handle
 * @return Queue handle
 */
QueueHandle_t main_board_get_data_log_queue(void);

/* Function Prototypes - Emergency Handling */

/**
 * @brief Execute emergency stop procedure
 * @param error_code Reason for emergency stop
 * @return PAPYRUS_OK on success
 */
papyrus_status_t emergency_stop_procedure(error_code_t error_code);

/**
 * @brief Check for critical system conditions
 * @return PAPYRUS_OK if all conditions normal
 */
papyrus_status_t check_critical_conditions(void);

/**
 * @brief Monitor task health and stack usage
 * @return PAPYRUS_OK if all tasks healthy
 */
papyrus_status_t monitor_task_health(void);

/* Function Prototypes - Board Communication */

/**
 * @brief Send command to controller board
 * @param board_id Target board ID
 * @param command Command to send
 * @param data Command data
 * @param length Data length
 * @return PAPYRUS_OK on success
 */
papyrus_status_t send_board_command(board_id_t board_id, uint8_t command,
                                   const uint8_t* data, uint8_t length);

/**
 * @brief Request status from controller board
 * @param board_id Target board ID
 * @return PAPYRUS_OK on success
 */
papyrus_status_t request_board_status(board_id_t board_id);

/**
 * @brief Check controller board health
 * @param board_id Board to check
 * @return true if board is healthy
 */
bool is_board_healthy(board_id_t board_id);

/**
 * @brief Get last communication time with board
 * @param board_id Board to check
 * @return Timestamp of last communication
 */
uint32_t get_board_last_comm(board_id_t board_id);

/* Function Prototypes - System Control */

/**
 * @brief Set system state
 * @param new_state Target system state
 * @return PAPYRUS_OK on success
 */
papyrus_status_t set_system_state(system_state_t new_state);

/**
 * @brief Process system events
 * @return PAPYRUS_OK on success
 */
papyrus_status_t process_system_events(void);

/**
 * @brief Send system command to all boards
 * @param command System command to send
 * @return PAPYRUS_OK on success
 */
papyrus_status_t broadcast_system_command(system_command_t command);

/**
 * @brief Update system statistics
 * @return PAPYRUS_OK on success
 */
papyrus_status_t update_system_statistics(void);

/* Configuration defaults */
#define DEFAULT_SYSTEM_ID              0x1001
#define DEFAULT_BOARD_REVISION         1
#define DEFAULT_CAN_BAUDRATE          CAN_BAUDRATE
#define DEFAULT_CAN_RETRY_COUNT       CAN_RETRY_COUNT
#define DEFAULT_CAN_TIMEOUT_MS        CAN_TIMEOUT_MS
#define DEFAULT_RADIO_FREQUENCY       RADIO_FREQUENCY
#define DEFAULT_RADIO_POWER_DBM       10
#define DEFAULT_RADIO_TIMEOUT_MS      GS_RADIO_TIMEOUT_MS
#define DEFAULT_VOLTAGE_3V3_MIN_MV    3000
#define DEFAULT_VOLTAGE_3V3_MAX_MV    3600
#define DEFAULT_VOLTAGE_5V0_MIN_MV    4500
#define DEFAULT_VOLTAGE_5V0_MAX_MV    5500
#define DEFAULT_VOLTAGE_12V_MIN_MV    10800
#define DEFAULT_VOLTAGE_12V_MAX_MV    13200
#define DEFAULT_CURRENT_LIMIT_MA      5000
#define DEFAULT_EMERGENCY_TIMEOUT_MS  EMERGENCY_STOP_HOLD_MS
#define DEFAULT_SAFE_MODE_TIMEOUT_MS  SAFE_MODE_TIMEOUT_MS
#define DEFAULT_WATCHDOG_TIMEOUT_MS   WATCHDOG_TIMEOUT_MS
#define DEFAULT_LOG_ENABLED           true
#define DEFAULT_LOG_RATE_HZ           10
#define DEFAULT_LOG_FILE_SIZE_KB      1024
#define DEFAULT_DEBUG_ENABLED         DEBUG_ENABLED
#define DEFAULT_DEBUG_LEVEL           3

#endif /* MAIN_BOARD_H */ 