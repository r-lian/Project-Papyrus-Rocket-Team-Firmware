/**
 * @file papyrus_utils.h
 * @brief Papyrus Common Utility Functions
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef PAPYRUS_UTILS_H
#define PAPYRUS_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "papyrus_config.h"
#include "papyrus_can.h"

/* Return Status Codes */
typedef enum {
    PAPYRUS_OK = 0,
    PAPYRUS_ERROR = -1,
    PAPYRUS_ERROR_INVALID_PARAM = -2,
    PAPYRUS_ERROR_TIMEOUT = -3,
    PAPYRUS_ERROR_NO_MEMORY = -4,
    PAPYRUS_ERROR_BUSY = -5,
    PAPYRUS_ERROR_NOT_READY = -6,
    PAPYRUS_ERROR_COMMUNICATION = -7,
    PAPYRUS_ERROR_HARDWARE = -8,
    PAPYRUS_ERROR_SAFETY = -9
} papyrus_status_t;

/* System State Definitions */
typedef enum {
    SYSTEM_STATE_INIT = 0,
    SYSTEM_STATE_NORMAL,
    SYSTEM_STATE_SAFE_MODE,
    SYSTEM_STATE_EMERGENCY_STOP,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_SLEEP,
    SYSTEM_STATE_MAINTENANCE
} system_state_t;

/* Board Health Status */
typedef struct {
    system_state_t state;
    uint8_t error_flags;
    uint16_t uptime_sec;
    uint8_t cpu_usage_percent;
    int8_t temperature_c;
    uint16_t free_memory_kb;
    uint32_t error_count;
    uint32_t last_error_code;
} board_health_t;

/* Error Log Entry */
typedef struct {
    uint32_t timestamp;
    error_code_t error_code;
    board_id_t source_board;
    uint16_t error_data;
    uint8_t severity;
    char description[32];
} error_log_entry_t;

/* System Statistics */
typedef struct {
    uint32_t can_messages_sent;
    uint32_t can_messages_received;
    uint32_t can_errors;
    uint32_t system_resets;
    uint32_t emergency_stops;
    uint32_t safe_mode_entries;
    uint32_t total_runtime_sec;
} system_stats_t;

/* Function Prototypes - Utility Functions */

/**
 * @brief Initialize common utility systems
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_utils_init(void);

/**
 * @brief Get current system timestamp in milliseconds
 * @return Current timestamp
 */
uint32_t papyrus_get_timestamp_ms(void);

/**
 * @brief Get current system timestamp in seconds
 * @return Current timestamp in seconds
 */
uint32_t papyrus_get_timestamp_sec(void);

/**
 * @brief Delay for specified milliseconds
 * @param ms Delay time in milliseconds
 */
void papyrus_delay_ms(uint32_t ms);

/**
 * @brief Convert status code to human-readable string
 * @param status Status code
 * @return String description
 */
const char* papyrus_status_to_string(papyrus_status_t status);

/**
 * @brief Convert system state to human-readable string
 * @param state System state
 * @return String description
 */
const char* papyrus_state_to_string(system_state_t state);

/* Function Prototypes - Error Handling */

/**
 * @brief Initialize error handling system
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_error_init(void);

/**
 * @brief Log an error event
 * @param error_code Error code
 * @param source_board Source board ID
 * @param error_data Additional error data
 * @param description Error description string
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_log_error(error_code_t error_code, 
                                  board_id_t source_board,
                                  uint16_t error_data, 
                                  const char* description);

/**
 * @brief Get the most recent error log entry
 * @param entry Pointer to store error entry
 * @return PAPYRUS_OK if entry available
 */
papyrus_status_t papyrus_get_last_error(error_log_entry_t* entry);

/**
 * @brief Get error log entry by index
 * @param index Error log index
 * @param entry Pointer to store error entry
 * @return PAPYRUS_OK if entry exists
 */
papyrus_status_t papyrus_get_error_by_index(uint16_t index, error_log_entry_t* entry);

/**
 * @brief Clear all error log entries
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_clear_error_log(void);

/**
 * @brief Get total number of logged errors
 * @return Number of error log entries
 */
uint16_t papyrus_get_error_count(void);

/* Function Prototypes - Safety Functions */

/**
 * @brief Initialize safety monitoring system
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_safety_init(void);

/**
 * @brief Trigger emergency stop sequence
 * @param reason Emergency stop reason code
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_emergency_stop(error_code_t reason);

/**
 * @brief Enter safe mode
 * @param reason Safe mode entry reason
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_enter_safe_mode(error_code_t reason);

/**
 * @brief Exit safe mode and return to normal operation
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_exit_safe_mode(void);

/**
 * @brief Check if system is in emergency stop state
 * @return true if in emergency stop
 */
bool papyrus_is_emergency_stop(void);

/**
 * @brief Check if system is in safe mode
 * @return true if in safe mode
 */
bool papyrus_is_safe_mode(void);

/**
 * @brief Get current system state
 * @return Current system state
 */
system_state_t papyrus_get_system_state(void);

/**
 * @brief Reset system from error state
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_system_reset(void);

/* Function Prototypes - Health Monitoring */

/**
 * @brief Initialize health monitoring system
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_health_init(void);

/**
 * @brief Update board health status
 * @param health Pointer to health status structure
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_update_health(const board_health_t* health);

/**
 * @brief Get current board health status
 * @param health Pointer to store health status
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_get_health(board_health_t* health);

/**
 * @brief Check if board health is within acceptable limits
 * @return true if healthy, false if unhealthy
 */
bool papyrus_is_board_healthy(void);

/**
 * @brief Get system statistics
 * @param stats Pointer to store statistics
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_get_statistics(system_stats_t* stats);

/**
 * @brief Reset system statistics
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_reset_statistics(void);

/* Function Prototypes - Memory Management */

/**
 * @brief Initialize memory management system
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_memory_init(void);

/**
 * @brief Allocate memory with error checking
 * @param size Size in bytes to allocate
 * @return Pointer to allocated memory, NULL on failure
 */
void* papyrus_malloc(size_t size);

/**
 * @brief Free allocated memory
 * @param ptr Pointer to memory to free
 */
void papyrus_free(void* ptr);

/**
 * @brief Get available free memory
 * @return Free memory in bytes
 */
size_t papyrus_get_free_memory(void);

/**
 * @brief Check for memory leaks
 * @return Number of unfreed allocations
 */
uint16_t papyrus_check_memory_leaks(void);

/* Function Prototypes - Watchdog Management */

/**
 * @brief Initialize watchdog timer
 * @param timeout_ms Watchdog timeout in milliseconds
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_watchdog_init(uint32_t timeout_ms);

/**
 * @brief Feed the watchdog timer
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_watchdog_feed(void);

/**
 * @brief Enable watchdog timer
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_watchdog_enable(void);

/**
 * @brief Disable watchdog timer (for debugging only)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_watchdog_disable(void);

/* Function Prototypes - Configuration Management */

/**
 * @brief Load configuration from flash memory
 * @param config_id Configuration identifier
 * @param data Pointer to store configuration data
 * @param size Size of configuration data
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_config_load(uint16_t config_id, void* data, size_t size);

/**
 * @brief Save configuration to flash memory
 * @param config_id Configuration identifier
 * @param data Pointer to configuration data
 * @param size Size of configuration data
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_config_save(uint16_t config_id, const void* data, size_t size);

/**
 * @brief Reset configuration to defaults
 * @param config_id Configuration identifier
 * @return PAPYRUS_OK on success
 */
papyrus_status_t papyrus_config_reset(uint16_t config_id);

/* Debug and Logging Macros */
#if DEBUG_ENABLED
    #define PAPYRUS_LOG_DEBUG(fmt, ...) printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
    #define PAPYRUS_LOG_INFO(fmt, ...)  printf("[INFO]  " fmt "\r\n", ##__VA_ARGS__)
    #define PAPYRUS_LOG_WARN(fmt, ...)  printf("[WARN]  " fmt "\r\n", ##__VA_ARGS__)
    #define PAPYRUS_LOG_ERROR(fmt, ...) printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)
    #define PAPYRUS_ASSERT(condition) \
        do { \
            if (!(condition)) { \
                printf("[ASSERT] %s:%d - %s\r\n", __FILE__, __LINE__, #condition); \
                papyrus_emergency_stop(ERROR_CRITICAL); \
            } \
        } while(0)
#else
    #define PAPYRUS_LOG_DEBUG(fmt, ...)
    #define PAPYRUS_LOG_INFO(fmt, ...)
    #define PAPYRUS_LOG_WARN(fmt, ...)
    #define PAPYRUS_LOG_ERROR(fmt, ...)
    #define PAPYRUS_ASSERT(condition)
#endif

/* Utility Macros */
#define PAPYRUS_MIN(a, b)           ((a) < (b) ? (a) : (b))
#define PAPYRUS_MAX(a, b)           ((a) > (b) ? (a) : (b))
#define PAPYRUS_ABS(x)              ((x) < 0 ? -(x) : (x))
#define PAPYRUS_CLAMP(x, min, max)  (PAPYRUS_MIN(PAPYRUS_MAX((x), (min)), (max)))
#define PAPYRUS_ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))
#define PAPYRUS_UNUSED(x)           ((void)(x))

/* Bit manipulation macros */
#define PAPYRUS_SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))
#define PAPYRUS_CLEAR_BIT(reg, bit) ((reg) &= ~(1U << (bit)))
#define PAPYRUS_TOGGLE_BIT(reg, bit) ((reg) ^= (1U << (bit)))
#define PAPYRUS_READ_BIT(reg, bit)  (((reg) >> (bit)) & 1U)

#endif /* PAPYRUS_UTILS_H */ 