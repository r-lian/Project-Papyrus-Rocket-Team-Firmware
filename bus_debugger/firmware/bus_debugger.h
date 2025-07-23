/**
 * @file bus_debugger.h
 * @brief Papyrus CAN Bus Debugger Firmware
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef BUS_DEBUGGER_H
#define BUS_DEBUGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "papyrus_config.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"

/* Bus Debugger Configuration */
typedef struct {
    /* CAN Configuration */
    uint32_t can_baudrate;
    bool can_silent_mode;
    bool can_loop_back;
    
    /* Display Configuration */
    uint8_t display_rows;
    uint8_t display_cols;
    uint16_t display_timeout_ms;
    bool display_auto_scroll;
    
    /* Logging Configuration */
    bool logging_enabled;
    uint32_t log_buffer_size;
    uint16_t log_retention_hours;
    
    /* Filter Configuration */
    bool filter_enabled;
    uint32_t filter_mask;
    uint32_t filter_id;
    board_id_t filter_board_id;
    msg_type_t filter_msg_type;
    
    /* Trigger Configuration */
    bool trigger_enabled;
    uint32_t trigger_id;
    uint8_t trigger_data[8];
    uint8_t trigger_mask[8];
    
    /* Power Configuration */
    bool battery_powered;
    uint16_t sleep_timeout_min;
    uint8_t backlight_brightness;
    
    /* Configuration checksum */
    uint32_t config_crc;
} bus_debugger_config_t;

/* Bus Debugger Status */
typedef struct {
    /* System Status */
    system_state_t system_state;
    uint32_t uptime_sec;
    uint8_t battery_percent;
    bool usb_connected;
    
    /* CAN Bus Status */
    bool can_bus_active;
    uint32_t can_bus_load_percent;
    uint32_t total_messages;
    uint32_t messages_per_second;
    uint32_t error_frames;
    uint32_t bus_off_events;
    
    /* Message Statistics by Board */
    uint32_t messages_by_board[16];
    uint32_t errors_by_board[16];
    uint32_t last_message_time[16];
    
    /* Message Statistics by Type */
    uint32_t messages_by_type[16];
    uint32_t message_rates[16];
    
    /* Capture Status */
    bool capturing;
    uint32_t captured_messages;
    uint32_t capture_start_time;
    uint32_t trigger_count;
    
    /* Memory Status */
    uint32_t log_buffer_used;
    uint16_t log_entries;
    
} bus_debugger_status_t;

/* Message Log Entry */
typedef struct {
    uint32_t timestamp;
    can_message_t message;
    uint8_t bus_state;
    uint8_t error_flags;
} message_log_entry_t;

/* Display Menu States */
typedef enum {
    MENU_MAIN = 0,
    MENU_LIVE_MONITOR,
    MENU_MESSAGE_LOG,
    MENU_STATISTICS,
    MENU_FILTERS,
    MENU_TRIGGERS,
    MENU_SETTINGS,
    MENU_DIAGNOSTICS
} menu_state_t;

/* Button States */
typedef enum {
    BUTTON_NONE = 0,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_SELECT,
    BUTTON_BACK
} button_state_t;

/* Bus Debugger Class */
typedef struct {
    /* Configuration and Status */
    bus_debugger_config_t config;
    bus_debugger_status_t status;
    
    /* Hardware Interfaces */
    CAN_HandleTypeDef* hcan;
    I2C_HandleTypeDef* hi2c_display;
    ADC_HandleTypeDef* hadc_battery;
    
    /* User Interface */
    menu_state_t current_menu;
    uint8_t menu_selection;
    uint8_t scroll_position;
    bool display_active;
    uint32_t last_user_input;
    
    /* Message Logging */
    message_log_entry_t* log_buffer;
    uint32_t log_head;
    uint32_t log_tail;
    bool log_overflow;
    
    /* Statistics Tracking */
    uint32_t stats_start_time;
    uint32_t last_stats_update;
    uint16_t message_rate_buffer[60]; /* 1-minute rolling buffer */
    uint8_t rate_buffer_index;
    
} bus_debugger_t;

/* Function Prototypes - Bus Debugger Main Functions */

/**
 * @brief Initialize bus debugger
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bus_debugger_init(bus_debugger_t* debugger);

/**
 * @brief Main bus debugger processing loop
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bus_debugger_process(bus_debugger_t* debugger);

/**
 * @brief Shutdown bus debugger
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bus_debugger_shutdown(bus_debugger_t* debugger);

/* Function Prototypes - CAN Bus Monitoring */

/**
 * @brief Initialize CAN bus monitoring
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_can_init(bus_debugger_t* debugger);

/**
 * @brief Process received CAN message
 * @param debugger Pointer to bus debugger instance
 * @param message Pointer to received message
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_can_process_message(bus_debugger_t* debugger, 
                                       const can_message_t* message);

/**
 * @brief Send test message on CAN bus
 * @param debugger Pointer to bus debugger instance
 * @param message Pointer to message to send
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_can_send_test_message(bus_debugger_t* debugger, 
                                         const can_message_t* message);

/**
 * @brief Monitor CAN bus health
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_can_monitor_health(bus_debugger_t* debugger);

/**
 * @brief Calculate bus load percentage
 * @param debugger Pointer to bus debugger instance
 * @return Bus load percentage (0-100)
 */
uint8_t bd_can_calculate_bus_load(bus_debugger_t* debugger);

/* Function Prototypes - Message Logging */

/**
 * @brief Initialize message logging
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_log_init(bus_debugger_t* debugger);

/**
 * @brief Log CAN message
 * @param debugger Pointer to bus debugger instance
 * @param message Pointer to message to log
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_log_message(bus_debugger_t* debugger, 
                               const can_message_t* message);

/**
 * @brief Get log entry by index
 * @param debugger Pointer to bus debugger instance
 * @param index Log entry index
 * @param entry Pointer to store log entry
 * @return PAPYRUS_OK if entry exists
 */
papyrus_status_t bd_log_get_entry(bus_debugger_t* debugger, 
                                 uint32_t index,
                                 message_log_entry_t* entry);

/**
 * @brief Clear message log
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_log_clear(bus_debugger_t* debugger);

/**
 * @brief Export log to file
 * @param debugger Pointer to bus debugger instance
 * @param filename Output filename
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_log_export(bus_debugger_t* debugger, const char* filename);

/* Function Prototypes - Statistics */

/**
 * @brief Update message statistics
 * @param debugger Pointer to bus debugger instance
 * @param message Pointer to message
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_stats_update(bus_debugger_t* debugger, 
                                const can_message_t* message);

/**
 * @brief Reset all statistics
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_stats_reset(bus_debugger_t* debugger);

/**
 * @brief Get message rate for specific board
 * @param debugger Pointer to bus debugger instance
 * @param board_id Board identifier
 * @return Messages per second
 */
uint16_t bd_stats_get_board_rate(bus_debugger_t* debugger, board_id_t board_id);

/**
 * @brief Get overall bus utilization
 * @param debugger Pointer to bus debugger instance
 * @return Bus utilization percentage
 */
uint8_t bd_stats_get_bus_utilization(bus_debugger_t* debugger);

/* Function Prototypes - Filtering */

/**
 * @brief Set message filter
 * @param debugger Pointer to bus debugger instance
 * @param filter_id CAN ID filter
 * @param filter_mask CAN ID mask
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_filter_set_id(bus_debugger_t* debugger, 
                                 uint32_t filter_id, 
                                 uint32_t filter_mask);

/**
 * @brief Set board filter
 * @param debugger Pointer to bus debugger instance
 * @param board_id Board to filter
 * @param enabled True to enable filter
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_filter_set_board(bus_debugger_t* debugger, 
                                    board_id_t board_id, 
                                    bool enabled);

/**
 * @brief Set message type filter
 * @param debugger Pointer to bus debugger instance
 * @param msg_type Message type to filter
 * @param enabled True to enable filter
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_filter_set_type(bus_debugger_t* debugger, 
                                   msg_type_t msg_type, 
                                   bool enabled);

/**
 * @brief Check if message passes filter
 * @param debugger Pointer to bus debugger instance
 * @param message Pointer to message
 * @return true if message passes filter
 */
bool bd_filter_check_message(bus_debugger_t* debugger, 
                            const can_message_t* message);

/**
 * @brief Clear all filters
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_filter_clear_all(bus_debugger_t* debugger);

/* Function Prototypes - Triggers */

/**
 * @brief Set trigger condition
 * @param debugger Pointer to bus debugger instance
 * @param trigger_id CAN ID to trigger on
 * @param trigger_data Data pattern to match
 * @param trigger_mask Data mask
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_trigger_set(bus_debugger_t* debugger, 
                               uint32_t trigger_id,
                               const uint8_t* trigger_data, 
                               const uint8_t* trigger_mask);

/**
 * @brief Check if message matches trigger
 * @param debugger Pointer to bus debugger instance
 * @param message Pointer to message
 * @return true if message triggers
 */
bool bd_trigger_check(bus_debugger_t* debugger, const can_message_t* message);

/**
 * @brief Enable/disable trigger
 * @param debugger Pointer to bus debugger instance
 * @param enabled True to enable trigger
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_trigger_enable(bus_debugger_t* debugger, bool enabled);

/**
 * @brief Clear trigger condition
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_trigger_clear(bus_debugger_t* debugger);

/* Function Prototypes - Display and User Interface */

/**
 * @brief Initialize display
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_init(bus_debugger_t* debugger);

/**
 * @brief Update display content
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_update(bus_debugger_t* debugger);

/**
 * @brief Display main menu
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_main_menu(bus_debugger_t* debugger);

/**
 * @brief Display live message monitor
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_live_monitor(bus_debugger_t* debugger);

/**
 * @brief Display message log
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_message_log(bus_debugger_t* debugger);

/**
 * @brief Display statistics
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_statistics(bus_debugger_t* debugger);

/**
 * @brief Display settings menu
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_settings(bus_debugger_t* debugger);

/**
 * @brief Clear display
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_clear(bus_debugger_t* debugger);

/**
 * @brief Set display backlight
 * @param debugger Pointer to bus debugger instance
 * @param brightness Brightness level (0-100)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_display_set_backlight(bus_debugger_t* debugger, 
                                         uint8_t brightness);

/* Function Prototypes - User Input */

/**
 * @brief Initialize user input (buttons)
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_input_init(bus_debugger_t* debugger);

/**
 * @brief Read button input
 * @param debugger Pointer to bus debugger instance
 * @return Button state
 */
button_state_t bd_input_read_buttons(bus_debugger_t* debugger);

/**
 * @brief Process user input
 * @param debugger Pointer to bus debugger instance
 * @param button Button pressed
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_input_process(bus_debugger_t* debugger, button_state_t button);

/**
 * @brief Handle menu navigation
 * @param debugger Pointer to bus debugger instance
 * @param button Button pressed
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_input_navigate_menu(bus_debugger_t* debugger, 
                                       button_state_t button);

/* Function Prototypes - Power Management */

/**
 * @brief Initialize power management
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_power_init(bus_debugger_t* debugger);

/**
 * @brief Read battery level
 * @param debugger Pointer to bus debugger instance
 * @return Battery percentage (0-100)
 */
uint8_t bd_power_read_battery(bus_debugger_t* debugger);

/**
 * @brief Enter sleep mode
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_power_sleep(bus_debugger_t* debugger);

/**
 * @brief Wake from sleep mode
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_power_wake(bus_debugger_t* debugger);

/**
 * @brief Check if USB is connected
 * @param debugger Pointer to bus debugger instance
 * @return true if USB connected
 */
bool bd_power_usb_connected(bus_debugger_t* debugger);

/* Function Prototypes - Configuration */

/**
 * @brief Load configuration from flash
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_config_load(bus_debugger_t* debugger);

/**
 * @brief Save configuration to flash
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_config_save(bus_debugger_t* debugger);

/**
 * @brief Load default configuration
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t bd_config_default(bus_debugger_t* debugger);

/**
 * @brief Validate configuration
 * @param debugger Pointer to bus debugger instance
 * @return PAPYRUS_OK if valid
 */
papyrus_status_t bd_config_validate(bus_debugger_t* debugger);

/* Utility Functions */

/**
 * @brief Format message for display
 * @param message Pointer to CAN message
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @return Formatted string length
 */
uint16_t bd_format_message(const can_message_t* message, 
                          char* buffer, 
                          uint16_t buffer_size);

/**
 * @brief Format timestamp for display
 * @param timestamp Timestamp in milliseconds
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @return Formatted string length
 */
uint16_t bd_format_timestamp(uint32_t timestamp, 
                            char* buffer, 
                            uint16_t buffer_size);

/**
 * @brief Convert button state to string
 * @param button Button state
 * @return String description
 */
const char* bd_button_to_string(button_state_t button);

/**
 * @brief Convert menu state to string
 * @param menu Menu state
 * @return String description
 */
const char* bd_menu_to_string(menu_state_t menu);

/* Default Configuration Values */
#define BD_DEFAULT_CAN_BAUDRATE           500000
#define BD_DEFAULT_DISPLAY_ROWS           4
#define BD_DEFAULT_DISPLAY_COLS           20
#define BD_DEFAULT_DISPLAY_TIMEOUT_MS     30000
#define BD_DEFAULT_LOG_BUFFER_SIZE        1000
#define BD_DEFAULT_LOG_RETENTION_HOURS    24
#define BD_DEFAULT_SLEEP_TIMEOUT_MIN      10
#define BD_DEFAULT_BACKLIGHT_BRIGHTNESS   50

/* Display Constants */
#define BD_DISPLAY_LINE_LENGTH            20
#define BD_DISPLAY_MAX_LINES              4
#define BD_MENU_ITEMS_PER_PAGE            3

#endif /* BUS_DEBUGGER_H */ 