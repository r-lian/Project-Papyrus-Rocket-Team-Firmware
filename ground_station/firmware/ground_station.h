/**
 * @file ground_station.h
 * @brief Papyrus Ground Station Firmware
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include <stdint.h>
#include <stdbool.h>
#include "papyrus_config.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"

/* Ground Station Configuration */
typedef struct {
    /* Radio Configuration */
    uint32_t radio_frequency;
    uint16_t radio_power_dbm;
    uint8_t radio_channel;
    uint16_t radio_timeout_ms;
    
    /* Network Configuration */
    uint16_t web_port;
    uint16_t websocket_port;
    uint8_t max_clients;
    char wifi_ssid[32];
    char wifi_password[32];
    
    /* Data Logging */
    bool logging_enabled;
    uint32_t log_file_size_mb;
    char log_directory[64];
    
    /* Emergency Controls */
    bool emergency_stop_enabled;
    uint8_t emergency_button_pin;
    uint16_t emergency_timeout_ms;
    
    /* Configuration checksum */
    uint32_t config_crc;
} ground_station_config_t;

/* Ground Station Status */
typedef struct {
    /* System Status */
    system_state_t system_state;
    uint32_t uptime_sec;
    uint8_t cpu_usage_percent;
    int8_t temperature_c;
    
    /* Radio Status */
    bool radio_connected;
    uint16_t radio_signal_strength;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packet_errors;
    uint32_t last_packet_time;
    
    /* Network Status */
    bool wifi_connected;
    bool web_server_running;
    uint8_t active_clients;
    uint32_t web_requests_served;
    
    /* Main Board Communication */
    bool main_board_connected;
    uint32_t last_main_board_comm;
    uint16_t command_responses;
    uint16_t telemetry_packets;
    
    /* Data Logging Status */
    bool logging_active;
    uint32_t log_entries;
    uint32_t log_file_size_bytes;
    uint16_t log_errors;
    
    /* Emergency Status */
    bool emergency_stop_active;
    uint32_t emergency_stop_time;
    
} ground_station_status_t;

/* Telemetry Data Structure */
typedef struct {
    uint32_t timestamp;
    uint8_t source_board;
    uint8_t data_type;
    uint16_t data_length;
    uint8_t data[256];
} telemetry_packet_t;

/* Command Structure */
typedef struct {
    uint32_t timestamp;
    uint8_t target_board;
    uint8_t command_type;
    uint16_t command_data_length;
    uint8_t command_data[64];
    bool response_expected;
    uint32_t timeout_ms;
} command_packet_t;

/* Web Client Structure */
typedef struct {
    uint8_t client_id;
    bool connected;
    uint32_t connect_time;
    uint32_t last_activity;
    bool operator_mode;
    char client_ip[16];
} web_client_t;

/* Function Prototypes - Ground Station Main Functions */

/**
 * @brief Initialize ground station
 * @return PAPYRUS_OK on success
 */
papyrus_status_t ground_station_init(void);

/**
 * @brief Main ground station processing loop
 * @return PAPYRUS_OK on success
 */
papyrus_status_t ground_station_process(void);

/**
 * @brief Shutdown ground station
 * @return PAPYRUS_OK on success
 */
papyrus_status_t ground_station_shutdown(void);

/**
 * @brief Get ground station status
 * @return Pointer to status structure
 */
ground_station_status_t* ground_station_get_status(void);

/**
 * @brief Get ground station configuration
 * @return Pointer to configuration structure
 */
ground_station_config_t* ground_station_get_config(void);

/* Function Prototypes - Radio Communication */

/**
 * @brief Initialize radio communication
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_radio_init(void);

/**
 * @brief Send packet to main board
 * @param data Pointer to data to send
 * @param length Data length
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_radio_send(const uint8_t* data, uint16_t length);

/**
 * @brief Receive packet from main board
 * @param data Buffer to store received data
 * @param max_length Maximum buffer length
 * @param received_length Pointer to store actual received length
 * @return PAPYRUS_OK if packet received
 */
papyrus_status_t gs_radio_receive(uint8_t* data, 
                                 uint16_t max_length, 
                                 uint16_t* received_length);

/**
 * @brief Check radio connection status
 * @return true if radio is connected
 */
bool gs_radio_is_connected(void);

/**
 * @brief Get radio signal strength
 * @return Signal strength (0-100)
 */
uint8_t gs_radio_get_signal_strength(void);

/**
 * @brief Send command to main board
 * @param command Pointer to command packet
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_send_command(const command_packet_t* command);

/**
 * @brief Process received telemetry
 * @param packet Pointer to telemetry packet
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_process_telemetry(const telemetry_packet_t* packet);

/* Function Prototypes - Web Interface */

/**
 * @brief Initialize web server
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_init(void);

/**
 * @brief Start web server
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_start(void);

/**
 * @brief Stop web server
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_stop(void);

/**
 * @brief Process web requests
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_process(void);

/**
 * @brief Handle client connection
 * @param client_id Client identifier
 * @param client_ip Client IP address
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_client_connect(uint8_t client_id, const char* client_ip);

/**
 * @brief Handle client disconnection
 * @param client_id Client identifier
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_client_disconnect(uint8_t client_id);

/**
 * @brief Send data to web client
 * @param client_id Client identifier
 * @param data Pointer to data
 * @param length Data length
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_send_to_client(uint8_t client_id, 
                                      const uint8_t* data, 
                                      uint16_t length);

/**
 * @brief Broadcast data to all clients
 * @param data Pointer to data
 * @param length Data length
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_broadcast(const uint8_t* data, uint16_t length);

/**
 * @brief Set client operator mode
 * @param client_id Client identifier
 * @param operator_mode True to enable operator mode
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_web_set_operator_mode(uint8_t client_id, bool operator_mode);

/* Function Prototypes - Data Logging */

/**
 * @brief Initialize data logging
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_log_init(void);

/**
 * @brief Start data logging
 * @param filename Log filename
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_log_start(const char* filename);

/**
 * @brief Stop data logging
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_log_stop(void);

/**
 * @brief Log telemetry data
 * @param packet Pointer to telemetry packet
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_log_telemetry(const telemetry_packet_t* packet);

/**
 * @brief Log command data
 * @param command Pointer to command packet
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_log_command(const command_packet_t* command);

/**
 * @brief Log system event
 * @param event_type Event type
 * @param description Event description
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_log_event(uint8_t event_type, const char* description);

/**
 * @brief Create new log file
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_log_create_new_file(void);

/* Function Prototypes - Emergency Controls */

/**
 * @brief Initialize emergency controls
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_emergency_init(void);

/**
 * @brief Trigger emergency stop
 * @param reason Emergency stop reason
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_emergency_stop(const char* reason);

/**
 * @brief Clear emergency stop
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_emergency_clear(void);

/**
 * @brief Check emergency button status
 * @return true if emergency button is pressed
 */
bool gs_emergency_button_pressed(void);

/**
 * @brief Send emergency command to main board
 * @param command Emergency command
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_send_emergency_command(system_command_t command);

/* Function Prototypes - System Commands */

/**
 * @brief Send reset command to main board
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_cmd_reset_system(void);

/**
 * @brief Send safe mode command
 * @param enter True to enter safe mode, false to exit
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_cmd_safe_mode(bool enter);

/**
 * @brief Request system status from main board
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_cmd_request_status(void);

/**
 * @brief Send servo position command
 * @param board_id Target servo controller board
 * @param servo_id Servo identifier
 * @param position Target position
 * @param speed Movement speed
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_cmd_servo_position(board_id_t board_id, 
                                      uint8_t servo_id,
                                      uint16_t position, 
                                      uint8_t speed);

/**
 * @brief Request temperature readings
 * @param board_id Target thermocouple controller board
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_cmd_request_temperatures(board_id_t board_id);

/**
 * @brief Send configuration update
 * @param board_id Target board
 * @param config_data Configuration data
 * @param data_length Data length
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_cmd_update_config(board_id_t board_id, 
                                     const uint8_t* config_data,
                                     uint16_t data_length);

/* Function Prototypes - Configuration Management */

/**
 * @brief Load ground station configuration
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_config_load(void);

/**
 * @brief Save ground station configuration
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_config_save(void);

/**
 * @brief Load default configuration
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_config_default(void);

/**
 * @brief Validate configuration
 * @return PAPYRUS_OK if valid
 */
papyrus_status_t gs_config_validate(void);

/* Function Prototypes - Status and Monitoring */

/**
 * @brief Update ground station status
 * @return PAPYRUS_OK on success
 */
papyrus_status_t gs_update_status(void);

/**
 * @brief Check system health
 * @return true if all systems healthy
 */
bool gs_is_healthy(void);

/**
 * @brief Get main board connection status
 * @return true if main board is connected
 */
bool gs_main_board_connected(void);

/**
 * @brief Get last communication time with main board
 * @return Timestamp of last communication
 */
uint32_t gs_get_last_comm_time(void);

/**
 * @brief Generate status report
 * @param buffer Buffer to store status report
 * @param buffer_size Buffer size
 * @return Length of status report
 */
uint16_t gs_generate_status_report(char* buffer, uint16_t buffer_size);

/* Utility Functions */

/**
 * @brief Convert command to string
 * @param command Command type
 * @return String description
 */
const char* gs_command_to_string(uint8_t command);

/**
 * @brief Get current timestamp
 * @return Current timestamp in milliseconds
 */
uint32_t gs_get_timestamp(void);

/**
 * @brief Format telemetry data for display
 * @param packet Pointer to telemetry packet
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @return Formatted string length
 */
uint16_t gs_format_telemetry(const telemetry_packet_t* packet, 
                            char* buffer, 
                            uint16_t buffer_size);

/* Default Configuration Values */
#define GS_DEFAULT_RADIO_FREQUENCY         433000000
#define GS_DEFAULT_RADIO_POWER_DBM         10
#define GS_DEFAULT_RADIO_CHANNEL           1
#define GS_DEFAULT_RADIO_TIMEOUT_MS        5000
#define GS_DEFAULT_WEB_PORT               8080
#define GS_DEFAULT_WEBSOCKET_PORT         8081
#define GS_DEFAULT_MAX_CLIENTS            5
#define GS_DEFAULT_LOG_FILE_SIZE_MB       100
#define GS_DEFAULT_EMERGENCY_TIMEOUT_MS   1000

/* Command Types */
#define GS_CMD_SYSTEM_RESET               0x01
#define GS_CMD_EMERGENCY_STOP             0x02
#define GS_CMD_SAFE_MODE_ENTER            0x03
#define GS_CMD_SAFE_MODE_EXIT             0x04
#define GS_CMD_REQUEST_STATUS             0x05
#define GS_CMD_SERVO_POSITION             0x10
#define GS_CMD_SERVO_ENABLE               0x11
#define GS_CMD_SERVO_DISABLE              0x12
#define GS_CMD_TC_READ_TEMP               0x20
#define GS_CMD_TC_READ_ALL                0x21
#define GS_CMD_CONFIG_UPDATE              0x30

/* Data Types */
#define GS_DATA_SYSTEM_STATUS             0x01
#define GS_DATA_SERVO_POSITION            0x10
#define GS_DATA_SERVO_STATUS              0x11
#define GS_DATA_TC_TEMPERATURE            0x20
#define GS_DATA_TC_STATUS                 0x21
#define GS_DATA_ERROR_LOG                 0x30

#endif /* GROUND_STATION_H */ 