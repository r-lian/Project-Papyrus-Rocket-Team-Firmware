/**
 * @file papyrus_can.h
 * @brief Papyrus CAN Bus Protocol Definitions
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef PAPYRUS_CAN_H
#define PAPYRUS_CAN_H

#include <stdint.h>
#include <stdbool.h>

/* CAN Configuration */
#define CAN_BAUDRATE                500000  // 500 kbps
#define CAN_MAX_DATA_LENGTH         8       // Standard CAN frame
#define CAN_TIMEOUT_MS              100     // Message timeout
#define CAN_RETRY_COUNT             3       // Retransmission attempts

/* CAN ID Structure (11-bit identifier)
 * Bits [10-8]: Priority (3 bits)
 * Bits [7-4]:  Source Board ID (4 bits) 
 * Bits [3-0]:  Message Type (4 bits)
 */
#define CAN_ID_PRIORITY_SHIFT       8
#define CAN_ID_PRIORITY_MASK        0x07
#define CAN_ID_SOURCE_SHIFT         4  
#define CAN_ID_SOURCE_MASK          0x0F
#define CAN_ID_MSGTYPE_SHIFT        0
#define CAN_ID_MSGTYPE_MASK         0x0F

/* Priority Levels */
typedef enum {
    CAN_PRIORITY_EMERGENCY = 0,     // Emergency stop, critical faults
    CAN_PRIORITY_CONTROL   = 1,     // Real-time control commands
    CAN_PRIORITY_DATA      = 2,     // Sensor data, status updates
    CAN_PRIORITY_DEBUG     = 3      // Debug messages, diagnostics
} can_priority_t;

/* Board IDs */
typedef enum {
    BOARD_ID_MAIN          = 0,     // Main coordination board
    BOARD_ID_SERVO_1       = 1,     // Servo controller #1
    BOARD_ID_SERVO_2       = 2,     // Servo controller #2  
    BOARD_ID_SERVO_3       = 3,     // Servo controller #3
    BOARD_ID_TC_1          = 4,     // Thermocouple controller #1
    BOARD_ID_TC_2          = 5,     // Thermocouple controller #2
    BOARD_ID_TC_3          = 6,     // Thermocouple controller #3
    BOARD_ID_IO_1          = 7,     // General I/O controller #1
    BOARD_ID_IO_2          = 8,     // General I/O controller #2
    BOARD_ID_POWER         = 9,     // Power management board
    BOARD_ID_RADIO         = 10,    // Radio communication board
    BOARD_ID_DEBUGGER      = 14,    // Bus debugger
    BOARD_ID_GROUND        = 15,    // Ground station
    BOARD_ID_INVALID       = 0xFF
} board_id_t;

/* Message Types */
typedef enum {
    MSG_TYPE_SYSTEM        = 0,     // System control (reset, config)
    MSG_TYPE_COMMAND       = 1,     // Device commands
    MSG_TYPE_DATA          = 2,     // Sensor data
    MSG_TYPE_STATUS        = 3,     // Board status reports
    MSG_TYPE_ERROR         = 4,     // Error notifications
    MSG_TYPE_CONFIG        = 5,     // Configuration messages
    MSG_TYPE_SYNC          = 6,     // Time synchronization
    MSG_TYPE_DEBUG         = 7,     // Debug information
    MSG_TYPE_HEARTBEAT     = 8,     // Periodic alive signals
    MSG_TYPE_ACK           = 9,     // Acknowledgments
    MSG_TYPE_NACK          = 10,    // Negative acknowledgments
    MSG_TYPE_RESERVED      = 15     // Reserved for future use
} msg_type_t;

/* System Commands */
typedef enum {
    SYS_CMD_RESET          = 0x01,  // Software reset
    SYS_CMD_EMERGENCY_STOP = 0x02,  // Emergency stop all
    SYS_CMD_SAFE_MODE      = 0x03,  // Enter safe mode
    SYS_CMD_NORMAL_MODE    = 0x04,  // Resume normal operation
    SYS_CMD_SLEEP          = 0x05,  // Enter low power mode
    SYS_CMD_WAKE           = 0x06,  // Wake from sleep
    SYS_CMD_IDENTIFY       = 0x07,  // Board identification request
    SYS_CMD_TIME_SYNC      = 0x08,  // Time synchronization
    SYS_CMD_CONFIG_SAVE    = 0x09,  // Save configuration to flash
    SYS_CMD_CONFIG_LOAD    = 0x0A   // Load configuration from flash
} system_command_t;

/* Error Codes */
typedef enum {
    ERROR_NONE             = 0x00,
    ERROR_COMMUNICATION    = 0x01,  // Communication timeout/failure
    ERROR_HARDWARE         = 0x02,  // Hardware malfunction
    ERROR_SENSOR           = 0x03,  // Sensor reading error
    ERROR_ACTUATOR         = 0x04,  // Actuator control error
    ERROR_POWER            = 0x05,  // Power supply issues
    ERROR_TEMPERATURE      = 0x06,  // Temperature out of range
    ERROR_OVERCURRENT      = 0x07,  // Current limit exceeded
    ERROR_WATCHDOG         = 0x08,  // Watchdog timer reset
    ERROR_MEMORY           = 0x09,  // Memory corruption/failure
    ERROR_CONFIG           = 0x0A,  // Configuration error
    ERROR_CRITICAL         = 0xFF   // Critical system failure
} error_code_t;

/* CAN Message Structure */
typedef struct {
    uint32_t id;                    // CAN identifier
    uint8_t data[CAN_MAX_DATA_LENGTH]; // Message data
    uint8_t length;                 // Data length (0-8 bytes)
    uint32_t timestamp;             // Message timestamp (ms)
} can_message_t;

/* Standard Message Formats */

/* System Message Format */
typedef struct {
    uint8_t command;                // System command
    uint8_t param1;                 // Command parameter 1
    uint8_t param2;                 // Command parameter 2
    uint8_t param3;                 // Command parameter 3
    uint32_t timestamp;             // Command timestamp
} __attribute__((packed)) system_msg_t;

/* Status Message Format */
typedef struct {
    uint8_t board_state;            // Current board state
    uint8_t error_flags;            // Error status flags
    uint16_t uptime_sec;            // Uptime in seconds
    uint16_t loop_count;            // Main loop counter
    uint8_t cpu_usage;              // CPU utilization %
    uint8_t temperature;            // Board temperature (°C)
} __attribute__((packed)) status_msg_t;

/* Servo Command Message */
typedef struct {
    uint8_t servo_mask;             // Which servos to control (bit mask)
    uint16_t position[3];           // Target positions (0-4095)
    uint8_t speed;                  // Movement speed (0-255)
    uint8_t flags;                  // Control flags
} __attribute__((packed)) servo_cmd_msg_t;

/* Thermocouple Data Message */
typedef struct {
    uint8_t tc_count;               // Number of active thermocouples
    int16_t temperatures[3];        // Temperature readings (0.1°C resolution)
    uint8_t status_flags;           // Sensor status flags
} __attribute__((packed)) tc_data_msg_t;

/* Function Prototypes */

/**
 * @brief Construct CAN identifier from components
 * @param priority Message priority level
 * @param source Source board ID  
 * @param msg_type Message type
 * @return Constructed 11-bit CAN ID
 */
uint32_t can_build_id(can_priority_t priority, board_id_t source, msg_type_t msg_type);

/**
 * @brief Extract priority from CAN ID
 * @param can_id CAN identifier
 * @return Priority level
 */
can_priority_t can_get_priority(uint32_t can_id);

/**
 * @brief Extract source board ID from CAN ID
 * @param can_id CAN identifier
 * @return Source board ID
 */
board_id_t can_get_source(uint32_t can_id);

/**
 * @brief Extract message type from CAN ID
 * @param can_id CAN identifier
 * @return Message type
 */
msg_type_t can_get_msg_type(uint32_t can_id);

/**
 * @brief Validate CAN message format
 * @param msg Pointer to CAN message
 * @return true if valid, false otherwise
 */
bool can_validate_message(const can_message_t* msg);

/**
 * @brief Calculate CRC8 checksum for message data
 * @param data Pointer to data
 * @param length Data length
 * @return CRC8 checksum
 */
uint8_t can_calculate_crc8(const uint8_t* data, uint8_t length);

#endif /* PAPYRUS_CAN_H */ 