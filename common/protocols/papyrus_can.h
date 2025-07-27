/**
 * @file papyrus_can.h
 * @brief Papyrus CAN Bus Protocol & Control
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef PAPYRUS_CAN_H
#define PAPYRUS_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef STM32C092xx
#include "stm32c0xx/stm32c0xx_hal_fdcan.h"
#else
/* Placeholder definitions */
typedef struct {
  uint32_t Identifier;
  uint32_t IdType;
  uint32_t TxFrameType;
  uint32_t DataLength;
  uint32_t ErrorStateIndicator;
  uint32_t BitRateSwitch;
  uint32_t FDFormat;
  uint32_t TxEventFifoControl;
  uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
  uint32_t Identifier;
  uint32_t IdType;
  uint32_t RxFrameType;
  uint32_t DataLength;
  uint32_t ErrorStateIndicator;
  uint32_t BitRateSwitch;
  uint32_t FDFormat;
  uint32_t RxTimestamp;
  uint32_t FilterIndex;
  uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;
#endif

/* CAN Configuration */
#define CAN_BAUDRATE 500000   // 500 kbps
#define CAN_MAX_DATA_LENGTH 8 // Standard CAN frame
#define CAN_TIMEOUT_MS 100    // Message timeout
#define CAN_RETRY_COUNT 3     // Retransmission attempts

/* CAN ID Structure (11-bit identifier)
 * Bits [10-8]: Message type (3 bits)
 * Bits [7-0]: Controller ID (8 bits)
 */

#define CAN_CONTROLLER_ID(id) (board_id_t)((id) & 0xFF)
#define CAN_MESSAGE_TYPE(id) (msg_type_t)(((id) >> 8) & 0x07)

/* Message Types (Priority Levels) */
typedef enum {
  MSG_TYPE_EMERGENCY, // Emergency priority commands
  MSG_TYPE_ERROR,     // Critical error notifications
  MSG_TYPE_PRIORITY,  // High priority commands
  MSG_TYPE_COMMAND,   // Standard commands
  MSG_TYPE_NOTIF,     // Status notifications
  MSG_TYPE_RESPONSE,  // Responses to commands
  MSG_TYPE_STREAM,    // Data streams
  MSG_TYPE_BULKDATA,  // Bulk data transfer blocks
} MsgType;

/* Board IDs */
typedef uint8_t BoardID;

/* System Commands (Block 0)*/
typedef enum {
  SYSCMD_PING = 0x00,
  SYSCMD_QUERY_TYPE = 0x01,
  SYSCMD_REASSIGN_ID = 0x02,
  SYSCMD_RAW_IO = 0x03,
  SYSCMD_RESET = 0x04,
  SYSCMD_NAME = 0x05,
  SYSCMD_NOTIFICATION = 0x06,
  SYSCMD_SUPPORTED = 0x07,
  SYSCMD_GET_ERROR = 0x08,
  SYSCMD_STATE = 0x09,
} SystemCommand;

/* Generic Sensor Commands (Block 1)*/
typedef enum {
  SENSOR_READ = 0x10,
  SENSOR_SUBSCRIBE = 0x11,
  SENSOR_CONFIG = 0x12,
} SensorCommand;

/* Error Codes */
typedef enum {
  ERROR_NONE,             // No error
  ERROR_UNKNOWN,          // Unknown error
  ERROR_CRITICAL,         // Unknown critical error
  ERROR_CAN_BUS,          // CAN bus failure
  ERROR_HARDWARE,         // Hardware configuration error
  ERROR_MEMORY,           // Internal memory problem
  ERROR_CONTROLLER_POWER, // Controller power supply problem
  ERROR_DEVICE_POWER,     // Device power supply problem
  ERROR_HIGH_LIMIT,       // Sensor exceeded high limit point
  ERROR_LOW_LIMIT,        // Sensor exceeded low limit point
  ERROR_TOO_HIGH,         // Sensor or actuator impossibly high
  ERROR_TOO_LOW,          // Sensor or actuator impossibly low
  ERROR_TEMPERATURE,      // Unsafe temperature
  ERROR_ACCURACY,         // Sensor accuracy may be compromised
  ERROR_OVERCURRENT,      // Unsafe current reading
  ERROR_OVERVOLTAGE,      // Unsafe voltage reading
  ERROR_DATA_STORAGE,     // Onboard data storage fault
} ErrorCode;

/* Placeholder Headers */

/* CAN Message Structure */
typedef struct {
  union {
    FDCAN_RxHeaderTypeDef rHeader; // Message header (ID, etc.)
    FDCAN_TxHeaderTypeDef tHeader; // Message header for transmission
  };
  union {
    struct {
      uint8_t command_id;
      union {
        uint8_t short_args[CAN_MAX_DATA_LENGTH - 1];
        struct {
          uint8_t arg_block_id;
          uint8_t long_args[CAN_MAX_DATA_LENGTH - 2];
        };
      };
    };
    struct {
      union {
        uint8_t short_data[CAN_MAX_DATA_LENGTH];
        struct {
          uint8_t data_block_id;
          uint8_t long_data[CAN_MAX_DATA_LENGTH - 1];
        };
      };
    } msg;
    uint8_t raw_data[CAN_MAX_DATA_LENGTH];
  } __attribute__((packed));
} CANMessage;
/* Function Prototypes */

#endif /* PAPYRUS_CAN_H */
