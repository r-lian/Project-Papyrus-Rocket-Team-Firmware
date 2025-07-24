/**
 * @file papyrus_can.c
 * @brief Papyrus CAN Bus Protocol Implementation
 * @author Papyrus Avionics Team
 * @date 2024
 */

#include "papyrus_can.h"
#include <stddef.h>
#include <stdio.h>

/* CRC8 lookup table for fast checksum calculation */
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3};

uint32_t can_build_id(can_priority_t priority, board_id_t source,
                      msg_type_t msg_type) {
  uint32_t id = 0;

  // Validate inputs
  if (priority > CAN_PRIORITY_DEBUG || source > BOARD_ID_GROUND ||
      msg_type > MSG_TYPE_RESERVED) {
    return 0; // Invalid ID
  }

  // Build CAN ID from components
  id |= ((uint32_t)priority & CAN_ID_PRIORITY_MASK) << CAN_ID_PRIORITY_SHIFT;
  id |= ((uint32_t)source & CAN_ID_SOURCE_MASK) << CAN_ID_SOURCE_SHIFT;
  id |= ((uint32_t)msg_type & CAN_ID_MSGTYPE_MASK) << CAN_ID_MSGTYPE_SHIFT;

  return id;
}

can_priority_t can_get_priority(uint32_t can_id) {
  return (can_priority_t)((can_id >> CAN_ID_PRIORITY_SHIFT) &
                          CAN_ID_PRIORITY_MASK);
}

board_id_t can_get_source(uint32_t can_id) {
  return (board_id_t)((can_id >> CAN_ID_SOURCE_SHIFT) & CAN_ID_SOURCE_MASK);
}

msg_type_t can_get_msg_type(uint32_t can_id) {
  return (msg_type_t)((can_id >> CAN_ID_MSGTYPE_SHIFT) & CAN_ID_MSGTYPE_MASK);
}

bool can_validate_message(const can_message_t *msg) {
  if (msg == NULL) {
    return false;
  }

  // Check data length
  if (msg->length > CAN_MAX_DATA_LENGTH) {
    return false;
  }

  // Check CAN ID validity
  if (msg->id > 0x7FF) { // 11-bit CAN ID limit
    return false;
  }

  // Extract components and validate
  can_priority_t priority = can_get_priority(msg->id);
  board_id_t source = can_get_source(msg->id);
  msg_type_t msg_type = can_get_msg_type(msg->id);

  if (priority > CAN_PRIORITY_DEBUG || source > BOARD_ID_GROUND ||
      msg_type > MSG_TYPE_RESERVED) {
    return false;
  }

  // Additional message-specific validation
  switch (msg_type) {
  case MSG_TYPE_SYSTEM:
    // System messages should have at least 1 byte (command)
    return (msg->length >= 1);

  case MSG_TYPE_HEARTBEAT:
    // Heartbeat messages should be exactly 4 bytes (timestamp)
    return (msg->length == 4);

  case MSG_TYPE_ACK:
  case MSG_TYPE_NACK:
    // Acknowledgment messages should have at least 2 bytes
    return (msg->length >= 2);

  default:
    // Other message types are valid if basic checks pass
    return true;
  }
}

uint8_t can_calculate_crc8(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0;

  if (data == NULL || length == 0) {
    return 0;
  }

  for (uint8_t i = 0; i < length; i++) {
    crc = crc8_table[crc ^ data[i]];
  }

  return crc;
}

/**
 * @brief Convert CAN message to human-readable string for debugging
 * @param msg Pointer to CAN message
 * @param buffer Output buffer
 * @param buffer_size Size of output buffer
 * @return Length of formatted string
 */
int can_message_to_string(const can_message_t *msg, char *buffer,
                          int buffer_size) {
  if (msg == NULL || buffer == NULL || buffer_size < 64) {
    return 0;
  }

  can_priority_t priority = can_get_priority(msg->id);
  board_id_t source = can_get_source(msg->id);
  msg_type_t msg_type = can_get_msg_type(msg->id);

  const char *priority_str[] = {"EMRG", "CTRL", "DATA", "DBUG"};
  const char *msg_type_str[] = {"SYS", "CMD", "DAT", "STA", "ERR", "CFG",
                                "SYN", "DBG", "HB",  "ACK", "NAK"};

  int len =
      snprintf(buffer, buffer_size,
               "ID:0x%03X [%s|%02d|%s] Len:%d Data:", (unsigned int)msg->id,
               (priority < 4) ? priority_str[priority] : "???", source,
               (msg_type < 11) ? msg_type_str[msg_type] : "???", msg->length);

  // Add hex data bytes
  for (int i = 0; i < msg->length && len < buffer_size - 4; i++) {
    len += snprintf(buffer + len, buffer_size - len, " %02X", msg->data[i]);
  }

  return len;
}

/**
 * @brief Check if board ID is a controller type
 * @param board_id Board identifier
 * @return true if controller board, false otherwise
 */
bool can_is_controller_board(board_id_t board_id) {
  return (board_id >= BOARD_ID_SERVO_1 && board_id <= BOARD_ID_IO_2);
}

/**
 * @brief Get board type string for debugging
 * @param board_id Board identifier
 * @return String description of board type
 */
const char *can_get_board_name(board_id_t board_id) {
  switch (board_id) {
  case BOARD_ID_MAIN:
    return "Main Board";
  case BOARD_ID_SERVO_1:
    return "Servo Ctrl #1";
  case BOARD_ID_SERVO_2:
    return "Servo Ctrl #2";
  case BOARD_ID_SERVO_3:
    return "Servo Ctrl #3";
  case BOARD_ID_TC_1:
    return "TC Ctrl #1";
  case BOARD_ID_TC_2:
    return "TC Ctrl #2";
  case BOARD_ID_TC_3:
    return "TC Ctrl #3";
  case BOARD_ID_IO_1:
    return "I/O Ctrl #1";
  case BOARD_ID_IO_2:
    return "I/O Ctrl #2";
  case BOARD_ID_POWER:
    return "Power Board";
  case BOARD_ID_RADIO:
    return "Radio Board";
  case BOARD_ID_DEBUGGER:
    return "Bus Debugger";
  case BOARD_ID_GROUND:
    return "Ground Station";
  default:
    return "Unknown";
  }
}
