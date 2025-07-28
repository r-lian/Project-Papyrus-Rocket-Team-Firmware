/**
 * @file controller_base.h
 * @brief Generic Controller Board Framework
 * @author Papyrus Avionics Team
 * @date 2024
 */
#pragma once

#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include <stdbool.h>
#include <stdint.h>

/* Controller Board Types */
typedef enum {
  CONTROLLER_TYPE_SERVO = 0,
  CONTROLLER_TYPE_THERMOCOUPLE,
  CONTROLLER_TYPE_UNKNOWN
} ControllerType;

/* Controller State Definitions */
typedef enum {
  CONTROLLER_STATE_INIT,       // Still initializing
  CONTROLLER_STATE_NOT_CONFIG, // Not yet given ID or other config parameters ,
  CONTROLLER_STATE_OKAY,       // Normal operational state
  CONTROLLER_STATE_FATAL,      // Fatal error state prevents further operation
  CONTROLLER_STATE_DISABLED,   // Disabled in software
  CONTROLLER_STATE_DANGER,     // Safety-critical errors have been detected
} ControllerState;

typedef enum {
  SUBDEV_STATE_ABSENT = 0,
  SUBDEV_STATE_OK,
  SUBDEV_STATE_ERROR,
  SUBDEV_STATE_NOT_CONFIG,
  SUBDEV_STATE_DISABLED
} SubdevState;

/* Board Status Structure */
typedef struct {
  ControllerState state;
  SystemStats statistics;
  uint8_t num_subdevices;
  SubdevState subdev_state[16];
  ErrorEntry error_queue[32];
  uint8_t error_queue_len;
  ErrorEntry persistent_errors[32];
  uint8_t persistent_errors_len;
} BoardStatus;

/* Controller Base Class Structure */
typedef struct {
  /* Configuration and Status */
  BoardID board_id;
  ControllerType controller_type;
  uint8_t board_revision;
  uint8_t firmware_revision;
  BoardStatus status;

  /* CAN Communication */
  PapyrusCAN can;
  CANMessage transactions[8];
  uint8_t num_transactions;

  /* UART Debug Communication */
  PapyrusUART uart;

  /* Indicator GPIOs */
  PapyrusGPIO fault_indicator;
  PapyrusGPIO status_indicator;

} ControllerBase;

typedef PapyrusStatus (*CommandRoutine)(CANMessage *, ControllerBase *,
                                        ErrorEntry *);
extern CommandRoutine papyrus_cmdrun_table[256];
extern CommandRoutine papyrus_cmdresp_table[256];
extern CANMsgLen papyrus_cmd_lens[256];

#define PAPYRUS_TODO_CAN_RXFIFO 1
extern uint32_t papyrus_todo_flags;

void papyrus_handle_todos(ControllerBase *base, uint32_t flags,
                          ErrorEntry *err);

/* Function Prototypes - Base Controller Functions */

/**
 * @brief Initialize controller base system
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus controller_base_init(ControllerBase *controller);

/**
 * @brief Initialize controller base hardware
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus controller_hardware_init(ControllerBase *controller);

/**
 * @brief Load controller configuration from flash
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus controller_base_load_config(ControllerBase *controller);

/**
 * @brief Save controller configuration to flash
 * @param controller Pointer to controller instance
 * @return PAPYRUS_OK on success
 */
PapyrusStatus controller_base_save_config(ControllerBase *controller);
