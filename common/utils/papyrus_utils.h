/**
 * @file papyrus_utils.h
 * @brief Papyrus Common Utility Functions
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef PAPYRUS_UTILS_H
#define PAPYRUS_UTILS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Return Status Codes */
typedef enum {
  PAPYRUS_OK = 0,              // No error occurred
  PAPYRUS_ERROR = -1,          // Generic error code
  PAPYRUS_ERROR_PARAM = -2,    // Parameter out of range/not valid
  PAPYRUS_ERROR_TIMEOUT = -3,  // Operation timed out
  PAPYRUS_ERROR_MEMORY = -4,   // Not enough/corrupt memory
  PAPYRUS_ERROR_BUSY = -5,     // Hardware resource busy
  PAPYRUS_ERROR_HARDWARE = -6, // Unable to initialize or configure hardware
  PAPYRUS_ERROR_SAFETY = -7,   // Operation prohibited due to unsafe conditions
} PapyrusStatus;

/* System Statistics */
typedef struct {
  uint32_t can_messages_sent;
  uint32_t can_messages_received;
  uint32_t can_errors;
  uint32_t board_errors;
  uint32_t timestamp_start;
  uint32_t notifications_sent;
} SystemStats;

/* Utility Macros */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) < 0 ? -(x) : (x))
#define CLAMP(x, min, max) (PAPYRUS_MIN(PAPYRUS_MAX((x), (min)), (max)))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define GPIO(p) p.grp, p.pin
#define FORWARD_ERR(stmt)                                                      \
  if ((err = stmt) != PAPYRUS_OK)                                              \
    return err;

#endif /* PAPYRUS_UTILS_H */
