/**
 * @file bus_debugger.h
 * @brief Papyrus CAN Bus Debugger Firmware
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef BUS_DEBUGGER_H
#define BUS_DEBUGGER_H

#include "papyrus_can.h"
#include "papyrus_config.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal_dma.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_i2c.h"
#include "stm32c0xx_hal_spi.h"
#include "stm32c0xx_hal_tim.h"
#include <stdbool.h>
#include <stdint.h>

#include "stm32c0xx_hal_adc.h"
/* Message Log Entry */
typedef struct {
  uint32_t timestamp;
  CANMessage message;
  bool received;
} MsgLogEntry;

#define BUTTON_UP 1
#define BUTTON_DOWN 2
#define BUTTON_LEFT 4
#define BUTTON_RIGHT 8
#define BUTTON_YES 16
#define BUTTON_NO 32
#define BUTTON_MENU 64

/* Predefined UI constructs:
 * Menu: a set of MenuEntry that can be navigated with the up and down buttons
 * MenuEntry has many different types:
 *  Label: a simple predefined text entry
 *  LongLabel: a text entry that can be navigated with left and right
 *  Pressable: an entry that can be selected and activated with YES
 *      when selected, it does things, including entering a new menu
 *  Choice: an entry that can be tweaked using left and right to select between
 * a set of options each option is another MenuEntry!
 *  Variable: an entry that
 * allows modification of some kind of value, like an integer, enum, etc.
 *  A single Menu is always active, and a Menu stack of previous Menus is
 * maintained By default: pressing NO will pop the Menu stack, and pressing MENU
 * will push a new copy of the root menu to the stack If more than two copies of
 * the root Menu are on the stack, all but the last two are removed
 * there is also a single MenuEntry called "sticky", which occupies the space
 * that the menu item selection isn't in no matter the scroll option
 * used for errors, notifications, headers, etc.
 */

typedef enum {
  MENT_LABEL,
  MENT_PRESSABLE,
  MENT_CHOICE,
  MENT_VARIABLE
} MEntType;

typedef enum {
  MVAR_INT,
  MVAR_FLOAT,
  MVAR_BOOL,
  MVAR_ENUM,
} MVarType;

typedef struct Menu Menu;
typedef struct MenuEntry MenuEntry;

struct MenuEntry {
  MEntType type;
  union {
    struct {
      char *text;
      bool owned_text;
      bool is_scrollable;
      uint16_t scroll_pos;
      void (*update_text)(char **);
    } label;
    struct {
      char *text;
      void (*on_yes)(Menu *);
      void (*on_no)(Menu *);
      void (*on_left)(Menu *);
      void (*on_right)(Menu *);
      void (*on_menu)(Menu *);
    } pressable;
    struct {
      MenuEntry *choices;
      bool owned_choices;
      uint8_t cur_choice;
      uint8_t num_choices;
      uint8_t default_choice;
      bool do_wrap;
    } choice;
    struct {
      char *format;
      MVarType kind;
      void (*commit)(void *);
      bool do_confirm;
      union {
        struct {
          int32_t int_val;
          int32_t int_min;
          int32_t int_max;
          int32_t int_step;
          int32_t default_int;
        };
        struct {
          float float_val;
          float float_min;
          float float_max;
          float float_step;
          float default_float;
          bool yes_enabled;
          bool no_enabled;
        };
        struct {
          bool bool_val;
          char *true_str;
          bool default_bool;
          char *false_str;
        };
        struct {
          char *enum_options;
          uint32_t enum_val;
          uint32_t default_enum;
        };
      };
    } variable;
  };
  bool is_selectable;
};

struct Menu {
  MenuEntry *entries;
  uint16_t num_entries;
  uint16_t cur_entry;
  uint16_t scroll_pos;
  MenuEntry sticky;
  bool sticky_enabled;
  Menu *prev_menu;
};

extern Menu *main_menu;
extern Menu *cur_menu;
/* Bus Debugger Class */
typedef struct {
  /* Configuration and Status */

  /* Hardware Interfaces */
  PapyrusCAN can;
  PapyrusI2C display;
  PapyrusADC batread;
  PapyrusPWM piezo;

  PapyrusGPIO status_led;
  PapyrusGPIO fault_led;
  PapyrusGPIO backlight;
  PapyrusGPIO lcd_reset;

  PapyrusGPIO button_pins[7];

  uint8_t buttons_pressed;
  uint8_t newly_pressed;
  uint32_t keyrep_timer;
  uint8_t keyrep_mask;

  uint32_t backlight_sleep;

  /* Message Logging */
  MsgLogEntry *message_log;
  uint32_t msg_log_len;
  uint32_t msg_log_cap;

} BusDebugger;

#endif /* BUS_DEBUGGER_H */

extern bool next_tick;
extern bool tick_too_slow;

PapyrusStatus bus_debugger_init(BusDebugger *bus_dbg);
PapyrusStatus bd_hardware_init(BusDebugger *bus_dbg);
void Error_Handler();
