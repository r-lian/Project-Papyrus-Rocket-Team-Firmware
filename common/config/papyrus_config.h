/**
 * @file papyrus_config.h
 * @brief Papyrus System Configuration
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef PAPYRUS_CONFIG_H
#define PAPYRUS_CONFIG_H

#include <stdint.h>

/* System Version Information */
#define PAPYRUS_VERSION_MAJOR       1
#define PAPYRUS_VERSION_MINOR       0
#define PAPYRUS_VERSION_PATCH       0
#define PAPYRUS_VERSION_BUILD       1

/* Timing Configuration */
#define SYSTEM_TICK_FREQUENCY_HZ    1000    // 1ms system tick
#define EMERGENCY_RESPONSE_TIME_MS  1       // Emergency stop response time
#define SERVO_UPDATE_RATE_HZ        50      // Servo control loop rate
#define SENSOR_UPDATE_RATE_HZ       100     // Sensor data collection rate
#define STATUS_REPORT_RATE_HZ       10      // Status reporting rate
#define HEARTBEAT_RATE_HZ           1       // Heartbeat transmission rate

/* Communication Configuration */
#define CAN_BUS_BAUDRATE           500000   // 500 kbps
#define RADIO_FREQUENCY            433000000 // 433 MHz
#define RADIO_BAUDRATE             9600     // Radio serial baud rate
#define UART_DEBUG_BAUDRATE        115200   // Debug UART baud rate

/* Hardware Configuration */
#define SYSTEM_VOLTAGE_3V3         3300     // 3.3V rail (mV)
#define SYSTEM_VOLTAGE_5V0         5000     // 5.0V rail (mV)
#define SYSTEM_VOLTAGE_12V         12000    // 12V rail (mV)

/* Temperature Limits */
#define TEMP_CRITICAL_HIGH_C       85       // Critical high temperature (°C)
#define TEMP_WARNING_HIGH_C        75       // Warning high temperature (°C)
#define TEMP_WARNING_LOW_C         -10      // Warning low temperature (°C)
#define TEMP_CRITICAL_LOW_C        -20      // Critical low temperature (°C)

/* Servo Configuration */
#define SERVO_PWM_FREQUENCY_HZ     50       // Standard servo PWM frequency
#define SERVO_PULSE_MIN_US         1000     // Minimum servo pulse width (μs)
#define SERVO_PULSE_MAX_US         2000     // Maximum servo pulse width (μs)
#define SERVO_POSITION_MIN         0        // Minimum servo position
#define SERVO_POSITION_MAX         4095     // Maximum servo position (12-bit)
#define SERVO_CURRENT_LIMIT_MA     2000     // Servo current limit (mA)

/* Thermocouple Configuration */
#define TC_MAX_CHANNELS            8        // Maximum TCs per controller
#define TC_SAMPLE_RATE_HZ          10       // TC sampling rate
#define TC_TEMP_RESOLUTION         0.1      // Temperature resolution (°C)
#define TC_MAX_TEMP_C              1200     // Maximum measurable temperature
#define TC_MIN_TEMP_C              -200     // Minimum measurable temperature

/* Memory Configuration */
#define FLASH_CONFIG_SIZE          4096     // Configuration storage size (bytes)
#define FLASH_LOG_SIZE             65536    // Log storage size (bytes)
#define RAM_BUFFER_SIZE            1024     // General purpose buffer size
#define CAN_RX_BUFFER_SIZE         32       // CAN receive buffer depth
#define CAN_TX_BUFFER_SIZE         32       // CAN transmit buffer depth

/* Safety Configuration */
#define WATCHDOG_TIMEOUT_MS        5000     // Hardware watchdog timeout
#define CAN_TIMEOUT_MS             1000     // CAN communication timeout
#define EMERGENCY_STOP_HOLD_MS     10000    // Emergency stop hold time
#define SAFE_MODE_TIMEOUT_MS       30000    // Auto-exit safe mode timeout

/* Board-Specific Configurations */

/* Main Board Configuration */
#define MAIN_BOARD_CPU_FREQ_MHZ    168      // CPU frequency
#define MAIN_BOARD_CAN_INTERFACES  2        // Number of CAN interfaces
#define MAIN_BOARD_UART_INTERFACES 3        // Number of UART interfaces
#define MAIN_BOARD_SPI_INTERFACES  2        // Number of SPI interfaces
#define MAIN_BOARD_I2C_INTERFACES  2        // Number of I2C interfaces

/* Controller Board Configuration */
#define CONTROLLER_CPU_FREQ_MHZ    48       // CPU frequency for controllers
#define CONTROLLER_CAN_INTERFACES  1        // Single CAN interface
#define CONTROLLER_PWM_CHANNELS    8        // Maximum PWM channels
#define CONTROLLER_ADC_CHANNELS    12       // Maximum ADC channels
#define CONTROLLER_GPIO_PINS       16       // Available GPIO pins

/* Ground Station Configuration */
#define GS_RADIO_TIMEOUT_MS        5000     // Radio communication timeout
#define GS_WEB_PORT               8080      // Web interface port
#define GS_WEBSOCKET_PORT         8081      // WebSocket port
#define GS_MAX_CLIENTS            5         // Maximum concurrent web clients

/* Error Handling Configuration */
#define MAX_ERROR_LOG_ENTRIES      100      // Maximum error log entries
#define ERROR_RETRY_COUNT          3        // Maximum error retry attempts
#define ERROR_ESCALATION_TIME_MS   1000     // Time before error escalation

/* Debug Configuration */
#ifdef DEBUG
    #define DEBUG_ENABLED          1
    #define DEBUG_CAN_VERBOSE      1
    #define DEBUG_TASK_MONITORING  1
    #define DEBUG_MEMORY_TRACKING  1
#else
    #define DEBUG_ENABLED          0
    #define DEBUG_CAN_VERBOSE      0
    #define DEBUG_TASK_MONITORING  0
    #define DEBUG_MEMORY_TRACKING  0
#endif

/* FreeRTOS Configuration */
#define RTOS_TICK_RATE_HZ          1000     // OS tick rate
#define RTOS_MIN_STACK_SIZE        256      // Minimum task stack size (words)
#define RTOS_HEAP_SIZE             32768    // FreeRTOS heap size (bytes)

/* Task Priorities (0 = lowest, 7 = highest) */
#define TASK_PRIORITY_EMERGENCY    7        // Emergency handler
#define TASK_PRIORITY_CAN_RX       6        // CAN receive
#define TASK_PRIORITY_CONTROL      5        // Control loops
#define TASK_PRIORITY_CAN_TX       4        // CAN transmit
#define TASK_PRIORITY_SENSOR       3        // Sensor reading
#define TASK_PRIORITY_STATUS       2        // Status reporting
#define TASK_PRIORITY_DEBUG        1        // Debug tasks
#define TASK_PRIORITY_IDLE         0        // Background tasks

/* Task Stack Sizes (in words) */
#define STACK_SIZE_EMERGENCY       512
#define STACK_SIZE_CAN_HANDLER     1024
#define STACK_SIZE_CONTROL         1024
#define STACK_SIZE_SENSOR          512
#define STACK_SIZE_STATUS          256
#define STACK_SIZE_DEBUG           512

/* Pin Definitions - These would be board-specific */
/* Note: Actual pin definitions should be in board-specific config files */

/* Common LED Indicators */
#define LED_POWER_PIN              0        // Power LED
#define LED_STATUS_PIN             1        // Status LED
#define LED_ERROR_PIN              2        // Error LED
#define LED_CAN_RX_PIN             3        // CAN RX activity LED
#define LED_CAN_TX_PIN             4        // CAN TX activity LED

/* Common Button Inputs */
#define BUTTON_RESET_PIN           0        // Reset button
#define BUTTON_SAFE_MODE_PIN       1        // Safe mode button

/* Validation Macros */
#define IS_VALID_BOARD_ID(id)      ((id) >= BOARD_ID_MAIN && (id) <= BOARD_ID_GROUND)
#define IS_VALID_TEMP(temp)        ((temp) >= TC_MIN_TEMP_C && (temp) <= TC_MAX_TEMP_C)
#define IS_VALID_SERVO_POS(pos)    ((pos) >= SERVO_POSITION_MIN && (pos) <= SERVO_POSITION_MAX)
#define IS_VALID_VOLTAGE_3V3(mv)   ((mv) >= 3000 && (mv) <= 3600)
#define IS_VALID_VOLTAGE_5V0(mv)   ((mv) >= 4500 && (mv) <= 5500)
#define IS_VALID_VOLTAGE_12V(mv)   ((mv) >= 10800 && (mv) <= 13200)

/* Utility Macros */
#define MS_TO_TICKS(ms)            ((ms) * RTOS_TICK_RATE_HZ / 1000)
#define TICKS_TO_MS(ticks)         ((ticks) * 1000 / RTOS_TICK_RATE_HZ)
#define CELSIUS_TO_KELVIN(c)       ((c) + 273.15f)
#define KELVIN_TO_CELSIUS(k)       ((k) - 273.15f)
#define SERVO_US_TO_POSITION(us)   (((us) - SERVO_PULSE_MIN_US) * SERVO_POSITION_MAX / (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US))
#define SERVO_POSITION_TO_US(pos)  (SERVO_PULSE_MIN_US + ((pos) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / SERVO_POSITION_MAX))

/* Compile-time Configuration Validation */
#if CAN_BUS_BAUDRATE < 125000 || CAN_BUS_BAUDRATE > 1000000
    #error "CAN bus baudrate must be between 125k and 1M bps"
#endif

#if SERVO_UPDATE_RATE_HZ > 100
    #error "Servo update rate should not exceed 100 Hz"
#endif

#if SENSOR_UPDATE_RATE_HZ > 1000
    #error "Sensor update rate should not exceed 1000 Hz"
#endif

#if WATCHDOG_TIMEOUT_MS < 1000
    #error "Watchdog timeout should be at least 1 second"
#endif

#endif /* PAPYRUS_CONFIG_H */ 