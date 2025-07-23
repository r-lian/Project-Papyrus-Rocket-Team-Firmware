/**
 * @file servo_controller.h
 * @brief Servo Controller Implementation
 * @author Papyrus Avionics Team
 * @date 2024
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "controller_base.h"
#include "papyrus_config.h"
#include "papyrus_can.h"

/* Servo Controller Specific Configuration */
typedef struct {
    /* Servo Hardware Configuration */
    uint8_t servo_count;
    uint16_t pwm_frequency_hz;
    uint16_t position_min[3];
    uint16_t position_max[3];
    uint16_t position_center[3];
    uint16_t pulse_width_min_us[3];
    uint16_t pulse_width_max_us[3];
    
    /* Current Monitoring */
    uint16_t current_limit_ma[3];
    uint16_t current_warning_ma[3];
    bool current_monitoring_enabled[3];
    
    /* Movement Parameters */
    uint16_t max_speed[3];
    uint16_t acceleration[3];
    uint16_t deceleration[3];
    
    /* Safety Parameters */
    uint16_t position_tolerance[3];
    uint16_t stall_timeout_ms[3];
    uint16_t overshoot_limit[3];
    
    /* Calibration Data */
    bool calibrated[3];
    int16_t position_offset[3];
    float position_scale[3];
    
    /* Feedback Configuration */
    bool feedback_enabled[3];
    uint8_t feedback_pin[3];
    uint16_t feedback_deadband[3];
    
} servo_config_t;

/* Servo Controller Status */
typedef struct {
    /* Current Servo Status */
    uint16_t current_position[3];
    uint16_t target_position[3];
    uint16_t actual_pulse_width[3];
    bool servo_enabled[3];
    bool servo_moving[3];
    
    /* Current Monitoring */
    uint16_t current_ma[3];
    bool current_alarm[3];
    uint32_t total_current_ma;
    
    /* Movement Status */
    uint16_t movement_speed[3];
    uint32_t movement_count[3];
    uint32_t last_movement_time[3];
    
    /* Error Status */
    bool position_error[3];
    bool stall_detected[3];
    bool overcurrent[3];
    bool feedback_error[3];
    uint16_t error_count[3];
    
    /* Performance Metrics */
    uint16_t average_response_time_ms[3];
    uint16_t max_response_time_ms[3];
    uint32_t total_operations[3];
    
} servo_status_t;

/* Servo Commands */
typedef enum {
    SERVO_CMD_SET_POSITION = 0x10,
    SERVO_CMD_SET_SPEED = 0x11,
    SERVO_CMD_ENABLE = 0x12,
    SERVO_CMD_DISABLE = 0x13,
    SERVO_CMD_STOP = 0x14,
    SERVO_CMD_HOME = 0x15,
    SERVO_CMD_CALIBRATE = 0x16,
    SERVO_CMD_SET_LIMITS = 0x17,
    SERVO_CMD_GET_POSITION = 0x18,
    SERVO_CMD_GET_STATUS = 0x19
} servo_command_t;

/* Servo Controller Class */
typedef struct {
    /* Base controller */
    controller_base_t base;
    
    /* Servo-specific configuration and status */
    servo_config_t servo_config;
    servo_status_t servo_status;
    
    /* Hardware interfaces */
    TIM_HandleTypeDef* htim_pwm;
    ADC_HandleTypeDef* hadc_current;
    ADC_HandleTypeDef* hadc_feedback;
    
    /* Control variables */
    uint32_t last_update_time;
    uint16_t control_loop_counter;
    bool position_control_active[3];
    
    /* Movement planning */
    uint16_t trajectory_target[3];
    uint16_t trajectory_current[3];
    uint16_t trajectory_velocity[3];
    uint32_t trajectory_start_time[3];
    
} servo_controller_t;

/* Function Prototypes - Servo Controller Main Functions */

/**
 * @brief Initialize servo controller
 * @param servo_ctrl Pointer to servo controller instance
 * @param board_id Board identifier
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_controller_init(servo_controller_t* servo_ctrl, board_id_t board_id);

/**
 * @brief Main servo controller processing loop
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_controller_process(servo_controller_t* servo_ctrl);

/**
 * @brief Deinitialize servo controller
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_controller_deinit(servo_controller_t* servo_ctrl);

/* Function Prototypes - Servo Control Functions */

/**
 * @brief Set servo position
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param position Target position (0-4095)
 * @param speed Movement speed (0-255)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_set_position(servo_controller_t* servo_ctrl, 
                                   uint8_t servo_id, 
                                   uint16_t position, 
                                   uint8_t speed);

/**
 * @brief Get servo position
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param position Pointer to store current position
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_get_position(servo_controller_t* servo_ctrl, 
                                   uint8_t servo_id, 
                                   uint16_t* position);

/**
 * @brief Enable servo
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_enable(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Disable servo
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_disable(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Stop servo movement immediately
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_stop(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Home servo to center position
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_home(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Set servo movement speed
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param speed Movement speed (0-255)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_set_speed(servo_controller_t* servo_ctrl, 
                                uint8_t servo_id, 
                                uint8_t speed);

/**
 * @brief Set servo position limits
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param min_position Minimum allowed position
 * @param max_position Maximum allowed position
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_set_limits(servo_controller_t* servo_ctrl, 
                                 uint8_t servo_id,
                                 uint16_t min_position, 
                                 uint16_t max_position);

/* Function Prototypes - Hardware Interface */

/**
 * @brief Initialize servo hardware (PWM, ADC, etc.)
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_hardware_init(servo_controller_t* servo_ctrl);

/**
 * @brief Update PWM output for servo
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param pulse_width_us Pulse width in microseconds
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_update_pwm(servo_controller_t* servo_ctrl, 
                                 uint8_t servo_id, 
                                 uint16_t pulse_width_us);

/**
 * @brief Read servo current consumption
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param current_ma Pointer to store current in mA
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_read_current(servo_controller_t* servo_ctrl, 
                                   uint8_t servo_id, 
                                   uint16_t* current_ma);

/**
 * @brief Read servo position feedback
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param feedback_position Pointer to store feedback position
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_read_feedback(servo_controller_t* servo_ctrl, 
                                    uint8_t servo_id, 
                                    uint16_t* feedback_position);

/* Function Prototypes - Control Algorithms */

/**
 * @brief Update servo control loop
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_update_control_loop(servo_controller_t* servo_ctrl);

/**
 * @brief Generate trajectory for smooth movement
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param target_position Target position
 * @param max_speed Maximum movement speed
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_plan_trajectory(servo_controller_t* servo_ctrl, 
                                      uint8_t servo_id,
                                      uint16_t target_position, 
                                      uint16_t max_speed);

/**
 * @brief Execute trajectory step
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_execute_trajectory(servo_controller_t* servo_ctrl, 
                                         uint8_t servo_id);

/**
 * @brief Check for servo stall condition
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return true if stall detected
 */
bool servo_check_stall(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Monitor servo current consumption
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_monitor_current(servo_controller_t* servo_ctrl, uint8_t servo_id);

/* Function Prototypes - Calibration */

/**
 * @brief Calibrate servo position
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_calibrate(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Auto-detect servo limits
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_auto_detect_limits(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Set calibration data
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param offset Position offset
 * @param scale Position scale factor
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_set_calibration(servo_controller_t* servo_ctrl, 
                                      uint8_t servo_id,
                                      int16_t offset, 
                                      float scale);

/* Function Prototypes - CAN Communication */

/**
 * @brief Process servo command from CAN
 * @param servo_ctrl Pointer to servo controller instance
 * @param msg Pointer to CAN message
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_process_can_command(servo_controller_t* servo_ctrl, 
                                          const can_message_t* msg);

/**
 * @brief Send servo status via CAN
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_send_status(servo_controller_t* servo_ctrl);

/**
 * @brief Send servo data via CAN
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_send_data(servo_controller_t* servo_ctrl);

/* Function Prototypes - Safety and Error Handling */

/**
 * @brief Enter safe mode (disable all servos)
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_enter_safe_mode(servo_controller_t* servo_ctrl);

/**
 * @brief Exit safe mode
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_exit_safe_mode(servo_controller_t* servo_ctrl);

/**
 * @brief Emergency stop all servos
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK on success
 */
papyrus_status_t servo_emergency_stop(servo_controller_t* servo_ctrl);

/**
 * @brief Check servo health
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @return true if servo is healthy
 */
bool servo_is_healthy(servo_controller_t* servo_ctrl, uint8_t servo_id);

/**
 * @brief Run servo self-test
 * @param servo_ctrl Pointer to servo controller instance
 * @return PAPYRUS_OK if all tests pass
 */
papyrus_status_t servo_self_test(servo_controller_t* servo_ctrl);

/* Utility Functions */

/**
 * @brief Convert position to pulse width
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param position Position value (0-4095)
 * @return Pulse width in microseconds
 */
uint16_t servo_position_to_pulse_width(servo_controller_t* servo_ctrl, 
                                      uint8_t servo_id, 
                                      uint16_t position);

/**
 * @brief Convert pulse width to position
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param pulse_width_us Pulse width in microseconds
 * @return Position value (0-4095)
 */
uint16_t servo_pulse_width_to_position(servo_controller_t* servo_ctrl, 
                                      uint8_t servo_id, 
                                      uint16_t pulse_width_us);

/**
 * @brief Validate servo position
 * @param servo_ctrl Pointer to servo controller instance
 * @param servo_id Servo identifier (0-2)
 * @param position Position to validate
 * @return true if position is valid
 */
bool servo_validate_position(servo_controller_t* servo_ctrl, 
                            uint8_t servo_id, 
                            uint16_t position);

/* Default Configuration Values */
#define SERVO_DEFAULT_PWM_FREQUENCY_HZ     50
#define SERVO_DEFAULT_PULSE_MIN_US         1000
#define SERVO_DEFAULT_PULSE_MAX_US         2000
#define SERVO_DEFAULT_POSITION_MIN         0
#define SERVO_DEFAULT_POSITION_MAX         4095
#define SERVO_DEFAULT_POSITION_CENTER      2048
#define SERVO_DEFAULT_CURRENT_LIMIT_MA     2000
#define SERVO_DEFAULT_CURRENT_WARNING_MA   1500
#define SERVO_DEFAULT_MAX_SPEED            255
#define SERVO_DEFAULT_ACCELERATION         100
#define SERVO_DEFAULT_POSITION_TOLERANCE   10
#define SERVO_DEFAULT_STALL_TIMEOUT_MS     5000
#define SERVO_DEFAULT_OVERSHOOT_LIMIT      50
#define SERVO_DEFAULT_FEEDBACK_DEADBAND    5

#endif /* SERVO_CONTROLLER_H */ 