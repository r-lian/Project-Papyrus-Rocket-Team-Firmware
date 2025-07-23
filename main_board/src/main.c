/**
 * @file main.c
 * @brief Papyrus Main Board - Central Coordinator
 * @author Papyrus Avionics Team
 * @date 2024
 */

#include <stdio.h>
#include <string.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* STM32 HAL includes */
#include "stm32h7xx_hal.h"

/* Papyrus common includes */
#include "papyrus_config.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"

/* Main board specific includes */
#include "main_board.h"
#include "can_manager.h"
#include "system_controller.h"
#include "data_logger.h"
#include "radio_handler.h"
#include "power_monitor.h"
#include "safety_monitor.h"

/* Global Variables */
static main_board_config_t g_main_config;
static main_board_status_t g_main_status;

/* Task Handles */
static TaskHandle_t h_emergency_task = NULL;
static TaskHandle_t h_can_manager_task = NULL;
static TaskHandle_t h_system_controller_task = NULL;
static TaskHandle_t h_data_logger_task = NULL;
static TaskHandle_t h_radio_handler_task = NULL;
static TaskHandle_t h_power_monitor_task = NULL;
static TaskHandle_t h_status_reporter_task = NULL;

/* Queue Handles */
static QueueHandle_t q_can_rx = NULL;
static QueueHandle_t q_can_tx = NULL;
static QueueHandle_t q_system_events = NULL;
static QueueHandle_t q_data_log = NULL;

/* Semaphore Handles */
static SemaphoreHandle_t sem_system_state = NULL;
static SemaphoreHandle_t sem_config_access = NULL;

/* Timer Handles */
static TimerHandle_t timer_heartbeat = NULL;
static TimerHandle_t timer_watchdog_feed = NULL;

/* Function Prototypes */
static void system_clock_config(void);
static void gpio_init(void);
static void hardware_init(void);
static papyrus_status_t create_rtos_objects(void);
static papyrus_status_t create_tasks(void);
static void start_timers(void);

/* Task Function Prototypes */
static void emergency_task(void *pvParameters);
static void can_manager_task(void *pvParameters);
static void system_controller_task(void *pvParameters);
static void data_logger_task(void *pvParameters);
static void radio_handler_task(void *pvParameters);
static void power_monitor_task(void *pvParameters);
static void status_reporter_task(void *pvParameters);

/* Timer Callback Prototypes */
static void heartbeat_timer_callback(TimerHandle_t xTimer);
static void watchdog_feed_timer_callback(TimerHandle_t xTimer);

/**
 * @brief Main application entry point
 */
int main(void)
{
    papyrus_status_t status;
    
    /* Initialize HAL */
    HAL_Init();
    
    /* Configure system clock */
    system_clock_config();
    
    /* Initialize GPIO */
    gpio_init();
    
    /* Initialize hardware peripherals */
    hardware_init();
    
    /* Initialize Papyrus common systems */
    papyrus_utils_init();
    papyrus_error_init();
    papyrus_safety_init();
    papyrus_health_init();
    papyrus_memory_init();
    
    /* Initialize watchdog */
    papyrus_watchdog_init(WATCHDOG_TIMEOUT_MS);
    
    /* Load configuration */
    status = main_board_load_config(&g_main_config);
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_WARN("Failed to load config, using defaults");
        main_board_default_config(&g_main_config);
    }
    
    /* Initialize main board status */
    memset(&g_main_status, 0, sizeof(g_main_status));
    g_main_status.system_state = SYSTEM_STATE_INIT;
    g_main_status.startup_time = papyrus_get_timestamp_ms();
    
    /* Create FreeRTOS objects */
    status = create_rtos_objects();
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_ERROR("Failed to create RTOS objects");
        papyrus_emergency_stop(ERROR_MEMORY);
        while (1) { /* System halt */ }
    }
    
    /* Create application tasks */
    status = create_tasks();
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_ERROR("Failed to create tasks");
        papyrus_emergency_stop(ERROR_MEMORY);
        while (1) { /* System halt */ }
    }
    
    /* Start timers */
    start_timers();
    
    /* Enable watchdog */
    papyrus_watchdog_enable();
    
    PAPYRUS_LOG_INFO("Papyrus Main Board starting...");
    PAPYRUS_LOG_INFO("Version: %d.%d.%d Build: %d", 
                    PAPYRUS_VERSION_MAJOR, PAPYRUS_VERSION_MINOR,
                    PAPYRUS_VERSION_PATCH, PAPYRUS_VERSION_BUILD);
    
    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();
    
    /* Should never reach here */
    PAPYRUS_LOG_ERROR("Scheduler failed to start!");
    while (1) {
        /* System halt */
    }
}

/**
 * @brief Create FreeRTOS objects (queues, semaphores, etc.)
 */
static papyrus_status_t create_rtos_objects(void)
{
    /* Create message queues */
    q_can_rx = xQueueCreate(CAN_RX_BUFFER_SIZE, sizeof(can_message_t));
    q_can_tx = xQueueCreate(CAN_TX_BUFFER_SIZE, sizeof(can_message_t));
    q_system_events = xQueueCreate(16, sizeof(system_event_t));
    q_data_log = xQueueCreate(32, sizeof(data_log_entry_t));
    
    if (!q_can_rx || !q_can_tx || !q_system_events || !q_data_log) {
        return PAPYRUS_ERROR_NO_MEMORY;
    }
    
    /* Create semaphores */
    sem_system_state = xSemaphoreCreateMutex();
    sem_config_access = xSemaphoreCreateMutex();
    
    if (!sem_system_state || !sem_config_access) {
        return PAPYRUS_ERROR_NO_MEMORY;
    }
    
    /* Create timers */
    timer_heartbeat = xTimerCreate("Heartbeat", 
                                   MS_TO_TICKS(1000 / HEARTBEAT_RATE_HZ),
                                   pdTRUE, NULL, heartbeat_timer_callback);
    
    timer_watchdog_feed = xTimerCreate("WatchdogFeed",
                                       MS_TO_TICKS(WATCHDOG_TIMEOUT_MS / 4),
                                       pdTRUE, NULL, watchdog_feed_timer_callback);
    
    if (!timer_heartbeat || !timer_watchdog_feed) {
        return PAPYRUS_ERROR_NO_MEMORY;
    }
    
    return PAPYRUS_OK;
}

/**
 * @brief Create application tasks
 */
static papyrus_status_t create_tasks(void)
{
    BaseType_t result;
    
    /* Emergency Handler Task - Highest Priority */
    result = xTaskCreate(emergency_task, "Emergency", 
                        STACK_SIZE_EMERGENCY, NULL,
                        TASK_PRIORITY_EMERGENCY, &h_emergency_task);
    if (result != pdPASS) return PAPYRUS_ERROR_NO_MEMORY;
    
    /* CAN Manager Task */
    result = xTaskCreate(can_manager_task, "CANManager",
                        STACK_SIZE_CAN_HANDLER, NULL,
                        TASK_PRIORITY_CAN_RX, &h_can_manager_task);
    if (result != pdPASS) return PAPYRUS_ERROR_NO_MEMORY;
    
    /* System Controller Task */
    result = xTaskCreate(system_controller_task, "SysCtrl",
                        STACK_SIZE_CONTROL, NULL,
                        TASK_PRIORITY_CONTROL, &h_system_controller_task);
    if (result != pdPASS) return PAPYRUS_ERROR_NO_MEMORY;
    
    /* Data Logger Task */
    result = xTaskCreate(data_logger_task, "DataLogger",
                        STACK_SIZE_SENSOR, NULL,
                        TASK_PRIORITY_SENSOR, &h_data_logger_task);
    if (result != pdPASS) return PAPYRUS_ERROR_NO_MEMORY;
    
    /* Radio Handler Task */
    result = xTaskCreate(radio_handler_task, "RadioHandler",
                        STACK_SIZE_CAN_HANDLER, NULL,
                        TASK_PRIORITY_CAN_TX, &h_radio_handler_task);
    if (result != pdPASS) return PAPYRUS_ERROR_NO_MEMORY;
    
    /* Power Monitor Task */
    result = xTaskCreate(power_monitor_task, "PowerMon",
                        STACK_SIZE_SENSOR, NULL,
                        TASK_PRIORITY_SENSOR, &h_power_monitor_task);
    if (result != pdPASS) return PAPYRUS_ERROR_NO_MEMORY;
    
    /* Status Reporter Task */
    result = xTaskCreate(status_reporter_task, "StatusRpt",
                        STACK_SIZE_STATUS, NULL,
                        TASK_PRIORITY_STATUS, &h_status_reporter_task);
    if (result != pdPASS) return PAPYRUS_ERROR_NO_MEMORY;
    
    return PAPYRUS_OK;
}

/**
 * @brief Start periodic timers
 */
static void start_timers(void)
{
    xTimerStart(timer_heartbeat, 0);
    xTimerStart(timer_watchdog_feed, 0);
}

/**
 * @brief Emergency Handler Task - Highest priority for safety-critical operations
 */
static void emergency_task(void *pvParameters)
{
    PAPYRUS_UNUSED(pvParameters);
    
    PAPYRUS_LOG_INFO("Emergency task started");
    
    while (1) {
        /* Wait for emergency events or timeout */
        uint32_t notification_value;
        BaseType_t result = xTaskNotifyWait(0, UINT32_MAX, &notification_value, 
                                           MS_TO_TICKS(100));
        
        if (result == pdTRUE) {
            /* Emergency event received */
            error_code_t error_code = (error_code_t)(notification_value & 0xFF);
            
            PAPYRUS_LOG_ERROR("Emergency event: 0x%02X", error_code);
            
            /* Execute emergency stop procedure */
            emergency_stop_procedure(error_code);
        }
        
        /* Check for critical system conditions */
        check_critical_conditions();
        
        /* Monitor task health */
        monitor_task_health();
    }
}

/**
 * @brief CAN Manager Task - Handles CAN communication
 */
static void can_manager_task(void *pvParameters)
{
    PAPYRUS_UNUSED(pvParameters);
    
    papyrus_status_t status = can_manager_init(q_can_rx, q_can_tx);
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_ERROR("CAN Manager init failed");
        papyrus_emergency_stop(ERROR_COMMUNICATION);
        vTaskDelete(NULL);
    }
    
    PAPYRUS_LOG_INFO("CAN Manager task started");
    
    while (1) {
        can_manager_process();
        vTaskDelay(MS_TO_TICKS(5)); /* 200 Hz processing rate */
    }
}

/**
 * @brief System Controller Task - Main system logic and coordination
 */
static void system_controller_task(void *pvParameters)
{
    PAPYRUS_UNUSED(pvParameters);
    
    papyrus_status_t status = system_controller_init(&g_main_config, &g_main_status);
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_ERROR("System Controller init failed");
        papyrus_emergency_stop(ERROR_HARDWARE);
        vTaskDelete(NULL);
    }
    
    PAPYRUS_LOG_INFO("System Controller task started");
    
    while (1) {
        system_controller_process();
        vTaskDelay(MS_TO_TICKS(20)); /* 50 Hz processing rate */
    }
}

/**
 * @brief Data Logger Task - Handles data logging and storage
 */
static void data_logger_task(void *pvParameters)
{
    PAPYRUS_UNUSED(pvParameters);
    
    papyrus_status_t status = data_logger_init(q_data_log);
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_ERROR("Data Logger init failed");
        /* Non-critical failure, continue without logging */
    } else {
        PAPYRUS_LOG_INFO("Data Logger task started");
    }
    
    while (1) {
        data_logger_process();
        vTaskDelay(MS_TO_TICKS(10)); /* 100 Hz processing rate */
    }
}

/**
 * @brief Radio Handler Task - Manages radio communication with ground station
 */
static void radio_handler_task(void *pvParameters)
{
    PAPYRUS_UNUSED(pvParameters);
    
    papyrus_status_t status = radio_handler_init();
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_ERROR("Radio Handler init failed");
        /* Non-critical failure, continue without radio */
    } else {
        PAPYRUS_LOG_INFO("Radio Handler task started");
    }
    
    while (1) {
        radio_handler_process();
        vTaskDelay(MS_TO_TICKS(50)); /* 20 Hz processing rate */
    }
}

/**
 * @brief Power Monitor Task - Monitors system power and voltages
 */
static void power_monitor_task(void *pvParameters)
{
    PAPYRUS_UNUSED(pvParameters);
    
    papyrus_status_t status = power_monitor_init();
    if (status != PAPYRUS_OK) {
        PAPYRUS_LOG_ERROR("Power Monitor init failed");
        papyrus_emergency_stop(ERROR_POWER);
        vTaskDelete(NULL);
    }
    
    PAPYRUS_LOG_INFO("Power Monitor task started");
    
    while (1) {
        power_monitor_process();
        vTaskDelay(MS_TO_TICKS(100)); /* 10 Hz processing rate */
    }
}

/**
 * @brief Status Reporter Task - Sends periodic status updates
 */
static void status_reporter_task(void *pvParameters)
{
    PAPYRUS_UNUSED(pvParameters);
    
    PAPYRUS_LOG_INFO("Status Reporter task started");
    
    while (1) {
        status_reporter_process();
        vTaskDelay(MS_TO_TICKS(100)); /* 10 Hz processing rate */
    }
}

/**
 * @brief Heartbeat timer callback
 */
static void heartbeat_timer_callback(TimerHandle_t xTimer)
{
    PAPYRUS_UNUSED(xTimer);
    
    /* Send heartbeat message on CAN bus */
    can_message_t heartbeat_msg;
    heartbeat_msg.id = can_build_id(CAN_PRIORITY_DATA, BOARD_ID_MAIN, MSG_TYPE_HEARTBEAT);
    heartbeat_msg.length = 4;
    uint32_t timestamp = papyrus_get_timestamp_ms();
    memcpy(heartbeat_msg.data, &timestamp, 4);
    heartbeat_msg.timestamp = timestamp;
    
    xQueueSend(q_can_tx, &heartbeat_msg, 0);
    
    /* Toggle status LED */
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
}

/**
 * @brief Watchdog feed timer callback
 */
static void watchdog_feed_timer_callback(TimerHandle_t xTimer)
{
    PAPYRUS_UNUSED(xTimer);
    
    /* Feed the watchdog timer */
    papyrus_watchdog_feed();
}

/**
 * @brief System clock configuration
 */
static void system_clock_config(void)
{
    /* Configure system clock to 480 MHz */
    /* Implementation depends on specific STM32H7 variant */
    /* This is a placeholder - actual implementation needed */
}

/**
 * @brief GPIO initialization
 */
static void gpio_init(void)
{
    /* Initialize GPIO pins for LEDs, buttons, etc. */
    /* Implementation depends on specific board design */
    /* This is a placeholder - actual implementation needed */
}

/**
 * @brief Hardware initialization
 */
static void hardware_init(void)
{
    /* Initialize CAN, UART, SPI, I2C, ADC, etc. */
    /* Implementation depends on specific board design */
    /* This is a placeholder - actual implementation needed */
}

/* Error handlers and callbacks */
void Error_Handler(void)
{
    papyrus_log_error(ERROR_HARDWARE, BOARD_ID_MAIN, 0, "HAL Error");
    papyrus_emergency_stop(ERROR_HARDWARE);
    while (1) {
        /* System halt */
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    PAPYRUS_UNUSED(xTask);
    PAPYRUS_LOG_ERROR("Stack overflow in task: %s", pcTaskName);
    papyrus_emergency_stop(ERROR_MEMORY);
}

void vApplicationMallocFailedHook(void)
{
    PAPYRUS_LOG_ERROR("Memory allocation failed");
    papyrus_emergency_stop(ERROR_MEMORY);
}

/* Public API Functions */
main_board_status_t* main_board_get_status(void)
{
    return &g_main_status;
}

main_board_config_t* main_board_get_config(void)
{
    return &g_main_config;
}

QueueHandle_t main_board_get_can_tx_queue(void)
{
    return q_can_tx;
}

QueueHandle_t main_board_get_system_event_queue(void)
{
    return q_system_events;
}

QueueHandle_t main_board_get_data_log_queue(void)
{
    return q_data_log;
} 