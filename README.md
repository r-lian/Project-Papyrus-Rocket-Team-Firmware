# Papyrus Rocket Avionics Firmware

Real-time embedded firmware for multi-board rocket avionics system with CAN bus communication, FreeRTOS task management, and modular controller architecture.

## System Architecture

```
┌─────────────────┐    CAN Bus    ┌─────────────────┐
│   Main Board    │◄─────────────►│ Controller      │
│   (STM32H7)     │               │ Boards          │
│                 │               │ (STM32C092)     │
│ • FreeRTOS      │               │                 │
│ • CAN Manager   │               │ • Servo Ctrl    │
│ • Data Logger   │               │ • Thermocouple  │
│ • Radio Handler │               │ • General I/O   │
│ • Power Monitor │               │                 │
└─────────────────┘               └─────────────────┘
         │                                │
         │ Radio                          │
         ▼                                │
┌─────────────────┐               ┌─────────────────┐
│ Ground Station  │               │ Bus Debugger    │
│ • Web Interface │               │ • CAN Monitor   │
│ • Radio Rx      │               │ • LCD Display   │
│ • Data Logging  │               │ • Standalone    │
└─────────────────┘               └─────────────────┘
```

## Project Structure

```
firmware/
├── common/                    # Shared libraries
│   ├── protocols/            # CAN bus protocol
│   │   ├── papyrus_can.h     # CAN ID structure, message types
│   │   └── papyrus_can.c     # CAN message handling
│   ├── config/               # System configuration
│   │   └── papyrus_config.h  # Timing, priorities, limits
│   └── utils/                # Common utilities
│       └── papyrus_utils.h   # Status codes, error handling
├── main_board/               # Main board firmware
│   ├── inc/
│   │   └── main_board.h      # Main board interface
│   └── src/
│       └── main.c            # FreeRTOS task implementation
├── controllers/              # Controller board firmware
│   ├── framework/            # Generic controller framework
│   │   ├── controller_base.h # Base controller structure
│   │   └── controller_base.c # Common controller functions
│   ├── servo_controller/     # Servo control implementation
│   │   └── servo_controller.h # Servo-specific interface
│   └── tc_controller/        # Thermocouple controller
├── ground_station/           # Ground station software
│   └── firmware/
│       └── ground_station.h  # Ground station interface
├── bus_debugger/             # CAN bus debugging tool
│   └── firmware/             # Debugger firmware
├── tools/                    # Development tools
│   └── testing/              # Test frameworks
├── drivers/                  # Hardware drivers
├── linker/                   # Linker scripts
├── docs/                     # Documentation
├── Makefile                  # Build system
└── README.md                 # This file
```

## File Descriptions and Key Functions

### Common Libraries (`common/`)

#### `common/protocols/papyrus_can.h`
**Purpose**: Defines the Papyrus CAN bus communication protocol and message structures.

**Key Functions**:
- **CAN ID Structure**: 11-bit identifier with priority (3 bits), source board ID (4 bits), message type (4 bits)
- **Message Types**: Emergency, Error, Priority, Command, Notification, Response, Stream, Bulk Data
- **Board IDs**: Main Board (0x0), Servo Controllers (0x1-0x3), Thermocouple (0x4), I/O (0x5), Ground Station (0x6), Bus Debugger (0x7)
- **Error Codes**: Comprehensive error enumeration (CAN bus, hardware, memory, power, temperature, accuracy, overcurrent)
- **Message Structures**: Standardized CAN message format with command IDs, arguments, and data blocks
- **Transaction Management**: Multi-message transaction handling for complex operations

**Critical Functions**:
```c
uint32_t can_build_id(can_priority_t priority, board_id_t source, msg_type_t msg_type);
bool can_validate_message(const can_message_t* msg);
uint8_t can_calculate_crc8(const uint8_t* data, uint8_t length);
```

#### `common/protocols/papyrus_can.c`
**Purpose**: Implements CAN bus protocol functions and message handling.

**Key Functions**:
- **Message Construction**: Build CAN IDs from components, validate message integrity
- **CRC Calculation**: 8-bit CRC for message integrity verification
- **Board Identification**: Convert board IDs to names, identify controller types
- **Debug Support**: Convert messages to human-readable strings for debugging
- **Error Handling**: Validate message parameters and detect protocol violations

#### `common/config/papyrus_config.h`
**Purpose**: Centralized system configuration with compile-time validation.

**Key Configurations**:
- **Timing**: System tick (1kHz), servo update (50Hz), sensor rate (100Hz), status rate (10Hz)
- **Communication**: CAN baudrate (500kbps), radio frequency (433MHz), UART baudrates
- **Hardware Limits**: Voltage rails (3.3V, 5V, 12V), temperature limits (-20°C to 85°C)
- **Safety**: Watchdog timeout (5s), emergency stop hold (10s), safe mode timeout (30s)
- **Memory**: Flash config (4KB), log storage (64KB), RAM buffers (1KB), CAN buffers (32 messages)
- **Task Priorities**: Emergency (7), CAN (6), Control (5), Data (4), Status (2)

**Validation Macros**:
```c
#if CAN_BUS_BAUDRATE < 125000 || CAN_BUS_BAUDRATE > 1000000
    #error "CAN bus baudrate must be between 125k and 1M bps"
#endif
```

#### `common/utils/papyrus_utils.h`
**Purpose**: Common utility functions and data structures for system-wide use.

**Key Components**:
- **Status Codes**: Comprehensive error enumeration (OK, TIMEOUT, INVALID_PARAM, HARDWARE, etc.)
- **System States**: INIT, RUNNING, SAFE_MODE, EMERGENCY_STOP, ERROR, SHUTDOWN
- **Board Health**: Health monitoring structures with error tracking
- **Error Logging**: Error entry structures with timestamps and escalation
- **System Statistics**: Performance metrics, memory usage, communication statistics
- **Debug Macros**: Conditional debug logging and assertion macros

**Utility Functions**:
```c
papyrus_status_t papyrus_init(void);
uint32_t papyrus_get_timestamp(void);
void papyrus_delay_ms(uint32_t ms);
papyrus_status_t papyrus_emergency_stop(error_code_t error);
```

### Main Board (`main_board/`)

#### `main_board/inc/main_board.h`
**Purpose**: Main board interface definitions and FreeRTOS task management.

**Key Structures**:
- **Configuration**: System ID, CAN/radio settings, power management, safety parameters, logging config
- **Status**: System state, power status, communication statistics, controller health, error tracking
- **Events**: System event structures for logging and notification
- **Data Logging**: Telemetry and command logging structures

**Core Functions**:
```c
papyrus_status_t main_board_init(void);
papyrus_status_t main_board_send_command(board_id_t target, system_command_t cmd, const uint8_t* data);
papyrus_status_t main_board_request_status(board_id_t target);
papyrus_status_t emergency_stop_procedure(error_code_t error_code);
QueueHandle_t main_board_get_can_tx_queue(void);
```

**FreeRTOS Integration**:
- **Task Queues**: CAN TX, system events, data logging queues
- **Task Management**: Emergency handler, CAN manager, system controller, data logger, radio handler, power monitor, status reporter
- **Priority Management**: Real-time task scheduling with defined priorities and periods

#### `main_board/src/main.c`
**Purpose**: FreeRTOS task implementation and system coordination.

**Key Tasks**:
- **Emergency Handler** (Priority 7, 1ms): Processes emergency stops and critical errors
- **CAN Manager** (Priority 6, 10ms): Handles CAN bus communication and message routing
- **System Controller** (Priority 5, 50ms): Coordinates system operations and controller management
- **Data Logger** (Priority 4, 100ms): Logs telemetry and system events to SD card
- **Radio Handler** (Priority 4, 20ms): Manages radio communication with ground station
- **Power Monitor** (Priority 3, 500ms): Monitors power rails and battery status
- **Status Reporter** (Priority 2, 1000ms): Reports system status and health metrics

#### `main_board/src/main_board.c`
**Purpose**: Main board firmware implementation with FreeRTOS task management.

**Key Functions**:
- **System Initialization**: Hardware setup, FreeRTOS initialization, task creation
- **Task Management**: Emergency handler, CAN manager, system controller, data logger, radio handler, power monitor, status reporter
- **Interrupt Handling**: CAN, UART, timer, and external interrupt handlers
- **System Coordination**: Board communication, status monitoring, error handling
- **Data Management**: SD card logging, telemetry processing, configuration management

**Core Implementation**:
```c
// Main board initialization
papyrus_status_t main_board_init(void);

// Task creation and management
void create_system_tasks(void);

// Interrupt service routines
void CAN1_RX0_IRQHandler(void);
void USART1_IRQHandler(void);
void TIM2_IRQHandler(void);
```

### Controller Framework (`controllers/`)

#### `controllers/framework/controller_base.h`
**Purpose**: Generic framework for all controller boards with polymorphic design.

**Key Structures**:
- **Controller Types**: SERVO, THERMOCOUPLE, UNKNOWN
- **Controller States**: INIT, NOT_CONFIG, OKAY, FATAL, DISABLED, DANGER
- **Board Status**: State tracking, statistics, subdevice states, error queues
- **Base Controller**: Configuration, CAN communication, UART debug, GPIO indicators

**Core Functions**:
```c
PapyrusStatus controller_base_init(ControllerBase *controller);
PapyrusStatus controller_hardware_init(ControllerBase *controller);
void papyrus_process_can_command(ControllerBase *base, ErrorEntry *err);
```

**Design Patterns**:
- **Polymorphic Interface**: Function pointers for derived class implementations
- **Error Management**: Comprehensive error tracking and escalation
- **State Machine**: Clear state transitions with validation
- **Hardware Abstraction**: Standardized hardware interface functions

#### `controllers/framework/controller_base.c`
**Purpose**: Implements common controller functionality and hardware abstraction.

**Key Functions**:
- **Initialization**: Hardware setup, configuration loading, CAN initialization
- **Command Processing**: CAN message parsing and routing to appropriate handlers
- **State Management**: State transitions with validation and error handling
- **Error Handling**: Error logging, escalation, and recovery procedures
- **Hardware Interface**: Standardized GPIO, UART, and CAN operations

#### `controllers/servo_controller/servo_controller.h`
**Purpose**: Servo control implementation with position feedback and current monitoring.

**Key Structures**:
- **Configuration**: PWM settings, position limits, current monitoring, movement parameters, safety limits, calibration data, feedback config
- **Status**: Current positions, target positions, movement status, error tracking, performance metrics
- **Commands**: SET_POSITION, SET_SPEED, ENABLE, DISABLE, STOP, HOME, CALIBRATE, SET_LIMITS, GET_POSITION, GET_STATUS

**Core Functions**:
```c
papyrus_status_t servo_set_position(servo_controller_t* servo_ctrl, uint8_t servo_id, uint16_t position, uint8_t speed);
papyrus_status_t servo_enable(servo_controller_t* servo_ctrl, uint8_t servo_id);
papyrus_status_t servo_emergency_stop(servo_controller_t* servo_ctrl);
papyrus_status_t servo_calibrate(servo_controller_t* servo_ctrl, uint8_t servo_id);
```

**Advanced Features**:
- **Trajectory Planning**: Smooth position transitions with acceleration/deceleration
- **Current Monitoring**: Stall detection and overcurrent protection
- **Position Feedback**: Potentiometer or encoder position sensing
- **Safety Limits**: Position and current limits with emergency stop capability
- **Calibration**: Automatic calibration and limit detection

#### `controllers/tc_controller/tc_controller.h`
**Purpose**: Thermocouple controller interface for temperature measurement and monitoring.

**Key Structures**:
- **Configuration**: Thermocouple type, cold junction compensation, temperature limits, sampling rate
- **Status**: Current temperatures, cold junction temperature, error status, calibration data
- **Commands**: READ_TEMPERATURE, SET_LIMITS, CALIBRATE, GET_STATUS, CONFIGURE

**Core Functions**:
```c
papyrus_status_t tc_controller_init(tc_controller_t* tc_ctrl, board_id_t board_id);
papyrus_status_t tc_read_temperature(tc_controller_t* tc_ctrl, uint8_t channel, float* temperature);
papyrus_status_t tc_set_limits(tc_controller_t* tc_ctrl, uint8_t channel, float min_temp, float max_temp);
papyrus_status_t tc_calibrate(tc_controller_t* tc_ctrl, uint8_t channel);
```

**Features**:
- **Multi-Channel Support**: Up to 4 thermocouple channels
- **Cold Junction Compensation**: Automatic cold junction temperature compensation
- **Temperature Limits**: Configurable high/low temperature alarms
- **Calibration**: Offset and scaling calibration for accuracy
- **Error Detection**: Open circuit, short circuit, and accuracy error detection

#### `controllers/tc_controller/tc_controller.c`
**Purpose**: Thermocouple controller implementation with SPI communication and temperature processing.

**Key Functions**:
- **Hardware Initialization**: SPI setup, GPIO configuration, interrupt setup
- **Temperature Reading**: SPI communication with MAX31855 amplifiers
- **Cold Junction Compensation**: Automatic compensation using onboard temperature sensor
- **Error Handling**: Detection and reporting of thermocouple faults
- **Data Processing**: Raw ADC to temperature conversion with calibration

#### `controllers/tc_controller/hardware_setup.c`
**Purpose**: Hardware-specific initialization and configuration for thermocouple controller.

**Key Functions**:
- **SPI Configuration**: Setup SPI interface for MAX31855 communication
- **GPIO Setup**: Configure thermocouple enable pins and status LEDs
- **Interrupt Configuration**: Setup interrupt handlers for temperature alarms
- **Power Management**: Control thermocouple amplifier power supplies
- **Hardware Validation**: Self-test of thermocouple circuits and amplifiers

#### `controllers/tc_controller/commands_tc1.h`
**Purpose**: CAN command definitions for thermocouple controller operations.

**Key Commands**:
- **TC_CMD_READ_TEMP**: Read temperature from specified channel
- **TC_CMD_SET_LIMITS**: Set temperature limits for alarm monitoring
- **TC_CMD_CALIBRATE**: Calibrate temperature offset and scaling
- **TC_CMD_GET_STATUS**: Get controller status and error information
- **TC_CMD_CONFIGURE**: Configure thermocouple type and parameters

#### `controllers/tc_controller/commands_tc1.c`
**Purpose**: Implementation of thermocouple CAN command handlers.

**Key Functions**:
- **Command Parsing**: Parse incoming CAN commands and extract parameters
- **Response Generation**: Generate appropriate CAN responses with data
- **Error Handling**: Validate commands and report errors via CAN
- **Data Formatting**: Format temperature data for CAN transmission
- **Status Reporting**: Report controller status and health information

#### `controllers/tc_controller/uart_debugger.c`
**Purpose**: UART debug interface for thermocouple controller development and testing.

**Key Functions**:
- **Debug Output**: Human-readable temperature and status information
- **Command Interface**: UART-based command interface for testing
- **Data Logging**: Real-time temperature data logging via UART
- **Error Reporting**: Detailed error information for debugging
- **Calibration Interface**: Interactive calibration via UART commands

#### `controllers/devices/tc_amplifier_max31855.h`
**Purpose**: Hardware abstraction layer for MAX31855 thermocouple amplifier.

**Key Functions**:
- **SPI Communication**: Low-level SPI interface for MAX31855
- **Data Conversion**: Convert raw SPI data to temperature values
- **Error Detection**: Detect open circuit, short circuit, and accuracy errors
- **Cold Junction**: Read cold junction temperature from onboard sensor
- **Configuration**: Configure amplifier settings and parameters

#### `controllers/devices/tc_amplifier_max31855.c`
**Purpose**: Implementation of MAX31855 thermocouple amplifier driver.

**Key Functions**:
- **SPI Transactions**: Handle SPI read/write operations to MAX31855
- **Temperature Calculation**: Convert thermocouple voltage to temperature
- **Error Processing**: Process and report amplifier error conditions
- **Data Validation**: Validate temperature readings and error flags
- **Hardware Interface**: Direct hardware control for amplifier operation

### Controller Target Files (`controllers/target/`)

#### `controllers/target/papyrus_hardware.h`
**Purpose**: Hardware abstraction layer interface for controller boards.

**Key Functions**:
- **GPIO Interface**: Pin configuration, digital I/O, interrupt setup
- **SPI Interface**: SPI communication for sensors and peripherals
- **UART Interface**: Serial communication for debugging and configuration
- **Timer Interface**: PWM generation and timer management
- **ADC Interface**: Analog-to-digital conversion for sensors
- **CAN Interface**: CAN bus communication and message handling

#### `controllers/target/papyrus_hardware.c`
**Purpose**: Hardware abstraction layer implementation for controller boards.

**Key Functions**:
- **Hardware Initialization**: GPIO, SPI, UART, timer, ADC, CAN setup
- **Peripheral Management**: Configure and manage all hardware peripherals
- **Interrupt Handling**: Setup and manage interrupt service routines
- **Error Handling**: Hardware error detection and reporting
- **Power Management**: Peripheral power control and monitoring

#### `controllers/target/commands_sys0.h`
**Purpose**: System command definitions for controller boards.

**Key Commands**:
- **SYS_CMD_RESET**: System reset command
- **SYS_CMD_GET_STATUS**: Get system status
- **SYS_CMD_SET_CONFIG**: Set system configuration
- **SYS_CMD_EMERGENCY_STOP**: Emergency stop command
- **SYS_CMD_SAFE_MODE**: Enter/exit safe mode
- **SYS_CMD_SELF_TEST**: Run self-test procedures

#### `controllers/target/commands_sys0.c`
**Purpose**: System command handler implementation for controller boards.

**Key Functions**:
- **Command Processing**: Parse and execute system commands
- **Response Generation**: Generate appropriate command responses
- **Error Handling**: Validate commands and report errors
- **Status Reporting**: Report system status and health information
- **Configuration Management**: Handle configuration updates

#### `controllers/target/startup.s`
**Purpose**: Assembly startup code for STM32C092FCP6 microcontroller.

**Key Functions**:
- **Vector Table**: Interrupt vector table initialization
- **Stack Setup**: Stack pointer initialization
- **Memory Initialization**: BSS and data section initialization
- **Clock Configuration**: System clock setup
- **Main Entry**: Jump to main application entry point

### Ground Station (`ground_station/`)

#### `ground_station/firmware/ground_station.h`
**Purpose**: Ground station firmware for radio communication and web interface management.

**Key Structures**:
- **Configuration**: Radio settings, network config, logging parameters, emergency controls
- **Status**: System state, radio status, network status, main board communication, logging status, emergency status
- **Telemetry**: Data packet structures for real-time monitoring
- **Commands**: Command packet structures for system control
- **Web Clients**: Client management for multi-user web interface

**Core Functions**:
```c
papyrus_status_t ground_station_init(void);
papyrus_status_t gs_radio_send(const uint8_t* data, uint16_t length);
papyrus_status_t gs_web_broadcast(const uint8_t* data, uint16_t length);
papyrus_status_t gs_emergency_stop(const char* reason);
papyrus_status_t gs_cmd_servo_position(board_id_t board_id, uint8_t servo_id, uint16_t position, uint8_t speed);
```

**Key Features**:
- **Radio Communication**: 433MHz/915MHz radio link with signal strength monitoring
- **Web Interface**: Real-time status display and control via web browser
- **Multi-Client Support**: Multiple web clients with operator mode control
- **Data Logging**: Telemetry and command logging to local storage
- **Emergency Controls**: Emergency stop and safe mode controls
- **Command Routing**: System command routing to appropriate boards

### Bus Debugger (`bus_debugger/`)

#### `bus_debugger/firmware/bus_debugger.h`
**Purpose**: Bus debugger firmware for CAN bus monitoring and debugging.

**Key Structures**:
- **Configuration**: CAN settings, display config, power management, logging parameters
- **Status**: System state, CAN traffic status, display status, power status, error tracking
- **CAN Monitoring**: Traffic analysis, message filtering, error detection
- **Display Interface**: LCD display management and user interface
- **Message Injection**: Test message generation and injection

**Core Functions**:
```c
papyrus_status_t bus_debugger_init(void);
papyrus_status_t bd_monitor_can_traffic(void);
papyrus_status_t bd_inject_test_message(const can_message_t* msg);
papyrus_status_t bd_update_display(void);
papyrus_status_t bd_log_can_message(const can_message_t* msg);
```

**Key Features**:
- **CAN Monitoring**: Passive CAN bus traffic analysis and logging
- **Message Injection**: Test message injection for system testing
- **Error Detection**: CAN bus error monitoring and reporting
- **LCD Display**: Real-time status display and user interface
- **Standalone Operation**: Independent power and processing capability
- **Data Logging**: CAN traffic logging for analysis and debugging

### Build System (`Makefile`)

**Purpose**: Comprehensive build system for all firmware components.

**Key Features**:
- **Multi-Target Support**: Main board, controller boards, ground station, bus debugger
- **Toolchain Integration**: ARM GCC, STM32CubeProgrammer, OpenOCD
- **Build Types**: Debug and Release configurations
- **Flash Targets**: Individual board flashing with verification
- **Debug Support**: GDB integration for development debugging
- **Testing**: Unit, integration, and hardware-in-loop testing
- **Documentation**: Doxygen documentation generation
- **Analysis**: Static analysis, memory usage, performance profiling

**Build Commands**:
```bash
make all                    # Build all targets
make main_board            # Build main board firmware
make servo_controller      # Build servo controller
make flash_main_board      # Flash main board
make debug_main_board      # Start debug session
make test_unit             # Run unit tests
make clean                 # Clean build artifacts
```

### Development Tools (`tools/`)

#### `tools/testing/test_framework.py`
**Purpose**: Comprehensive testing framework for firmware validation and verification.

**Key Features**:
- **Unit Testing**: Automated unit tests for individual functions and modules
- **Integration Testing**: System-level integration tests for board communication
- **Hardware-in-Loop**: Real hardware testing with simulated inputs
- **Performance Testing**: Timing and memory usage analysis
- **Regression Testing**: Automated regression test suites
- **Test Reporting**: Detailed test results and coverage reports

**Test Categories**:
- **CAN Protocol Tests**: Message construction, validation, error handling
- **Controller Tests**: Servo control, thermocouple reading, safety features
- **Main Board Tests**: FreeRTOS task scheduling, system coordination
- **Ground Station Tests**: Radio communication, web interface, data logging
- **Hardware Tests**: GPIO, SPI, I2C, PWM, ADC functionality

**Usage**:
```bash
# Run all tests
python tools/testing/test_framework.py --all

# Run specific test category
python tools/testing/test_framework.py --category can

# Run with hardware
python tools/testing/test_framework.py --hardware

# Generate coverage report
python tools/testing/test_framework.py --coverage
```

### Hardware Drivers (`drivers/`)

#### `drivers/stm32c0xx/`
**Purpose**: STM32C0xx family hardware abstraction layer and peripheral drivers.

**Key Components**:
- **HAL Drivers**: Hardware abstraction layer for STM32C092FCP6
- **Peripheral Drivers**: CAN, SPI, I2C, UART, PWM, ADC drivers
- **Clock Configuration**: System clock setup and peripheral clock management
- **Interrupt Handlers**: Interrupt service routines for all peripherals
- **GPIO Management**: Pin configuration and GPIO control functions

**Key Driver Files**:
- **`stm32c0xx_hal_fdcan.h/.c`**: CAN bus driver for SN65HVD232QDRG4Q1 transceiver
- **`stm32c0xx_hal_spi.h/.c`**: SPI driver for sensor communication (MAX31855, etc.)
- **`stm32c0xx_hal_tim.h/.c`**: Timer/PWM driver for servo control and timing
- **`stm32c0xx_hal_uart.h/.c`**: UART driver for debugging and configuration
- **`stm32c0xx_hal_gpio.h/.c`**: GPIO driver for digital I/O and interrupt handling
- **`stm32c0xx_hal_adc.h/.c`**: ADC driver for analog sensor reading
- **`stm32c0xx_hal_rcc.h/.c`**: Clock configuration and peripheral clock management
- **`stm32c0xx_hal_dma.h/.c`**: DMA driver for efficient data transfer
- **`stm32c0xx_hal_flash.h/.c`**: Flash memory driver for configuration storage
- **`stm32c0xx_hal_pwr.h/.c`**: Power management and low-power modes

#### `drivers/common/`
**Purpose**: Common hardware drivers shared across all boards.

**Key Components**:
- **CMSIS Core**: ARM Cortex-M core support files
- **CMSIS Compiler**: Compiler-specific definitions and macros
- **TrustZone**: Security and memory protection support
- **MPU**: Memory Protection Unit support for safety-critical applications
- **Cache**: Cache management for performance optimization

### Linker Scripts (`linker/`)

#### `linker/stm32c092xx_flash.ld`
**Purpose**: Linker script for STM32C092FCP6 flash memory layout and section placement.

**Key Sections**:
- **Memory Layout**: Flash and RAM memory region definitions
- **Section Placement**: Code, data, and stack section placement
- **Vector Table**: Interrupt vector table placement and configuration
- **Stack Configuration**: Stack size and placement for FreeRTOS
- **Heap Configuration**: Dynamic memory allocation for FreeRTOS heap

**Memory Map**:
```
Flash Memory (256KB):
- 0x08000000: Vector table and startup code
- 0x08001000: Application code
- 0x08020000: Configuration data
- 0x08030000: Bootloader (if present)

RAM Memory (32KB):
- 0x20000000: Stack and heap
- 0x20001000: Global variables
- 0x20002000: FreeRTOS task stacks
```

### Documentation (`docs/`)

#### `docs/IMPLEMENTATION_SUMMARY.md`
**Purpose**: Comprehensive implementation summary and technical reference document.

**Key Sections**:
- **System Architecture**: Detailed system design and component interactions
- **Implementation Phases**: Complete 8-phase implementation breakdown
- **Technical Specifications**: Detailed technical requirements and constraints
- **Integration Guide**: Step-by-step integration procedures
- **Testing Procedures**: Comprehensive testing methodology and procedures
- **Performance Analysis**: System performance characteristics and benchmarks
- **Maintenance Guide**: Long-term maintenance and support procedures

**Usage**:
- **Development Reference**: Technical details for firmware development
- **Integration Guide**: Step-by-step system integration procedures
- **Troubleshooting**: Comprehensive troubleshooting and debugging guide
- **Maintenance**: Long-term maintenance and support procedures

## Hardware Requirements

### Main Board (STM32H7)
- **MCU**: STM32H7 series (high-performance)
- **Memory**: 1MB Flash, 1MB RAM
- **Interfaces**: CAN, I2C, SPI, UART, USB-C
- **Storage**: SD card, external flash
- **Power**: 12V LiPo, USB power switching
- **Radio**: 433MHz/915MHz module

### Controller Boards (STM32C092FCP6)
- **MCU**: STM32C092FCP6
- **Memory**: 256KB Flash, 32KB RAM
- **Interfaces**: CAN, I2C, SPI, PWM
- **Power**: 3.3V from main board
- **Connectors**: JST-XH for CAN bus

### Programming Hardware
- **ST-Link V3** or **ST-Link V2** programmer
- **Tag-Connect 6-pin** cables
- **USB-C** cable for ground station

## Build System

### Prerequisites

```bash
# Install ARM toolchain
sudo apt-get install gcc-arm-none-eabi gdb-arm-none-eabi

# Install STM32CubeProgrammer
# Download from: https://www.st.com/en/development-tools/stm32cubeprog.html

# Install Python dependencies
pip install pytest pytest-cov
```

### Build Commands

```bash
# Build all targets
make all

# Build specific components
make main_board          # Main board firmware
make servo_controller    # Servo controller
make tc_controller       # Thermocouple controller
make ground_station      # Ground station
make bus_debugger        # Bus debugger

# Clean build artifacts
make clean
```

### Flash Commands

```bash
# Flash main board (requires ST-Link)
make flash_main_board

# Flash controller boards
make flash_servo
make flash_tc
make flash_io

# Flash ground station
make flash_gs

# Flash bus debugger
make flash_debugger
```

## CAN Bus Protocol

### Message ID Structure (11-bit)
```
Bits [10-8]: Priority (3 bits)
Bits [7-4]:  Source Board ID (4 bits)  
Bits [3-0]:  Message Type (4 bits)
```

### Board IDs
- `0x0`: Main Board
- `0x1`: Servo Controller 1
- `0x2`: Servo Controller 2
- `0x3`: Servo Controller 3
- `0x4`: Thermocouple Controller
- `0x5`: General I/O Controller
- `0x6`: Ground Station
- `0x7`: Bus Debugger

### Message Types
- `0x0`: System Command
- `0x1`: Status Report
- `0x2`: Servo Command
- `0x3`: Thermocouple Data
- `0x4`: Emergency Stop
- `0x5`: Configuration
- `0x6`: Data Log
- `0x7`: Error Report

### Priority Levels
- `0x0`: Emergency (highest)
- `0x1`: Critical
- `0x2`: High
- `0x3`: Normal
- `0x4`: Low
- `0x5`: Background
- `0x6`: Debug
- `0x7`: Maintenance (lowest)

## Main Board Firmware

### FreeRTOS Tasks

| Task | Priority | Period (ms) | Stack (words) | Function |
|------|----------|-------------|---------------|----------|
| Emergency Handler | 7 | 1 | 512 | Emergency stop processing |
| CAN Manager | 6 | 10 | 1024 | CAN bus communication |
| System Controller | 5 | 50 | 2048 | System coordination |
| Data Logger | 4 | 100 | 1024 | SD card logging |
| Radio Handler | 4 | 20 | 1024 | Radio communication |
| Power Monitor | 3 | 500 | 512 | Power management |
| Status Reporter | 2 | 1000 | 512 | Status reporting |

### Key Functions

```c
// Initialize main board
papyrus_status_t main_board_init(void);

// Send command to controller
papyrus_status_t main_board_send_command(board_id_t target, 
                                       system_command_t cmd, 
                                       const uint8_t* data);

// Request status from controller
papyrus_status_t main_board_request_status(board_id_t target);

// Check controller health
papyrus_status_t main_board_check_health(board_id_t target);
```

## Controller Framework

### Base Controller Structure

```c
typedef struct {
    controller_config_t config;      // Configuration
    controller_status_t status;      // Current status
    device_interface_t devices[8];   // Device interfaces
    CAN_HandleTypeDef* hcan;        // CAN handle
    
    // Function pointers for derived classes
    papyrus_status_t (*init_hardware)(void);
    papyrus_status_t (*update_devices)(void);
    papyrus_status_t (*process_command)(const can_message_t* msg);
    papyrus_status_t (*read_sensors)(void);
    papyrus_status_t (*enter_safe_mode)(void);
    papyrus_status_t (*emergency_stop)(void);
    papyrus_status_t (*self_test)(void);
} controller_base_t;
```

### Controller Types
- **Servo Controller**: PWM servo control with position feedback
- **Thermocouple Controller**: SPI thermocouple reading with cold junction compensation
- **General I/O Controller**: Digital I/O, analog inputs, relay control

## Servo Controller

### Features
- **PWM Control**: 50Hz servo signals with variable pulse width
- **Position Feedback**: Potentiometer or encoder position sensing
- **Current Monitoring**: ADC current sensing for stall detection
- **Trajectory Planning**: Smooth position transitions
- **Safety Limits**: Position and current limits with emergency stop

### Configuration

```c
typedef struct {
    uint16_t pwm_frequency;         // PWM frequency (Hz)
    uint16_t min_position;           // Minimum position (ADC counts)
    uint16_t max_position;           // Maximum position (ADC counts)
    uint16_t current_limit;          // Current limit (mA)
    uint16_t stall_current;          // Stall detection threshold (mA)
    uint16_t move_speed;             // Movement speed (steps/sec)
    uint16_t acceleration;           // Acceleration (steps/sec²)
    uint8_t feedback_type;           // Feedback sensor type
    uint8_t servo_count;             // Number of servos (1-3)
} servo_config_t;
```

### Commands

```c
// Set servo position
papyrus_status_t servo_set_position(uint8_t servo_id, uint16_t position);

// Enable/disable servo
papyrus_status_t servo_enable(uint8_t servo_id, bool enable);

// Emergency stop all servos
papyrus_status_t servo_emergency_stop(void);

// Home servo to reference position
papyrus_status_t servo_home(uint8_t servo_id);
```

## Ground Station

### Features
- **Radio Communication**: 433MHz/915MHz radio link
- **Web Interface**: Real-time status display and control
- **Data Logging**: Telemetry and command logging
- **Emergency Controls**: Emergency stop and safe mode
- **Multi-Client Support**: Multiple web clients with access control

### Web Interface

```bash
# Start ground station
make ground_station

# Access web interface
# Open browser to: http://localhost:8080
```

### Emergency Controls
- **Emergency Stop**: Immediate system shutdown
- **Safe Mode**: Reduced functionality mode
- **System Reset**: Full system restart
- **Status Monitoring**: Real-time system health

## Bus Debugger

### Features
- **CAN Monitoring**: Passive CAN bus traffic analysis
- **Message Injection**: Test message injection capability
- **Error Detection**: CAN bus error monitoring
- **LCD Display**: Real-time status display
- **Standalone Operation**: Independent power and processing

### Usage

```bash
# Connect to CAN bus
# Power debugger with LiPo battery
# LCD shows real-time CAN traffic

# Inject test message
# Use debugger interface to send test CAN messages
```

## Development Workflow

### 1. Initial Setup

```bash
# Clone repository
git clone <repository-url>
cd Papyrus-RT-Firmware

# Install dependencies
sudo apt-get install gcc-arm-none-eabi stm32cubeprog

# Build all components
make all
```

### 2. Hardware Connection

```bash
# Connect ST-Link to main board
# Tag-Connect 6-pin cable:
# Pin 1: VCC (3.3V)
# Pin 2: SWCLK
# Pin 3: GND
# Pin 4: SWDIO
# Pin 5: NRST
# Pin 6: GND

# Connect controller boards via CAN bus
# JST-XH connectors for CAN communication
```

### 3. Flash Firmware

```bash
# Flash main board
make flash_main_board

# Flash controller boards
make flash_servo
make flash_tc
make flash_io

# Verify flash
STM32_Programmer_CLI -c port=SWD -v
```

### 4. System Testing

```bash
# Run unit tests
make test_unit

# Run integration tests
make test_integration

# Run hardware-in-loop tests
make test_hardware
```

### 5. Debugging

```bash
# Start debug session
make debug_main_board

# Monitor CAN traffic
make monitor_can

# Check system status
make status
```

## Configuration

### System Configuration (`papyrus_config.h`)

```c
// Timing configuration
#define SYSTEM_TICK_RATE_HZ        1000
#define CAN_BUS_BAUDRATE          500000
#define RADIO_FREQUENCY           915000000

// Safety limits
#define MAX_TEMPERATURE_C          85
#define MAX_CURRENT_MA            5000
#define WATCHDOG_TIMEOUT_MS       1000

// Task priorities
#define TASK_PRIORITY_EMERGENCY   7
#define TASK_PRIORITY_CAN         6
#define TASK_PRIORITY_CONTROL     5
```

### Board-Specific Configuration

```c
// Main board config
typedef struct {
    uint32_t can_baudrate;
    uint32_t radio_frequency;
    uint16_t watchdog_timeout;
    uint8_t log_level;
    bool emergency_stop_enabled;
} main_board_config_t;

// Controller config
typedef struct {
    uint16_t update_rate_hz;
    uint16_t timeout_ms;
    uint8_t can_id;
    bool safe_mode_enabled;
} controller_config_t;
```

## Error Handling

### Error Codes

```c
typedef enum {
    PAPYRUS_OK = 0,
    PAPYRUS_ERROR = -1,
    PAPYRUS_ERROR_TIMEOUT = -2,
    PAPYRUS_ERROR_INVALID_PARAM = -3,
    PAPYRUS_ERROR_HARDWARE = -4,
    PAPYRUS_ERROR_COMMUNICATION = -5,
    PAPYRUS_ERROR_MEMORY = -6,
    PAPYRUS_ERROR_CONFIG = -7,
    PAPYRUS_ERROR_EMERGENCY = -8
} papyrus_status_t;
```

### Emergency Procedures

```c
// Emergency stop function
void papyrus_emergency_stop(error_code_t error);

// Safe mode entry
papyrus_status_t enter_safe_mode(void);

// System recovery
papyrus_status_t system_recovery(void);
```

## Performance Characteristics

### Timing Requirements
- **Emergency Stop**: < 1ms response time
- **CAN Communication**: 10ms update rate
- **Servo Control**: 20ms control loop
- **Data Logging**: 100ms logging rate
- **Status Reporting**: 1000ms status rate

### Memory Usage
- **Main Board**: 512KB RAM, 1MB Flash
- **Controller Boards**: 32KB RAM, 256KB Flash
- **FreeRTOS Heap**: 32KB
- **CAN Buffer**: 64 messages

### Power Consumption
- **Main Board**: 500mA @ 3.3V
- **Controller Board**: 100mA @ 3.3V
- **Ground Station**: 200mA @ 5V
- **Bus Debugger**: 150mA @ 3.3V

## Troubleshooting

### Common Issues

**CAN Bus Communication Failure**
```bash
# Check CAN bus termination
# Verify 120Ω termination resistors

# Check CAN transceiver
# Verify SN65HVD232QDRG4Q1 connections

# Monitor CAN traffic
make monitor_can
```

**Flash Programming Issues**
```bash
# Check ST-Link connection
STM32_Programmer_CLI -c port=SWD

# Reset target if needed
STM32_Programmer_CLI -c port=SWD -rst

# Verify flash contents
STM32_Programmer_CLI -c port=SWD -v
```

**Servo Control Problems**
```bash
# Check servo power supply
# Verify PWM signal on oscilloscope
# Test servo feedback circuit
# Check current monitoring
```

### Debug Commands

```bash
# System status
make status

# CAN bus monitor
make monitor_can

# Memory usage
make memory_usage

# Performance analysis
make performance
```

## Maintenance

### Regular Tasks
- **Weekly**: Check system logs for errors
- **Monthly**: Verify CAN bus integrity
- **Quarterly**: Calibrate sensors and servos
- **Annually**: Full system test and validation

### Firmware Updates
```bash
# Backup current firmware
make backup

# Update firmware
make flash_all

# Verify update
make verify_flash
```