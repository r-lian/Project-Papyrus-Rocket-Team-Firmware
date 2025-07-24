# Papyrus Rocket Avionics Firmware Makefile
# Author: Papyrus Avionics Team
# Date: 2024

# Project Configuration
PROJECT_NAME = papyrus-firmware
VERSION_MAJOR = 1
VERSION_MINOR = 0
VERSION_PATCH = 0
BUILD_NUMBER = 1

# Build Configuration
BUILD_TYPE ?= Release
VERBOSE ?= 0
PARALLEL_JOBS ?= 4

# Toolchain Configuration
ARM_TOOLCHAIN_PREFIX = arm-none-eabi-
CC = $(ARM_TOOLCHAIN_PREFIX)gcc
CXX = $(ARM_TOOLCHAIN_PREFIX)g++
AS = $(ARM_TOOLCHAIN_PREFIX)as
LD = $(ARM_TOOLCHAIN_PREFIX)ld
OBJCOPY = $(ARM_TOOLCHAIN_PREFIX)objcopy
OBJDUMP = $(ARM_TOOLCHAIN_PREFIX)objdump
SIZE = $(ARM_TOOLCHAIN_PREFIX)size
GDB = $(ARM_TOOLCHAIN_PREFIX)gdb

# Host tools
PYTHON = python3
STLINK = st-flash
OPENOCD = openocd

# Directory Structure
BUILD_DIR = build
COMMON_DIR = common
MAIN_BOARD_DIR = main_board
CONTROLLERS_DIR = controllers
GROUND_STATION_DIR = ground_station
BUS_DEBUGGER_DIR = bus_debugger
TOOLS_DIR = tools
DRIVERS_DIR = drivers
DOCS_DIR = docs

# Output Directories
BUILD_COMMON = $(BUILD_DIR)/common
BUILD_MAIN = $(BUILD_DIR)/main_board
BUILD_SERVO = $(BUILD_DIR)/servo_controller
BUILD_TC = $(BUILD_DIR)/tc_controller
BUILD_IO = $(BUILD_DIR)/io_controller
BUILD_GS = $(BUILD_DIR)/ground_station
BUILD_DEBUGGER = $(BUILD_DIR)/bus_debugger

# Common Compiler Flags
COMMON_CFLAGS = -Wall -Wextra -Werror -std=c23 -fdata-sections -ffunction-sections -fanalyzer
COMMON_CPPFLAGS = -I$(COMMON_DIR)/protocols -I$(COMMON_DIR)/drivers -I$(COMMON_DIR)/utils -I$(COMMON_DIR)/config -I$(DRIVERS_DIR)/common

# Target-specific flags
MAIN_BOARD_CFLAGS = $(COMMON_CFLAGS) -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
CONTROLLER_CFLAGS = $(COMMON_CFLAGS) -DSTM32C092xx=1 -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -I$(DRIVERS_DIR)/stm32c0xx
GS_CFLAGS = $(COMMON_CFLAGS) -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Debug/Release specific flags
ifeq ($(BUILD_TYPE),Debug)
    OPT_FLAGS = -Og -g3 -DDEBUG=1
else
    OPT_FLAGS = -O2 -g1 -DNDEBUG=1
endif

# Linker Flags
MAIN_BOARD_LDFLAGS = -T linker/stm32h7xx_flash.ld -Wl,--gc-sections -Wl,--print-memory-usage
CONTROLLER_LDFLAGS = -T linker/stm32c092xx_flash.ld -Wl,--gc-sections -Wl,--print-memory-usage

MAIN_BOARD_STARTUP = $(COMMON_DIR)/target/stm32h7xx/startup.s
CONTROLLER_STARTUP = $(COMMON_DIR)/target/stm32c092xx/startup.s

# Source Files
COMMON_SOURCES = $(wildcard $(COMMON_DIR)/*/*.c) $(wildcard $(COMMON_DIR)/*/*.s)
MAIN_BOARD_SOURCES = $(wildcard $(MAIN_BOARD_DIR)/src/*.c)
SERVO_SOURCES = $(wildcard $(CONTROLLERS_DIR)/servo_controller/*.c)
TC_SOURCES = $(wildcard $(CONTROLLERS_DIR)/tc_controller/*.c)
IO_SOURCES = $(wildcard $(CONTROLLERS_DIR)/io_controller/*.c)
GS_SOURCES = $(wildcard $(GROUND_STATION_DIR)/firmware/*.c)
DEBUGGER_SOURCES = $(wildcard $(BUS_DEBUGGER_DIR)/firmware/*.c)

# Object Files
COMMON_OBJECTS = $(COMMON_SOURCES:%.c=$(BUILD_COMMON)/%.o)
MAIN_BOARD_OBJECTS = $(MAIN_BOARD_SOURCES:%.c=$(BUILD_MAIN)/%.o) $(COMMON_OBJECTS)
SERVO_OBJECTS = $(SERVO_SOURCES:%.c=$(BUILD_SERVO)/%.o) $(COMMON_OBJECTS)
TC_OBJECTS = $(TC_SOURCES:%.c=$(BUILD_TC)/%.o) $(COMMON_OBJECTS) $(CONTROLLER_STARTUP:%.s=$(BUILD_COMMON)/%.o)
IO_OBJECTS = $(IO_SOURCES:%.c=$(BUILD_IO)/%.o) $(COMMON_OBJECTS)
GS_OBJECTS = $(GS_SOURCES:%.c=$(BUILD_GS)/%.o) $(COMMON_OBJECTS)
DEBUGGER_OBJECTS = $(DEBUGGER_SOURCES:%.c=$(BUILD_DEBUGGER)/%.o) $(COMMON_OBJECTS)

# Target Binaries
MAIN_BOARD_ELF = $(BUILD_MAIN)/main_board.elf
SERVO_ELF = $(BUILD_SERVO)/servo_controller.elf
TC_ELF = $(BUILD_TC)/tc_controller.elf
IO_ELF = $(BUILD_IO)/io_controller.elf
GS_ELF = $(BUILD_GS)/ground_station.elf
DEBUGGER_ELF = $(BUILD_DEBUGGER)/bus_debugger.elf

# All targets
ALL_TARGETS = main_board servo_controller tc_controller io_controller ground_station bus_debugger

# Default target
.PHONY: all
all: $(ALL_TARGETS)

# Help target
.PHONY: help
help:
	@echo "Papyrus Firmware Build System"
	@echo "=============================="
	@echo ""
	@echo "Available targets:"
	@echo "  all               - Build all firmware components"
	@echo "  main_board        - Build main board firmware"
	@echo "  servo_controller  - Build servo controller firmware"
	@echo "  tc_controller     - Build thermocouple controller firmware"
	@echo "  io_controller     - Build I/O controller firmware"
	@echo "  ground_station    - Build ground station firmware"
	@echo "  bus_debugger      - Build bus debugger firmware"
	@echo "  clean             - Clean all build outputs"
	@echo "  flash_<target>    - Flash firmware to target board"
	@echo "  debug_<target>    - Start GDB debug session"
	@echo "  test              - Run unit tests"
	@echo "  docs              - Generate documentation"
	@echo "  release           - Create release package"
	@echo ""
	@echo "Build options:"
	@echo "  BUILD_TYPE=Debug|Release  - Build type (default: Release)"
	@echo "  VERBOSE=1                 - Verbose build output"
	@echo "  PARALLEL_JOBS=N           - Number of parallel jobs (default: 4)"

# Main Board Target
.PHONY: main_board
main_board: $(MAIN_BOARD_ELF)

$(MAIN_BOARD_ELF): $(MAIN_BOARD_OBJECTS) | $(BUILD_MAIN)
	@echo "Linking main board firmware..."
	$(CC) $(MAIN_BOARD_CFLAGS) $(OPT_FLAGS) $(MAIN_BOARD_OBJECTS) -o $@ $(MAIN_BOARD_LDFLAGS)
	$(OBJCOPY) -O binary $@ $(BUILD_MAIN)/main_board.bin
	$(OBJCOPY) -O ihex $@ $(BUILD_MAIN)/main_board.hex
	$(SIZE) $@

# Servo Controller Target
.PHONY: servo_controller
servo_controller: $(SERVO_ELF)

$(SERVO_ELF): $(SERVO_OBJECTS) | $(BUILD_SERVO)
	@echo "Linking servo controller firmware..."
	$(CC) $(CONTROLLER_CFLAGS) $(OPT_FLAGS) $(SERVO_OBJECTS) -o $@ $(CONTROLLER_LDFLAGS)
	$(OBJCOPY) -O binary $@ $(BUILD_SERVO)/servo_controller.bin
	$(OBJCOPY) -O ihex $@ $(BUILD_SERVO)/servo_controller.hex
	$(SIZE) $@

# Thermocouple Controller Target
.PHONY: tc_controller
tc_controller: $(TC_ELF)

$(TC_ELF): $(TC_OBJECTS) | $(BUILD_TC)
	@echo "Linking thermocouple controller firmware..."
	$(CC) $(CONTROLLER_CFLAGS) $(OPT_FLAGS) $(TC_OBJECTS) -o $@ $(CONTROLLER_LDFLAGS)
	$(OBJCOPY) -O binary $@ $(BUILD_TC)/tc_controller.bin
	$(OBJCOPY) -O ihex $@ $(BUILD_TC)/tc_controller.hex
	$(SIZE) $@

# I/O Controller Target
.PHONY: io_controller
io_controller: $(IO_ELF)

$(IO_ELF): $(IO_OBJECTS) | $(BUILD_IO)
	@echo "Linking I/O controller firmware..."
	$(CC) $(CONTROLLER_CFLAGS) $(OPT_FLAGS) $(IO_OBJECTS) -o $@ $(CONTROLLER_LDFLAGS)
	$(OBJCOPY) -O binary $@ $(BUILD_IO)/io_controller.bin
	$(OBJCOPY) -O ihex $@ $(BUILD_IO)/io_controller.hex
	$(SIZE) $@

# Ground Station Target
.PHONY: ground_station
ground_station: $(GS_ELF)

$(GS_ELF): $(GS_OBJECTS) | $(BUILD_GS)
	@echo "Linking ground station firmware..."
	$(CC) $(GS_CFLAGS) $(OPT_FLAGS) $(GS_OBJECTS) -o $@ $(CONTROLLER_LDFLAGS)
	$(OBJCOPY) -O binary $@ $(BUILD_GS)/ground_station.bin
	$(OBJCOPY) -O ihex $@ $(BUILD_GS)/ground_station.hex
	$(SIZE) $@

# Bus Debugger Target
.PHONY: bus_debugger
bus_debugger: $(DEBUGGER_ELF)

$(DEBUGGER_ELF): $(DEBUGGER_OBJECTS) | $(BUILD_DEBUGGER)
	@echo "Linking bus debugger firmware..."
	$(CC) $(CONTROLLER_CFLAGS) $(OPT_FLAGS) $(DEBUGGER_OBJECTS) -o $@ $(CONTROLLER_LDFLAGS)
	$(OBJCOPY) -O binary $@ $(BUILD_DEBUGGER)/bus_debugger.bin
	$(OBJCOPY) -O ihex $@ $(BUILD_DEBUGGER)/bus_debugger.hex
	$(SIZE) $@

# Common object compilation
$(BUILD_COMMON)/%.o: %.c | $(BUILD_COMMON)
	@mkdir -p $(dir $@)
	@echo "Compiling $<..."
	$(CC) $(COMMON_CFLAGS) $(COMMON_CPPFLAGS) $(OPT_FLAGS) -c $< -o $@

$(BUILD_COMMON)/%.o: %.s | $(BUILD_COMMON)
	@mkdir -p $(dir $@)
	@echo "Assembling $<..."
	$(AS) $< -o $@


# Main board object compilation
$(BUILD_MAIN)/%.o: %.c | $(BUILD_MAIN)
	@mkdir -p $(dir $@)
	@echo "Compiling $< for main board..."
	$(CC) $(MAIN_BOARD_CFLAGS) $(COMMON_CPPFLAGS) -I$(MAIN_BOARD_DIR)/inc $(OPT_FLAGS) -c $< -o $@

# Controller object compilation
$(BUILD_SERVO)/%.o $(BUILD_TC)/%.o $(BUILD_IO)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling $< for controller..."
	$(CC) $(CONTROLLER_CFLAGS) $(COMMON_CPPFLAGS) -I$(CONTROLLERS_DIR)/framework $(OPT_FLAGS) -c $< -o $@

# Ground station object compilation
$(BUILD_GS)/%.o: %.c | $(BUILD_GS)
	@mkdir -p $(dir $@)
	@echo "Compiling $< for ground station..."
	$(CC) $(GS_CFLAGS) $(COMMON_CPPFLAGS) -I$(GROUND_STATION_DIR)/firmware $(OPT_FLAGS) -c $< -o $@

# Bus debugger object compilation
$(BUILD_DEBUGGER)/%.o: %.c | $(BUILD_DEBUGGER)
	@mkdir -p $(dir $@)
	@echo "Compiling $< for bus debugger..."
	$(CC) $(CONTROLLER_CFLAGS) $(COMMON_CPPFLAGS) -I$(BUS_DEBUGGER_DIR)/firmware $(OPT_FLAGS) -c $< -o $@

# Create build directories
$(BUILD_DIR) $(BUILD_COMMON) $(BUILD_MAIN) $(BUILD_SERVO) $(BUILD_TC) $(BUILD_IO) $(BUILD_GS) $(BUILD_DEBUGGER):
	@mkdir -p $@

# Flash targets
.PHONY: flash_main_board flash_servo flash_tc flash_io flash_gs flash_debugger
flash_main_board: $(MAIN_BOARD_ELF)
	$(STLINK) write $(BUILD_MAIN)/main_board.bin 0x08000000

flash_servo: $(SERVO_ELF)
	$(STLINK) write $(BUILD_SERVO)/servo_controller.bin 0x08000000

flash_tc: $(TC_ELF)
	$(STLINK) write $(BUILD_TC)/tc_controller.bin 0x08000000

flash_io: $(IO_ELF)
	$(STLINK) write $(BUILD_IO)/io_controller.bin 0x08000000

flash_gs: $(GS_ELF)
	$(STLINK) write $(BUILD_GS)/ground_station.bin 0x08000000

flash_debugger: $(DEBUGGER_ELF)
	$(STLINK) write $(BUILD_DEBUGGER)/bus_debugger.bin 0x08000000

# Debug targets
.PHONY: debug_main_board debug_servo debug_tc debug_io debug_gs debug_debugger
debug_main_board: $(MAIN_BOARD_ELF)
	$(GDB) $< -ex "target extended-remote localhost:3333"

debug_servo: $(SERVO_ELF)
	$(GDB) $< -ex "target extended-remote localhost:3333"

debug_tc: $(TC_ELF)
	$(GDB) $< -ex "target extended-remote localhost:3333"

debug_io: $(IO_ELF)
	$(GDB) $< -ex "target extended-remote localhost:3333"

debug_gs: $(GS_ELF)
	$(GDB) $< -ex "target extended-remote localhost:3333"

debug_debugger: $(DEBUGGER_ELF)
	$(GDB) $< -ex "target extended-remote localhost:3333"

# Testing
.PHONY: test test_unit test_integration test_hardware
test: test_unit test_integration

test_unit:
	@echo "Running unit tests..."
	$(PYTHON) $(TOOLS_DIR)/testing/run_unit_tests.py

test_integration:
	@echo "Running integration tests..."
	$(PYTHON) $(TOOLS_DIR)/testing/run_integration_tests.py

test_hardware:
	@echo "Running hardware-in-the-loop tests..."
	$(PYTHON) $(TOOLS_DIR)/testing/run_hardware_tests.py

# Documentation
.PHONY: docs docs_clean
docs:
	@echo "Generating documentation..."
	doxygen $(DOCS_DIR)/Doxyfile
	$(PYTHON) $(TOOLS_DIR)/utilities/generate_docs.py

docs_clean:
	rm -rf $(DOCS_DIR)/html $(DOCS_DIR)/latex

# Release package
.PHONY: release
release: all docs
	@echo "Creating release package..."
	$(PYTHON) $(TOOLS_DIR)/utilities/create_release.py $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)-$(BUILD_NUMBER)

# Lint and code quality
.PHONY: lint format
lint:
	@echo "Running code linter..."
	$(PYTHON) $(TOOLS_DIR)/utilities/run_linter.py

format:
	@echo "Formatting code..."
	find . -name "*.c" -o -name "*.h" | xargs clang-format -i

# Static analysis
.PHONY: analyze
analyze:
	@echo "Running static analysis..."
	cppcheck --enable=all --suppress=missingIncludeSystem --xml --xml-version=2 . 2> $(BUILD_DIR)/cppcheck.xml

# Memory usage analysis
.PHONY: memory_usage
memory_usage: all
	@echo "Analyzing memory usage..."
	$(PYTHON) $(TOOLS_DIR)/utilities/analyze_memory.py $(BUILD_DIR)

# Clean targets
.PHONY: clean clean_all
clean:
	rm -rf $(BUILD_DIR)

clean_all: clean docs_clean

# Version information
.PHONY: version
version:
	@echo "Papyrus Firmware Version: $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)-$(BUILD_NUMBER)"
	@echo "Build Type: $(BUILD_TYPE)"
	@echo "Toolchain: $(CC) --version | head -1"

# Install toolchain (for CI/development setup)
.PHONY: install_toolchain
install_toolchain:
	@echo "Installing ARM toolchain..."
	$(PYTHON) $(TOOLS_DIR)/utilities/install_toolchain.py

# Continuous Integration targets
.PHONY: ci_build ci_test ci_release
ci_build: clean all lint analyze

ci_test: test

ci_release: ci_build ci_test release

# Board-specific configurations
.PHONY: config_main_board config_servo config_tc config_io config_gs config_debugger
config_main_board:
	@echo "Configuring main board..."
	$(PYTHON) $(TOOLS_DIR)/utilities/configure_board.py main_board

config_servo:
	@echo "Configuring servo controller..."
	$(PYTHON) $(TOOLS_DIR)/utilities/configure_board.py servo_controller

config_tc:
	@echo "Configuring thermocouple controller..."
	$(PYTHON) $(TOOLS_DIR)/utilities/configure_board.py tc_controller

config_io:
	@echo "Configuring I/O controller..."
	$(PYTHON) $(TOOLS_DIR)/utilities/configure_board.py io_controller

config_gs:
	@echo "Configuring ground station..."
	$(PYTHON) $(TOOLS_DIR)/utilities/configure_board.py ground_station

config_debugger:
	@echo "Configuring bus debugger..."
	$(PYTHON) $(TOOLS_DIR)/utilities/configure_board.py bus_debugger

# Verbose output control
ifeq ($(VERBOSE),1)
    Q =
else
    Q = @
endif

# Include dependency files
-include $(BUILD_DIR)/*.d

# Export compilation database for IDE support
.PHONY: compile_commands
compile_commands:
	bear -- make all

# Print variables for debugging
.PHONY: print-%
print-%:
	@echo $* = $($*)

# Default goal
.DEFAULT_GOAL := all 
