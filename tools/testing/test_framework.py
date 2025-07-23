#!/usr/bin/env python3
"""
Papyrus Firmware Testing Framework
Author: Papyrus Avionics Team
Date: 2024

Comprehensive testing framework for validation of:
- Individual board firmware
- Inter-board communication
- System integration
- Hardware-in-the-loop testing
- Performance validation
"""

import sys
import os
import time
import json
import logging
import argparse
import threading
import serial
import socket
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import unittest

# Test Result Types
class TestResult(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    ERROR = "ERROR"

@dataclass
class TestCase:
    name: str
    description: str
    category: str
    timeout: float
    setup_func: Optional[callable] = None
    test_func: Optional[callable] = None
    teardown_func: Optional[callable] = None
    expected_result: TestResult = TestResult.PASS
    actual_result: TestResult = TestResult.SKIP
    error_message: str = ""
    duration: float = 0.0
    data: Dict[str, Any] = None

class PapyrusTestFramework:
    """Main testing framework for Papyrus firmware"""
    
    def __init__(self, config_file: str = "test_config.json"):
        self.config = self._load_config(config_file)
        self.test_cases: List[TestCase] = []
        self.results: Dict[str, TestResult] = {}
        self.serial_connections: Dict[str, serial.Serial] = {}
        self.socket_connections: Dict[str, socket.socket] = {}
        self.logger = self._setup_logging()
        
    def _load_config(self, config_file: str) -> Dict:
        """Load test configuration"""
        try:
            with open(config_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return self._default_config()
    
    def _default_config(self) -> Dict:
        """Default test configuration"""
        return {
            "serial_ports": {
                "main_board": "/dev/ttyUSB0",
                "servo_controller": "/dev/ttyUSB1",
                "tc_controller": "/dev/ttyUSB2",
                "ground_station": "/dev/ttyUSB3",
                "bus_debugger": "/dev/ttyUSB4"
            },
            "network": {
                "ground_station_ip": "192.168.1.100",
                "web_port": 8080,
                "websocket_port": 8081
            },
            "timeouts": {
                "default": 30.0,
                "communication": 5.0,
                "startup": 10.0,
                "emergency_stop": 1.0
            },
            "test_data": {
                "servo_positions": [0, 1024, 2048, 3072, 4095],
                "test_temperatures": [-50, 0, 25, 100, 500, 1000],
                "can_ids": [0x100, 0x200, 0x300, 0x400]
            }
        }
    
    def _setup_logging(self) -> logging.Logger:
        """Setup logging configuration"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(levelname)s] %(message)s',
            handlers=[
                logging.FileHandler('papyrus_test.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
        return logging.getLogger(__name__)
    
    def add_test_case(self, test_case: TestCase):
        """Add a test case to the framework"""
        self.test_cases.append(test_case)
        self.logger.info(f"Added test case: {test_case.name}")
    
    def setup_hardware_connections(self) -> bool:
        """Setup connections to hardware boards"""
        self.logger.info("Setting up hardware connections...")
        
        # Setup serial connections
        for board, port in self.config["serial_ports"].items():
            try:
                self.serial_connections[board] = serial.Serial(
                    port, 115200, timeout=1.0
                )
                self.logger.info(f"Connected to {board} on {port}")
            except Exception as e:
                self.logger.error(f"Failed to connect to {board}: {e}")
                return False
        
        # Setup network connections
        try:
            gs_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            gs_socket.connect((
                self.config["network"]["ground_station_ip"],
                self.config["network"]["web_port"]
            ))
            self.socket_connections["ground_station"] = gs_socket
            self.logger.info("Connected to ground station web interface")
        except Exception as e:
            self.logger.error(f"Failed to connect to ground station: {e}")
            return False
        
        return True
    
    def cleanup_connections(self):
        """Cleanup all hardware connections"""
        for board, conn in self.serial_connections.items():
            try:
                conn.close()
                self.logger.info(f"Closed connection to {board}")
            except:
                pass
        
        for name, sock in self.socket_connections.items():
            try:
                sock.close()
                self.logger.info(f"Closed socket connection to {name}")
            except:
                pass
    
    def send_command(self, board: str, command: str, timeout: float = 5.0) -> str:
        """Send command to a board and get response"""
        if board not in self.serial_connections:
            raise ValueError(f"No connection to board: {board}")
        
        conn = self.serial_connections[board]
        conn.write(f"{command}\n".encode())
        
        start_time = time.time()
        response = ""
        while time.time() - start_time < timeout:
            if conn.in_waiting:
                response += conn.read(conn.in_waiting).decode()
                if response.endswith('\n'):
                    break
            time.sleep(0.01)
        
        return response.strip()
    
    def run_test_case(self, test_case: TestCase) -> TestResult:
        """Run a single test case"""
        self.logger.info(f"Running test: {test_case.name}")
        start_time = time.time()
        
        try:
            # Setup phase
            if test_case.setup_func:
                test_case.setup_func()
            
            # Test execution phase
            if test_case.test_func:
                result = test_case.test_func()
                test_case.actual_result = result if result else TestResult.PASS
            else:
                test_case.actual_result = TestResult.SKIP
                
        except AssertionError as e:
            test_case.actual_result = TestResult.FAIL
            test_case.error_message = str(e)
            self.logger.error(f"Test failed: {test_case.name} - {e}")
            
        except Exception as e:
            test_case.actual_result = TestResult.ERROR
            test_case.error_message = str(e)
            self.logger.error(f"Test error: {test_case.name} - {e}")
            
        finally:
            # Teardown phase
            if test_case.teardown_func:
                try:
                    test_case.teardown_func()
                except Exception as e:
                    self.logger.warning(f"Teardown error for {test_case.name}: {e}")
            
            test_case.duration = time.time() - start_time
            
        self.logger.info(f"Test {test_case.name}: {test_case.actual_result.value} "
                        f"({test_case.duration:.2f}s)")
        return test_case.actual_result
    
    def run_all_tests(self, categories: Optional[List[str]] = None) -> Dict[str, int]:
        """Run all test cases, optionally filtered by category"""
        if not self.setup_hardware_connections():
            self.logger.error("Failed to setup hardware connections")
            return {"PASS": 0, "FAIL": 0, "SKIP": 0, "ERROR": 1}
        
        try:
            results = {"PASS": 0, "FAIL": 0, "SKIP": 0, "ERROR": 0}
            
            filtered_tests = self.test_cases
            if categories:
                filtered_tests = [t for t in self.test_cases if t.category in categories]
            
            self.logger.info(f"Running {len(filtered_tests)} tests...")
            
            for test_case in filtered_tests:
                result = self.run_test_case(test_case)
                results[result.value] += 1
                self.results[test_case.name] = result
            
            self._generate_report(results)
            return results
            
        finally:
            self.cleanup_connections()
    
    def _generate_report(self, results: Dict[str, int]):
        """Generate test report"""
        total_tests = sum(results.values())
        pass_rate = (results["PASS"] / total_tests * 100) if total_tests > 0 else 0
        
        report = f"""
=== PAPYRUS FIRMWARE TEST REPORT ===
Total Tests: {total_tests}
Passed: {results["PASS"]}
Failed: {results["FAIL"]}
Skipped: {results["SKIP"]}
Errors: {results["ERROR"]}
Pass Rate: {pass_rate:.1f}%

Detailed Results:
"""
        
        for test_case in self.test_cases:
            status_symbol = {
                TestResult.PASS: "✓",
                TestResult.FAIL: "✗",
                TestResult.SKIP: "○",
                TestResult.ERROR: "!"
            }[test_case.actual_result]
            
            report += f"{status_symbol} {test_case.name} ({test_case.duration:.2f}s)"
            if test_case.error_message:
                report += f" - {test_case.error_message}"
            report += "\n"
        
        print(report)
        
        # Save detailed report
        with open("test_report.txt", "w") as f:
            f.write(report)
        
        # Save JSON report for automation
        json_report = {
            "summary": results,
            "pass_rate": pass_rate,
            "tests": [
                {
                    "name": tc.name,
                    "result": tc.actual_result.value,
                    "duration": tc.duration,
                    "error": tc.error_message
                } for tc in self.test_cases
            ]
        }
        
        with open("test_report.json", "w") as f:
            json.dump(json_report, f, indent=2)

# Specific test implementations
class PapyrusUnitTests:
    """Unit tests for individual board functionality"""
    
    def __init__(self, framework: PapyrusTestFramework):
        self.framework = framework
        self._register_tests()
    
    def _register_tests(self):
        """Register all unit tests"""
        
        # Main Board Tests
        self.framework.add_test_case(TestCase(
            name="main_board_startup",
            description="Verify main board starts up correctly",
            category="unit",
            timeout=10.0,
            test_func=self.test_main_board_startup
        ))
        
        self.framework.add_test_case(TestCase(
            name="main_board_can_init",
            description="Verify CAN bus initialization",
            category="unit",
            timeout=5.0,
            test_func=self.test_main_board_can_init
        ))
        
        # Servo Controller Tests
        self.framework.add_test_case(TestCase(
            name="servo_controller_startup",
            description="Verify servo controller starts up",
            category="unit",
            timeout=10.0,
            test_func=self.test_servo_controller_startup
        ))
        
        self.framework.add_test_case(TestCase(
            name="servo_position_control",
            description="Test servo position control",
            category="unit",
            timeout=15.0,
            test_func=self.test_servo_position_control
        ))
        
        # Thermocouple Controller Tests
        self.framework.add_test_case(TestCase(
            name="tc_controller_startup",
            description="Verify TC controller starts up",
            category="unit",
            timeout=10.0,
            test_func=self.test_tc_controller_startup
        ))
        
        self.framework.add_test_case(TestCase(
            name="tc_temperature_reading",
            description="Test temperature reading accuracy",
            category="unit",
            timeout=10.0,
            test_func=self.test_tc_temperature_reading
        ))
    
    def test_main_board_startup(self) -> TestResult:
        """Test main board startup sequence"""
        response = self.framework.send_command("main_board", "STATUS")
        assert "MAIN_BOARD_READY" in response, f"Unexpected response: {response}"
        return TestResult.PASS
    
    def test_main_board_can_init(self) -> TestResult:
        """Test CAN bus initialization"""
        response = self.framework.send_command("main_board", "CAN_STATUS")
        assert "CAN_ACTIVE" in response, f"CAN not active: {response}"
        return TestResult.PASS
    
    def test_servo_controller_startup(self) -> TestResult:
        """Test servo controller startup"""
        response = self.framework.send_command("servo_controller", "STATUS")
        assert "SERVO_CTRL_READY" in response, f"Servo controller not ready: {response}"
        return TestResult.PASS
    
    def test_servo_position_control(self) -> TestResult:
        """Test servo position control functionality"""
        test_positions = self.framework.config["test_data"]["servo_positions"]
        
        for position in test_positions:
            # Send position command
            cmd = f"SERVO_SET_POS 0 {position}"
            self.framework.send_command("servo_controller", cmd)
            
            # Wait for movement
            time.sleep(2.0)
            
            # Verify position
            response = self.framework.send_command("servo_controller", "SERVO_GET_POS 0")
            actual_pos = int(response.split()[-1])
            
            assert abs(actual_pos - position) <= 10, \
                f"Position error: expected {position}, got {actual_pos}"
        
        return TestResult.PASS
    
    def test_tc_controller_startup(self) -> TestResult:
        """Test thermocouple controller startup"""
        response = self.framework.send_command("tc_controller", "STATUS")
        assert "TC_CTRL_READY" in response, f"TC controller not ready: {response}"
        return TestResult.PASS
    
    def test_tc_temperature_reading(self) -> TestResult:
        """Test temperature reading functionality"""
        response = self.framework.send_command("tc_controller", "READ_TEMP 0")
        
        # Parse temperature value
        temp_str = response.split()[-1]
        temperature = float(temp_str)
        
        # Verify reasonable temperature range
        assert -100 <= temperature <= 1500, \
            f"Temperature out of range: {temperature}°C"
        
        return TestResult.PASS

class PapyrusIntegrationTests:
    """Integration tests for multi-board functionality"""
    
    def __init__(self, framework: PapyrusTestFramework):
        self.framework = framework
        self._register_tests()
    
    def _register_tests(self):
        """Register integration tests"""
        
        self.framework.add_test_case(TestCase(
            name="can_bus_communication",
            description="Test CAN bus communication between boards",
            category="integration",
            timeout=30.0,
            test_func=self.test_can_bus_communication
        ))
        
        self.framework.add_test_case(TestCase(
            name="emergency_stop_propagation",
            description="Test emergency stop propagation",
            category="integration",
            timeout=15.0,
            test_func=self.test_emergency_stop_propagation
        ))
        
        self.framework.add_test_case(TestCase(
            name="ground_station_communication",
            description="Test ground station radio communication",
            category="integration",
            timeout=20.0,
            test_func=self.test_ground_station_communication
        ))
    
    def test_can_bus_communication(self) -> TestResult:
        """Test CAN bus communication between boards"""
        # Send message from main board to servo controller
        self.framework.send_command("main_board", "CAN_SEND_TEST_MSG")
        
        # Check if servo controller received the message
        time.sleep(1.0)
        response = self.framework.send_command("servo_controller", "CAN_GET_LAST_MSG")
        
        assert "TEST_MSG" in response, f"Test message not received: {response}"
        return TestResult.PASS
    
    def test_emergency_stop_propagation(self) -> TestResult:
        """Test emergency stop signal propagation"""
        # Trigger emergency stop from main board
        self.framework.send_command("main_board", "EMERGENCY_STOP")
        
        # Check all controllers enter emergency state
        time.sleep(2.0)
        
        boards = ["servo_controller", "tc_controller"]
        for board in boards:
            response = self.framework.send_command(board, "STATUS")
            assert "EMERGENCY_STOP" in response, \
                f"Board {board} not in emergency state: {response}"
        
        # Clear emergency stop
        self.framework.send_command("main_board", "EMERGENCY_CLEAR")
        time.sleep(2.0)
        
        return TestResult.PASS
    
    def test_ground_station_communication(self) -> TestResult:
        """Test ground station radio communication"""
        # Send command through ground station
        gs_socket = self.framework.socket_connections["ground_station"]
        
        command = {
            "type": "servo_command",
            "board_id": 1,
            "servo_id": 0,
            "position": 2048
        }
        
        gs_socket.send(json.dumps(command).encode())
        
        # Verify command execution
        time.sleep(3.0)
        response = self.framework.send_command("servo_controller", "SERVO_GET_POS 0")
        actual_pos = int(response.split()[-1])
        
        assert abs(actual_pos - 2048) <= 20, \
            f"Ground station command failed: expected 2048, got {actual_pos}"
        
        return TestResult.PASS

def main():
    """Main test execution function"""
    parser = argparse.ArgumentParser(description="Papyrus Firmware Test Framework")
    parser.add_argument("--config", default="test_config.json",
                       help="Test configuration file")
    parser.add_argument("--categories", nargs="+", 
                       choices=["unit", "integration", "hardware"],
                       help="Test categories to run")
    parser.add_argument("--verbose", "-v", action="store_true",
                       help="Verbose output")
    
    args = parser.parse_args()
    
    # Initialize test framework
    framework = PapyrusTestFramework(args.config)
    
    if args.verbose:
        framework.logger.setLevel(logging.DEBUG)
    
    # Register test suites
    unit_tests = PapyrusUnitTests(framework)
    integration_tests = PapyrusIntegrationTests(framework)
    
    # Run tests
    results = framework.run_all_tests(args.categories)
    
    # Exit with appropriate code
    exit_code = 0 if results["FAIL"] == 0 and results["ERROR"] == 0 else 1
    sys.exit(exit_code)

if __name__ == "__main__":
    main() 