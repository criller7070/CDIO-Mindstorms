#!/usr/bin/env python3
"""
Unit Tests for Navigation Command Parsing
Tests that commands are parsed correctly and execute without errors
"""

def test_command_parsing():
    """Test COMMAND:parameter parsing"""
    test_cases = [
        ("FORWARD:1000", "FORWARD", 1000),
        ("TURN:90", "TURN", 90),
        ("TURN:-45", "TURN", -45),
        ("REVERSE:500", "REVERSE", 500),
        ("SPEED:200", "SPEED", 200),
        ("# This is a comment", None, None),
        ("", None, None),
        ("STOP", "STOP", 0),
    ]
    
    print("=" * 60)
    print("TEST: Command Parsing")
    print("=" * 60)
    
    passed = 0
    failed = 0
    
    for test_input, expected_cmd, expected_val in test_cases:
        try:
            command_str = test_input.strip().upper()
            
            # Skip empty lines and comments
            if not command_str or command_str.startswith("#"):
                result_cmd = None
                result_val = None
            else:
                # Parse command format: "TYPE:parameter"
                if ":" in command_str:
                    cmd, value_str = command_str.split(":", 1)
                    result_cmd = cmd.strip()
                    try:
                        result_val = int(value_str.strip())
                    except ValueError:
                        result_val = None
                else:
                    result_cmd = command_str.strip()
                    result_val = 0
            
            # Check results
            if result_cmd == expected_cmd and result_val == expected_val:
                print("[PASS] '{}' → ({}, {})".format(test_input, result_cmd, result_val))
                passed += 1
            else:
                print("[FAIL] '{}' → ({}, {}) EXPECTED ({}, {})".format(
                    test_input, result_cmd, result_val, expected_cmd, expected_val))
                failed += 1
        
        except Exception as e:
            print("[ERROR] '{}' → {}".format(test_input, str(e)))
            failed += 1
    
    print("\n" + "=" * 60)
    print("RESULTS: {} PASSED, {} FAILED".format(passed, failed))
    print("=" * 60)
    
    return failed == 0


def test_mission_file_loading():
    """Test loading commands from file"""
    print("\n" + "=" * 60)
    print("TEST: Mission File Loading")
    print("=" * 60)
    
    try:
        # Try to load commands.txt
        commands = []
        
        try:
            with open("commands.txt", "r") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#"):
                        commands.append(line)
        except OSError:
            print("[FAIL] Could not find commands.txt")
            return False
        
        if not commands:
            print("[FAIL] No commands found in commands.txt")
            return False
        
        print("[PASS] Loaded {} commands from commands.txt:".format(len(commands)))
        for i, cmd in enumerate(commands, 1):
            print("  [{}] {}".format(i, cmd))
        
        # Verify it's a valid mission
        has_forward = any("FORWARD" in cmd for cmd in commands)
        has_turn = any("TURN" in cmd for cmd in commands)
        has_stop = any("STOP" in cmd for cmd in commands)
        
        if has_forward and has_turn and has_stop:
            print("\n[PASS] Mission file contains required command types")
            return True
        else:
            print("\n[FAIL] Mission file missing required commands")
            print("  FORWARD: {}, TURN: {}, STOP: {}".format(has_forward, has_turn, has_stop))
            return False
    
    except Exception as e:
        print("[ERROR] {}".format(str(e)))
        return False


def test_command_validation():
    """Test that invalid commands are rejected"""
    print("\n" + "=" * 60)
    print("TEST: Command Validation")
    print("=" * 60)
    
    invalid_commands = [
        ("FORWARD:abc", "Non-integer value"),
        ("FORWARD:-100", "Negative distance"),
        ("TURN:361", "Angle > 360"),
        ("SPEED:0", "Zero speed"),
        ("UNKNOWN:100", "Unknown command type"),
    ]
    
    passed = 0
    failed = 0
    
    for cmd, description in invalid_commands:
        try:
            command_str = cmd.strip().upper()
            
            if ":" in command_str:
                cmd_type, value_str = command_str.split(":", 1)
                cmd_type = cmd_type.strip()
                
                try:
                    value = int(value_str.strip())
                except ValueError:
                    print("[PASS] '{}' rejected: {}".format(cmd, description))
                    passed += 1
                    continue
                
                # Validate command-specific ranges
                if cmd_type == "FORWARD" and value <= 0:
                    print("[PASS] '{}' rejected: {}".format(cmd, description))
                    passed += 1
                    continue
                elif cmd_type == "SPEED" and value <= 0:
                    print("[PASS] '{}' rejected: {}".format(cmd, description))
                    passed += 1
                    continue
                
            print("[FAIL] '{}' should have been rejected: {}".format(cmd, description))
            failed += 1
        
        except Exception as e:
            print("[ERROR] '{}' → {}".format(cmd, str(e)))
            failed += 1
    
    print("\nRESULTS: {} PASSED, {} FAILED".format(passed, failed))
    return failed == 0


def main():
    """Run all tests"""
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 58 + "║")
    print("║" + "  GolfBot 9000: Navigation Unit Tests".center(58) + "║")
    print("║" + " " * 58 + "║")
    print("╚" + "=" * 58 + "╝")
    print("\n")
    
    results = []
    
    # Run tests
    results.append(("Command Parsing", test_command_parsing()))
    results.append(("Mission File Loading", test_mission_file_loading()))
    results.append(("Command Validation", test_command_validation()))
    
    # Summary
    print("\n" + "╔" + "=" * 58 + "╗")
    print("║" + " TEST SUMMARY ".center(58, "=") + "║")
    
    passed_tests = sum(1 for _, result in results if result)
    total_tests = len(results)
    
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print("║ {}: {}".format(test_name.ljust(40), status).ljust(59) + "║")
    
    print("║" + "=" * 58 + "║")
    print("║ Overall: {}/{} tests passed".format(passed_tests, total_tests).ljust(59) + "║")
    print("╚" + "=" * 58 + "╝\n")
    
    return all(result for _, result in results)


if __name__ == "__main__":
    import sys
    success = main()
    sys.exit(0 if success else 1)
