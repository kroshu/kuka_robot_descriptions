#!/usr/bin/env python3

import subprocess
import os
import re
# Absolute path to your workspace
workspace_path = os.path.expanduser("~/ros2_ws/src/kuka_robot_descriptions/kuka_gazebo/test")

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

def kill_gazebo_gui( *args, **kwargs):
    try:
        result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE, text=True)
        found = False
        for line in result.stdout.splitlines():
            if 'ign gazebo gui' in line or 'ign gazebo server' in line:
                pid = line.split()[1]
                print(f"Killing Gazebo process with PID: {pid}")
                subprocess.run(['kill', '-9', pid])
                found = True
        if not found:
            print("No matching Gazebo process found.")
    except Exception as e:
        print(f"Error occurred: {e}")

def get_robots():
    supported_robots = []
    intable = False
    scan_complete = False

    

    # Get the directory of the current script
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Go four levels up
    root_dir = current_dir
    for _ in range(4):
        root_dir = os.path.dirname(root_dir)

    # Navigate into src/kuka_robot_descriptions
    target_dir = os.path.join(root_dir, "src", "kuka_robot_descriptions")


    # Construct the full path to README.md
    file_path = os.path.join(target_dir, "README.md")

    try:
        with open(file_path, 'r') as readme:
            for line in readme:
                line = line.strip()
                if intable == False:
                    if 'Supported features' in line:
                        intable = True
                else:
                    if line.startswith('|'):
                        parts = [part.strip() for part in line.strip('|').split('|')]
                        if len(parts) >= 5 and parts[4] == '✓':
                            robot_name = parts[0]
                            robot_family = parts[1]
                            robot_family_support_read = 'kuka_' + robot_family + '_support'
                            supported_robots.append((robot_name, robot_family_support_read))
                            
                            scan_complete = True
                    elif scan_complete == True and intable == True:
                        intable = False
        return supported_robots
    except FileNotFoundError:
        return f"Error: The file '{file_path}' was not found."
    except IOError as e:
        return f"Error reading file: {e}"


tests = get_robots()
print(tests)
summary = []
problems = []

for model, support in tests:
    print(f"Running test for model: {model}, support: {support}")
    test_file = os.path.join(workspace_path, "gazebo_support_test.py")

    if not os.path.exists(test_file):
        print(f"❌ Test file '{test_file}' does not exist\n")
        continue

    result = subprocess.run([
        "python3", "-m", "launch_testing.launch_test",
        test_file,
        f"robot_model:={model}",
        f"robot_family_support:={support}"
    ], capture_output=True, text=True)

    print("--- STDOUT ---")
    print(result.stdout)
    
    for line in result.stdout.splitlines():
        if '[Err]' in line:
            print(line)
            problems.append((model, line))

    print("--- STDERR ---")
    print(result.stderr)
    kill_gazebo_gui()
    print("--- END OF TEST ---\n")
    
    # Check for 'OK' in stdout
    if 'test_robot_initialization (gazebo_support_test.TestDuringLaunch) ... FAIL' in result.stderr:
        summary.append((model, support, 'FAIL'))
    else:
        summary.append((model, support, 'PASS'))

# Print summary
print("\nTest Summary:")

for model, support, status in summary:
    color = GREEN if status == 'PASS' else RED
    print(f"{color}Model: {model}, Support: {support}, Status: {status}{RESET}")

prev_robot = 0


if not problems:
    print("\nNo errors in supported robots")
else:
    print("\nDetailed error log:")
    for model, line in problems:
        if prev_robot == model:
            print(f"\t{line}")
        else:
            print(f"Robot: {model}")
            print(f"\t{line}")
            prev_robot = model

