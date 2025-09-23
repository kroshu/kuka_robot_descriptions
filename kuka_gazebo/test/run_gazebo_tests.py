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

# List of models and their support packages
tests = [
    ("lbr_iisy3_r760", "kuka_lbr_iisy_support"),
    ("lbr_iisy11_r1300", "kuka_lbr_iisy_support"),
    ("lbr_iisy15_r930", "kuka_lbr_iisy_support"),
    ("kr10_r1100_2", "kuka_agilus_support"),
    ("kr16_r2010_2", "kuka_cybertech_support"),
    ("kr70_r2100", "kuka_iontec_support"),
    ("kr210_r2700_2", "kuka_quantec_support"),
    ("kr210_r3100_2", "kuka_quantec_support"),
    ("kr240_r3330", "kuka_fortec_support"),
    ("kr560_r3100_2", "kuka_fortec_support"),
]

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

print("\nDetailed error log:")
for model, line in problems:
    if prev_robot == model:
        print(f"\t{line}")
    else:
        print(f"Robot: {model}")
        print(f"\t{line}")
        prev_robot = model


 
