#!/usr/bin/env python3

import subprocess
import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory

workspace_path = get_package_prefix("kuka_gazebo")
test_path = os.path.join(workspace_path, "lib", "kuka_gazebo")

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


# Shutting down gazebo
def kill_gazebo_gui(*args, **kwargs):
    try:
        result = subprocess.run(["ps", "aux"], stdout=subprocess.PIPE, text=True)
        found = False
        for line in result.stdout.splitlines():
            if "gz sim gui" in line or "gz sim server" in line:
                pid = line.split()[1]
                print(f"Killing Gazebo process with PID: {pid}")
                subprocess.run(["kill", "-9", pid])
                found = True
        if not found:
            print("No matching Gazebo process found.")
    except Exception as e:
        print(f"Error occurred: {e}")


# Getting the list of robots from README.md
def get_robots():
    supported_robots = []
    intable = False
    scan_complete = False

    target_dir = get_package_share_directory("kuka_gazebo")

    # Construct the full path to README.md
    file_path = os.path.join(target_dir, "README.md")

    try:
        with open(file_path) as readme:
            for line in readme:
                line = line.strip()
                if not intable:
                    if "Supported features" in line:
                        intable = True
                else:
                    if line.startswith("|"):
                        parts = [part.strip() for part in line.strip("|").split("|")]
                        if len(parts) >= 5 and parts[4] == "✓":
                            robot_name = parts[0]
                            robot_family = parts[1]
                            robot_family_support_read = "kuka_" + robot_family + "_support"
                            supported_robots.append((robot_name, robot_family_support_read))
                            scan_complete = True
                    elif scan_complete and intable:
                        intable = False
        return supported_robots
    except FileNotFoundError:
        return f"Error: The file '{file_path}' was not found."
    except OSError as e:
        return f"Error reading file: {e}"


tests = get_robots()
print(tests)
summary = []
problems = []

# Testing the supported robots
for model, support in tests:
    print(f"Running test for model: {model}, support: {support}")
    test_file = os.path.join(test_path, "gazebo_support_test.py")

    if not os.path.exists(test_file):
        print(f"Test file '{test_file}' does not exist\n")
        continue

    result = subprocess.run(
        [
            "python3",
            "-m",
            "launch_testing.launch_test",
            test_file,
            f"robot_model:={model}",
            f"robot_family_support:={support}",
        ],
        capture_output=True,
        text=True,
    )

    print("--- STDOUT ---")
    print(result.stdout)

    for line in result.stdout.splitlines():
        if "[Err]" in line:
            print(line)
            problems.append((model, line))

    print("--- STDERR ---")
    if (
        "test_robot_initialization "
        "(gazebo_support_test.TestDuringLaunch.test_robot_initialization) ... ok" in result.stderr
    ):
        summary.append((model, support, "PASS"))
    else:
        summary.append((model, support, "FAIL"))
    print(result.stderr)
    kill_gazebo_gui()
    print("--- END OF TEST ---\n")


# Print summary
file_path = os.path.join(get_package_share_directory("kuka_gazebo"), "gazebo_test.txt")

with open(file_path, "w") as gazebo_test:
    gazebo_test.write("\nTest Summary:\n")
    for model, support, status in summary:
        color = GREEN if status == "PASS" else RED
        line = f"Model: {model}, Support: {support}, Status: {status}\n"
        gazebo_test.write(line)
        print(f"{color}{line.strip()}{RESET}")

    prev_robot = None
    if not problems:
        gazebo_test.write("\nNo errors in supported robots\n")
        print("\nNo errors in supported robots")
    else:
        gazebo_test.write("\nDetailed error log:\n")
        print("\nDetailed error log:")
        for model, line in problems:
            if prev_robot == model:
                gazebo_test.write(f"\t{line}\n")
                print(f"\t{line}")
            else:
                gazebo_test.write(f"Robot: {model}\n")
                gazebo_test.write(f"\t{line}\n")
                print(f"Robot: {model}")
                print(f"\t{line}")
            prev_robot = model
