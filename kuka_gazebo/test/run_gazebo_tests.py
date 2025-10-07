#!/usr/bin/env python3

import subprocess
import os

# Absolute path to your workspace
workspace_path = os.path.expanduser("~/ros2_ws/src/kuka_robot_descriptions/kuka_gazebo/test")

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


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

for model, support in tests:
    print(f"Running test for model: {model}, support: {support}")
    test_file = os.path.join(workspace_path, "gazebo_support_test.py")

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
        "test_robot_initialization (gazebo_support_test.TestDuringLaunch.test_robot_initialization) ... ok"
        in result.stderr
    ):
        summary.append((model, support, "PASS"))
    else:
        summary.append((model, support, "FAIL"))
    print(result.stderr)
    kill_gazebo_gui()
    print("--- END OF TEST ---\n")


# Print summary
log_file_path = os.path.expanduser("~/ros2_ws/src/kuka_robot_descriptions/kuka_gazebo/test")
file_path = os.path.join(log_file_path, "gazebo_test.txt")

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
