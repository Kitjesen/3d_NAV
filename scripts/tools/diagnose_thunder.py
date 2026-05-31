#!/usr/bin/env python3
"""
thunder robot diagnostic tool — automatically checks common configuration issues.
"""

import sys
import subprocess
import os


class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'


def print_success(msg):
    print(f"{Colors.GREEN}[OK]   {msg}{Colors.END}")


def print_error(msg):
    print(f"{Colors.RED}[ERR]  {msg}{Colors.END}")


def print_warning(msg):
    print(f"{Colors.YELLOW}[WARN] {msg}{Colors.END}")


def print_info(msg):
    print(f"{Colors.BLUE}[INFO] {msg}{Colors.END}")


def print_header(msg):
    print(f"\n{'='*60}")
    print(f"{Colors.BLUE}{msg}{Colors.END}")
    print('='*60)


def check_python():
    """Check Python version (requires 3.8+)."""
    print_header("Python Environment")

    version = sys.version_info
    version_str = f"{version.major}.{version.minor}.{version.micro}"

    if version.major >= 3 and version.minor >= 8:
        print_success(f"Python version: {version_str}")
        return True
    else:
        print_error(f"Python version too low: {version_str} (requires 3.8+)")
        return False


def check_pip():
    """Check pip3 availability."""
    try:
        result = subprocess.run(['pip3', '--version'],
                                capture_output=True, text=True)
        if result.returncode == 0:
            print_success(f"pip3 installed: {result.stdout.strip()}")
            return True
        else:
            print_error("pip3 not installed")
            return False
    except FileNotFoundError:
        print_error("pip3 not found")
        return False


def check_lark_oapi():
    """Check Feishu SDK (lark-oapi)."""
    print_header("Feishu SDK")

    try:
        import lark_oapi
        version = getattr(lark_oapi, '__version__', 'unknown')
        print_success(f"lark-oapi installed (version: {version})")
        return True
    except ImportError:
        print_error("lark-oapi not installed")
        print_info("Install with: pip3 install lark-oapi")
        return False


def check_ros2():
    """Check ROS2 environment."""
    print_header("ROS2 Environment")

    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print_success(f"ROS2 environment active: {ros_distro}")

        try:
            import rclpy
            print_success("rclpy installed")
            return True
        except ImportError:
            print_error("rclpy not installed")
            return False
    else:
        print_warning("ROS2 environment not active")
        print_info("Activate with: source /opt/ros/humble/setup.bash")
        return False


def check_ros2_topic():
    """Check that /nav/semantic/status topic is active."""
    if not os.environ.get('ROS_DISTRO'):
        print_warning("Skipping topic check (ROS2 not active)")
        return False

    try:
        result = subprocess.run(['ros2', 'topic', 'list'],
                                capture_output=True, text=True, timeout=5)

        if '/nav/semantic/status' in result.stdout:
            print_success("Topic /nav/semantic/status is active")
            return True
        else:
            print_warning("Topic /nav/semantic/status not found")
            print_info("Make sure the navigation node is running")
            return False
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print_warning("Cannot check ROS2 topics")
        return False


def check_config_file():
    """Check feishu_monitor_bot.py configuration."""
    print_header("Configuration File")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    bot_file = os.path.join(script_dir, 'feishu_monitor_bot.py')

    if not os.path.exists(bot_file):
        print_error("feishu_monitor_bot.py not found")
        return False

    print_success("feishu_monitor_bot.py exists")

    with open(bot_file, 'r', encoding='utf-8') as f:
        content = f.read()

        if 'cli_xxxxxxxxxxxxxxxx' in content:
            print_warning("APP_ID is still the placeholder value — update it")
            print_info("Edit feishu_monitor_bot.py line 149")
            return False

        if 'ou_xxxxxxxxxxxxxxxx' in content:
            print_warning("RECEIVE_ID is still the placeholder value — update it")
            print_info("Edit feishu_monitor_bot.py line 151")
            return False

    print_success("Configuration has been updated")
    return True


def check_scripts():
    """Check that required script files exist."""
    print_header("Script Files")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    scripts = [
        'feishu_monitor_bot.py',
        'test_feishu.py',
        'install_feishu_bot.sh',
        'start_thunder.sh'
    ]

    all_exist = True
    for script in scripts:
        path = os.path.join(script_dir, script)
        if os.path.exists(path):
            print_success(f"{script} exists")
        else:
            print_error(f"{script} not found")
            all_exist = False

    return all_exist


def check_permissions():
    """Check execute permissions on shell scripts."""
    print_header("Execute Permissions")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    scripts = [
        'install_feishu_bot.sh',
        'start_thunder.sh',
        'thunder_service.sh'
    ]

    all_executable = True
    for script in scripts:
        path = os.path.join(script_dir, script)
        if os.path.exists(path):
            if os.access(path, os.X_OK):
                print_success(f"{script} is executable")
            else:
                print_warning(f"{script} is not executable")
                print_info(f"Fix with: chmod +x {path}")
                all_executable = False

    return all_executable


def main():
    print(f"\n{Colors.BLUE}{'='*60}")
    print("thunder Robot Diagnostic Tool")
    print(f"{'='*60}{Colors.END}\n")

    results = []

    results.append(("Python",       check_python()))
    results.append(("pip3",         check_pip()))
    results.append(("lark-oapi",    check_lark_oapi()))
    results.append(("ROS2",         check_ros2()))
    results.append(("ROS2 topic",   check_ros2_topic()))
    results.append(("Config file",  check_config_file()))
    results.append(("Script files", check_scripts()))
    results.append(("Permissions",  check_permissions()))

    print_header("Diagnostic Summary")

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"  {name:20s} {status}")

    print(f"\n  Passed: {passed}/{total}")

    if passed == total:
        print_success("\nAll checks passed — thunder robot is ready to run")
        print_info("Start with: ./start_thunder.sh")
    else:
        print_warning(f"\n{total - passed} check(s) failed — fix the issues above")

    print(f"\n{'='*60}\n")
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
