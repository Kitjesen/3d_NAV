#!/usr/bin/env python3
"""
thunder 机器人诊断工具
自动检测常见配置问题
"""

import sys
import subprocess
import os

# 颜色输出
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'

def print_success(msg):
    print(f"{Colors.GREEN}✅ {msg}{Colors.END}")

def print_error(msg):
    print(f"{Colors.RED}❌ {msg}{Colors.END}")

def print_warning(msg):
    print(f"{Colors.YELLOW}⚠️  {msg}{Colors.END}")

def print_info(msg):
    print(f"{Colors.BLUE}ℹ️  {msg}{Colors.END}")

def print_header(msg):
    print(f"\n{'='*60}")
    print(f"{Colors.BLUE}{msg}{Colors.END}")
    print('='*60)

def check_python():
    """检查 Python 版本"""
    print_header("检查 Python 环境")

    version = sys.version_info
    version_str = f"{version.major}.{version.minor}.{version.micro}"

    if version.major >= 3 and version.minor >= 8:
        print_success(f"Python 版本: {version_str}")
        return True
    else:
        print_error(f"Python 版本过低: {version_str} (需要 3.8+)")
        return False

def check_pip():
    """检查 pip"""
    try:
        result = subprocess.run(['pip3', '--version'],
                              capture_output=True, text=True)
        if result.returncode == 0:
            print_success(f"pip3 已安装: {result.stdout.strip()}")
            return True
        else:
            print_error("pip3 未安装")
            return False
    except FileNotFoundError:
        print_error("pip3 未找到")
        return False

def check_lark_oapi():
    """检查 lark-oapi"""
    print_header("检查飞书 SDK")

    try:
        import lark_oapi
        version = getattr(lark_oapi, '__version__', 'unknown')
        print_success(f"lark-oapi 已安装 (版本: {version})")
        return True
    except ImportError:
        print_error("lark-oapi 未安装")
        print_info("安装命令: pip3 install lark-oapi")
        return False

def check_ros2():
    """检查 ROS2 环境"""
    print_header("检查 ROS2 环境")

    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print_success(f"ROS2 环境已激活: {ros_distro}")

        # 检查 rclpy
        try:
            import rclpy
            print_success("rclpy 已安装")
            return True
        except ImportError:
            print_error("rclpy 未安装")
            return False
    else:
        print_warning("ROS2 环境未激活")
        print_info("激活命令: source /opt/ros/humble/setup.bash")
        return False

def check_ros2_topic():
    """检查 ROS2 话题"""
    if not os.environ.get('ROS_DISTRO'):
        print_warning("跳过话题检查（ROS2 未激活）")
        return False

    try:
        result = subprocess.run(['ros2', 'topic', 'list'],
                              capture_output=True, text=True, timeout=5)

        if '/nav/semantic/status' in result.stdout:
            print_success("话题 /nav/semantic/status 存在")
            return True
        else:
            print_warning("话题 /nav/semantic/status 不存在")
            print_info("请确保导航节点正在运行")
            return False
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print_warning("无法检查 ROS2 话题")
        return False

def check_config_file():
    """检查配置文件"""
    print_header("检查配置文件")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    bot_file = os.path.join(script_dir, 'feishu_monitor_bot.py')

    if not os.path.exists(bot_file):
        print_error(f"找不到 feishu_monitor_bot.py")
        return False

    print_success("feishu_monitor_bot.py 存在")

    # 检查配置是否为默认值
    with open(bot_file, 'r', encoding='utf-8') as f:
        content = f.read()

        if 'cli_xxxxxxxxxxxxxxxx' in content:
            print_warning("APP_ID 仍为默认值，需要修改")
            print_info("编辑 feishu_monitor_bot.py 第 149 行")
            return False

        if 'ou_xxxxxxxxxxxxxxxx' in content:
            print_warning("RECEIVE_ID 仍为默认值，需要修改")
            print_info("编辑 feishu_monitor_bot.py 第 151 行")
            return False

    print_success("配置文件已修改")
    return True

def check_scripts():
    """检查脚本文件"""
    print_header("检查脚本文件")

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
            print_success(f"{script} 存在")
        else:
            print_error(f"{script} 不存在")
            all_exist = False

    return all_exist

def check_permissions():
    """检查脚本执行权限"""
    print_header("检查执行权限")

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
                print_success(f"{script} 可执行")
            else:
                print_warning(f"{script} 不可执行")
                print_info(f"修复命令: chmod +x {path}")
                all_executable = False

    return all_executable

def main():
    """主函数"""
    print(f"\n{Colors.BLUE}{'='*60}")
    print("thunder 机器人诊断工具")
    print(f"{'='*60}{Colors.END}\n")

    results = []

    # 运行所有检查
    results.append(("Python 环境", check_python()))
    results.append(("pip3", check_pip()))
    results.append(("lark-oapi", check_lark_oapi()))
    results.append(("ROS2 环境", check_ros2()))
    results.append(("ROS2 话题", check_ros2_topic()))
    results.append(("配置文件", check_config_file()))
    results.append(("脚本文件", check_scripts()))
    results.append(("执行权限", check_permissions()))

    # 总结
    print_header("诊断总结")

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{name:20s} {status}")

    print(f"\n通过: {passed}/{total}")

    if passed == total:
        print_success("\n所有检查通过！thunder 机器人可以运行")
        print_info("\n启动命令: ./start_thunder.sh")
    else:
        print_warning(f"\n有 {total - passed} 项检查未通过")
        print_info("请根据上述提示修复问题")

    print(f"\n{'='*60}\n")

    return 0 if passed == total else 1

if __name__ == '__main__':
    sys.exit(main())
