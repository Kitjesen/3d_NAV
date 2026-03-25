#!/bin/bash
# thunder 服务管理脚本

SERVICE_NAME="thunder"
SERVICE_FILE="thunder.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="/etc/systemd/system"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_info() {
    echo "ℹ️  $1"
}

# 检查是否为 root
check_root() {
    if [ "$EUID" -ne 0 ]; then
        print_error "需要 root 权限，请使用 sudo 运行"
        exit 1
    fi
}

# 安装服务
install_service() {
    check_root

    print_info "安装 thunder 服务..."

    if [ ! -f "$SCRIPT_DIR/$SERVICE_FILE" ]; then
        print_error "找不到服务文件: $SERVICE_FILE"
        exit 1
    fi

    # 复制服务文件
    cp "$SCRIPT_DIR/$SERVICE_FILE" "$SYSTEMD_DIR/$SERVICE_FILE"
    print_success "服务文件已复制到 $SYSTEMD_DIR"

    # 重载 systemd
    systemctl daemon-reload
    print_success "systemd 已重载"

    # 启用服务
    systemctl enable $SERVICE_NAME
    print_success "服务已设置为开机自启"

    print_success "thunder 服务安装完成！"
    echo ""
    print_info "使用以下命令管理服务:"
    echo "  启动: sudo systemctl start $SERVICE_NAME"
    echo "  停止: sudo systemctl stop $SERVICE_NAME"
    echo "  状态: sudo systemctl status $SERVICE_NAME"
    echo "  日志: sudo journalctl -u $SERVICE_NAME -f"
}

# 卸载服务
uninstall_service() {
    check_root

    print_info "卸载 thunder 服务..."

    # 停止服务
    systemctl stop $SERVICE_NAME 2>/dev/null

    # 禁用服务
    systemctl disable $SERVICE_NAME 2>/dev/null

    # 删除服务文件
    rm -f "$SYSTEMD_DIR/$SERVICE_FILE"

    # 重载 systemd
    systemctl daemon-reload

    print_success "thunder 服务已卸载"
}

# 启动服务
start_service() {
    check_root
    print_info "启动 thunder 服务..."
    systemctl start $SERVICE_NAME

    if [ $? -eq 0 ]; then
        print_success "服务已启动"
        sleep 1
        systemctl status $SERVICE_NAME --no-pager
    else
        print_error "服务启动失败"
        exit 1
    fi
}

# 停止服务
stop_service() {
    check_root
    print_info "停止 thunder 服务..."
    systemctl stop $SERVICE_NAME

    if [ $? -eq 0 ]; then
        print_success "服务已停止"
    else
        print_error "服务停止失败"
        exit 1
    fi
}

# 重启服务
restart_service() {
    check_root
    print_info "重启 thunder 服务..."
    systemctl restart $SERVICE_NAME

    if [ $? -eq 0 ]; then
        print_success "服务已重启"
        sleep 1
        systemctl status $SERVICE_NAME --no-pager
    else
        print_error "服务重启失败"
        exit 1
    fi
}

# 查看状态
status_service() {
    systemctl status $SERVICE_NAME --no-pager
}

# 查看日志
logs_service() {
    print_info "查看 thunder 日志 (Ctrl+C 退出)..."
    journalctl -u $SERVICE_NAME -f
}

# 显示帮助
show_help() {
    echo "thunder 服务管理脚本"
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  install    - 安装服务并设置开机自启"
    echo "  uninstall  - 卸载服务"
    echo "  start      - 启动服务"
    echo "  stop       - 停止服务"
    echo "  restart    - 重启服务"
    echo "  status     - 查看服务状态"
    echo "  logs       - 查看实时日志"
    echo "  help       - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  sudo $0 install    # 安装服务"
    echo "  sudo $0 start      # 启动服务"
    echo "  $0 status          # 查看状态（不需要 sudo）"
    echo "  sudo $0 logs       # 查看日志"
}

# 主函数
main() {
    case "$1" in
        install)
            install_service
            ;;
        uninstall)
            uninstall_service
            ;;
        start)
            start_service
            ;;
        stop)
            stop_service
            ;;
        restart)
            restart_service
            ;;
        status)
            status_service
            ;;
        logs)
            logs_service
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "未知命令: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@"
