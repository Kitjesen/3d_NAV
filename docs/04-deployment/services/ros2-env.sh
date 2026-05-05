#!/bin/bash
# ROS2 environment helper for systemd service wrappers.
# NAV_WS follows the /opt/lingtu/current symlink so OTA updates work automatically.

export NAV_WS=/opt/lingtu/current
source /opt/ros/humble/setup.bash
source "${NAV_WS}/install/setup.bash"
export ROS_DOMAIN_ID=0
