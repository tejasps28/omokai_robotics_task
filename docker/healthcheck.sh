#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/omokai_ws/install/setup.bash

topics="$(timeout 8 ros2 topic list 2>/dev/null)"
for topic in /clock /odom /scan /tf; do
  [[ "${topics}" == *"${topic}"* ]] || {
    echo "Missing required topic: ${topic}" >&2
    exit 1
  }
done

amcl_state="$(timeout 4 ros2 lifecycle get /amcl 2>/dev/null)"
navigator_state="$(timeout 4 ros2 lifecycle get /bt_navigator 2>/dev/null)"
actions="$(timeout 4 ros2 action list 2>/dev/null)"

[[ "${amcl_state}" == active* ]] || exit 1
[[ "${navigator_state}" == active* ]] || exit 1
[[ "${actions}" == *"/navigate_to_pose"* ]] || exit 1
timeout 4 ros2 topic echo /amcl_pose --once >/dev/null 2>&1
