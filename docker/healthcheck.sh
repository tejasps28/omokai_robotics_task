#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/omokai_ws/install/setup.bash

topics="$(timeout 8 ros2 topic list 2>/dev/null)"
for topic in /clock; do
  [[ "${topics}" == *"${topic}"* ]] || {
    echo "Missing required topic: ${topic}" >&2
    exit 1
  }
done

if [[ "${OMOKAI_MODE:-core}" == vision ]]; then
  for topic in \
    /robot1/camera/image \
    /robot1/camera/depth_image \
    /robot1/camera/camera_info \
    /robot1/camera/points \
    /robot1/camera/depth_visualization \
    /vision/detections \
    /vision/detections/image \
    /vision/target_detection \
    /vision/selection_status \
    /vision/follow_status; do
    [[ "${topics}" == *"${topic}"* ]] || {
      echo "Missing required vision topic: ${topic}" >&2
      exit 1
    }
    timeout 5 ros2 topic echo "${topic}" --once >/dev/null 2>&1
  done
  exit 0
fi

for topic in /odom /scan /tf; do
  [[ "${topics}" == *"${topic}"* ]] || {
    echo "Missing required topic: ${topic}" >&2
    exit 1
  }
done

navigator_state="$(timeout 4 ros2 lifecycle get /bt_navigator 2>/dev/null)"
actions="$(timeout 4 ros2 action list 2>/dev/null)"

[[ "${navigator_state}" == active* ]] || exit 1
[[ "${actions}" == *"/navigate_to_pose"* ]] || exit 1

amcl_state="$(timeout 4 ros2 lifecycle get /amcl 2>/dev/null)"
[[ "${amcl_state}" == active* ]] || exit 1
timeout 4 ros2 topic echo /amcl_pose --once >/dev/null 2>&1
