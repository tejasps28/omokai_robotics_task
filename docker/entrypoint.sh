#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/omokai_ws/install/setup.bash

if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
  mkdir -p "${XDG_RUNTIME_DIR}"
  chmod 0700 "${XDG_RUNTIME_DIR}"
fi

exec "$@"
