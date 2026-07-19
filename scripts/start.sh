#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

gui=false
mode=core
while (( $# > 0 )); do
  case "$1" in
    --gui)
      gui=true
      ;;
    --multi|--multi-agent)
      [[ "${mode}" == core ]] || {
        echo "Only one multi-agent mode flag may be supplied." >&2
        exit 2
      }
      mode=multi
      ;;
    --help|-h)
      echo "Usage: $0 [--gui] [--multi-agent]"
      exit 0
      ;;
    *)
      echo "Usage: $0 [--gui] [--multi-agent]" >&2
      exit 2
      ;;
  esac
  shift
done

mkdir -p \
  "${root}/runtime/artifacts" \
  "${root}/runtime/ros_logs"
compose=(docker compose --project-directory "${root}" -f "${root}/compose.yaml")

export OMOKAI_MODE="${mode}"
if [[ "${mode}" == multi ]]; then
  export OMOKAI_LAUNCH_FILE=multi_robot_simulation.launch.py
  export OMOKAI_RVIZ=false
  export OMOKAI_MAP_FILE=
else
  export OMOKAI_LAUNCH_FILE=core_navigation.launch.py
  export OMOKAI_RVIZ=false
  export OMOKAI_MAP_FILE=
fi

if [[ "${gui}" == true ]]; then
  [[ -n "${DISPLAY:-}" ]] || {
    echo "DISPLAY is not set." >&2
    exit 1
  }
  export XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}"
  [[ -f "${XAUTHORITY}" ]] || {
    echo "XAUTHORITY does not reference a file: ${XAUTHORITY}" >&2
    exit 1
  }
  compose+=(-f "${root}/compose.gui.yaml")
fi

"${compose[@]}" up --detach --build --force-recreate robot

for _ in {1..36}; do
  health="$(docker inspect --format '{{.State.Health.Status}}' omokai-task1 2>/dev/null || true)"
  case "${health}" in
    healthy)
      echo "Robot simulation is ready."
      exit 0
      ;;
    unhealthy)
      "${compose[@]}" logs --tail=120 robot >&2
      exit 1
      ;;
  esac
  sleep 5
done

echo "Timed out waiting for the robot simulation." >&2
"${compose[@]}" logs --tail=120 robot >&2
exit 1
