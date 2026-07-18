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
    --slam)
      mode=slam
      ;;
    --help|-h)
      echo "Usage: $0 [--gui] [--slam]"
      exit 0
      ;;
    *)
      echo "Usage: $0 [--gui] [--slam]" >&2
      exit 2
      ;;
  esac
  shift
done

mkdir -p \
  "${root}/runtime/artifacts" \
  "${root}/runtime/maps" \
  "${root}/runtime/ros_logs"
compose=(docker compose --project-directory "${root}" -f "${root}/compose.yaml")

export OMOKAI_MODE="${mode}"
if [[ "${mode}" == slam ]]; then
  export OMOKAI_LAUNCH_FILE=slam_navigation.launch.py
else
  export OMOKAI_LAUNCH_FILE=core_navigation.launch.py
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
