#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

gui=false
actor_motion=auto
red_actor=false
print_config=false
mode=core
map_name=""
while (( $# > 0 )); do
  case "$1" in
    --gui)
      gui=true
      ;;
    --slam)
      [[ "${mode}" == core ]] || {
        echo "--slam, --multi, and --map are mutually exclusive." >&2
        exit 2
      }
      mode=slam
      ;;
    --multi|--multi-agent)
      [[ "${mode}" == core ]] || {
        echo "--slam, --multi, and --map are mutually exclusive." >&2
        exit 2
      }
      mode=multi
      ;;
    --vision|--perception)
      [[ "${mode}" == core ]] || {
        echo "--slam, --multi, --vision, and --map are mutually exclusive." >&2
        exit 2
      }
      mode=vision
      ;;
    --moving-actor)
      actor_motion=true
      ;;
    --stationary-actor)
      actor_motion=false
      ;;
    --red-actor)
      red_actor=true
      ;;
    --print-config)
      print_config=true
      ;;
    --map)
      [[ "${mode}" == core ]] || {
        echo "--slam, --multi, and --map are mutually exclusive." >&2
        exit 2
      }
      shift
      map_name="${1:-}"
      [[ "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
        echo "Map name may contain only letters, numbers, underscores, and hyphens." >&2
        exit 2
      }
      [[ -s "${root}/runtime/maps/${map_name}.yaml" ]] || {
        echo "Saved map does not exist: runtime/maps/${map_name}.yaml" >&2
        exit 1
      }
      mode=localization
      ;;
    --help|-h)
      echo "Usage: $0 [--gui] [--moving-actor | --stationary-actor] [--red-actor] [--slam | --multi-agent | --perception | --map MAP_NAME]"
      exit 0
      ;;
    *)
      echo "Usage: $0 [--gui] [--moving-actor | --stationary-actor] [--red-actor] [--slam | --multi-agent | --perception | --map MAP_NAME]" >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "${actor_motion}" != auto && "${mode}" != vision ]]; then
  echo "--moving-actor and --stationary-actor are valid only with --perception." >&2
  exit 2
fi
if [[ "${red_actor}" == true && "${mode}" != vision ]]; then
  echo "--red-actor is valid only with --perception." >&2
  exit 2
fi

# A translating target is the normal perception demonstration. The explicit
# stationary override remains available for repeatable detector-only tests.
if [[ "${actor_motion}" == auto ]]; then
  if [[ "${mode}" == vision ]]; then
    actor_motion=true
  else
    actor_motion=false
  fi
fi

mkdir -p \
  "${root}/runtime/artifacts" \
  "${root}/runtime/assets" \
  "${root}/runtime/maps" \
  "${root}/runtime/ros_logs" \
  "${root}/runtime/test_reports"
compose=(docker compose --project-directory "${root}" -f "${root}/compose.yaml")

export OMOKAI_MODE="${mode}"
export OMOKAI_ACTOR_MOTION="${actor_motion}"
export OMOKAI_RED_ACTOR="${red_actor}"
if [[ "${mode}" == slam ]]; then
  export OMOKAI_LAUNCH_FILE=slam_navigation.launch.py
  export OMOKAI_RVIZ="${gui}"
  export OMOKAI_MAP_FILE=
elif [[ "${mode}" == multi ]]; then
  export OMOKAI_LAUNCH_FILE=multi_robot_simulation.launch.py
  export OMOKAI_RVIZ="${gui}"
  export OMOKAI_MAP_FILE=
elif [[ "${mode}" == vision ]]; then
  export OMOKAI_LAUNCH_FILE=vision_rgbd.launch.py
  export OMOKAI_RVIZ="${gui}"
  export OMOKAI_MAP_FILE=
elif [[ "${mode}" == localization ]]; then
  export OMOKAI_LAUNCH_FILE=saved_map_navigation.launch.py
  export OMOKAI_RVIZ="${gui}"
  export OMOKAI_MAP_FILE="/data/maps/${map_name}.yaml"
else
  export OMOKAI_LAUNCH_FILE=core_navigation.launch.py
  export OMOKAI_RVIZ=false
  export OMOKAI_MAP_FILE=
fi

if [[ "${print_config}" == true ]]; then
  printf 'mode=%s\n' "${OMOKAI_MODE}"
  printf 'launch_file=%s\n' "${OMOKAI_LAUNCH_FILE}"
  printf 'gui=%s\n' "${gui}"
  printf 'actor_motion=%s\n' "${OMOKAI_ACTOR_MOTION}"
  printf 'red_actor=%s\n' "${OMOKAI_RED_ACTOR}"
  exit 0
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
