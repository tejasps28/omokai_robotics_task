#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run.sh --slam COMMAND [OPTIONS]
  ./scripts/run.sh --multi-agent COMMAND [OPTIONS]
  ./scripts/run.sh --perception COMMAND [OPTIONS]

Examples:
  ./scripts/run.sh --slam start --gui
  ./scripts/run.sh --slam explore --exploration-id slam-demo
  ./scripts/run.sh --multi-agent start --gui
  ./scripts/run.sh --multi-agent run --planner fake "Form a wedge, split the route, and regroup."
  ./scripts/run.sh --perception start --gui --red-actor
  ./scripts/run.sh --perception run --coat-color white --yes

The three modes share the same Docker image and Compose service. The selected
mode changes the ROS launch file; existing run_slam.sh, run_multi_robot.sh, and
run_vision.sh commands remain supported.
EOF
}

mode=""
arguments=()
while (( $# > 0 )); do
  case "$1" in
    --slam)
      [[ -z "${mode}" ]] || {
        echo "Choose exactly one of --slam, --multi-agent, or --perception." >&2
        exit 2
      }
      mode=slam
      ;;
    --multi-agent|--multi)
      [[ -z "${mode}" ]] || {
        echo "Choose exactly one of --slam, --multi-agent, or --perception." >&2
        exit 2
      }
      mode=multi
      ;;
    --perception|--vision)
      [[ -z "${mode}" ]] || {
        echo "Choose exactly one of --slam, --multi-agent, or --perception." >&2
        exit 2
      }
      mode=vision
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      arguments+=("$1")
      ;;
  esac
  shift
done

[[ -n "${mode}" ]] || {
  usage >&2
  exit 2
}
(( ${#arguments[@]} > 0 )) || {
  echo "A command is required after the challenge flag." >&2
  usage >&2
  exit 2
}

case "${mode}" in
  slam)
    exec "${root}/scripts/run_slam.sh" "${arguments[@]}"
    ;;
  multi)
    exec "${root}/scripts/run_multi_robot.sh" "${arguments[@]}"
    ;;
  vision)
    exec "${root}/scripts/run_vision.sh" "${arguments[@]}"
    ;;
esac
