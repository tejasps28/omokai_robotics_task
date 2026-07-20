#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(
  docker compose
  --project-directory "${root}"
  -f "${root}/compose.yaml"
)

command="${1:-}"
case "${command}" in
  start)
    shift
    "${root}/scripts/start.sh" --multi "$@"
    ;;
  preview|run|status|cancel)
    shift
    health="$(docker inspect --format '{{.State.Health.Status}}' omokai-task1 2>/dev/null || true)"
    [[ "${health}" == healthy ]] || {
      echo "Start multi-robot mode first: ./scripts/run_multi_robot.sh start" >&2
      exit 1
    }
    "${compose[@]}" exec robot /usr/local/bin/omokai-entrypoint \
      ros2 run omokai_fleet omokai-squad "${command}" "$@"
    ;;
  stop)
    "${root}/scripts/stop.sh"
    ;;
  *)
    echo "Usage: $0 start [--gui] | preview [options] [prompt] | run [options] [prompt] | status [id] | cancel | stop" >&2
    exit 2
    ;;
esac
