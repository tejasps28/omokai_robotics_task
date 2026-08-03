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
    "${root}/scripts/start.sh" --vision "$@"
    ;;
  preview|run|status|cancel)
    shift
    health="$(docker inspect --format '{{.State.Health.Status}}' omokai-task1 2>/dev/null || true)"
    [[ "${health}" == healthy ]] || {
      echo "Start vision mode first: ./scripts/run_vision.sh start [--gui]" >&2
      exit 1
    }
    running_mode="$(
      docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' \
        omokai-task1 2>/dev/null \
        | sed -n 's/^OMOKAI_MODE=//p'
    )"
    [[ "${running_mode}" == vision ]] || {
      echo "The running container is in '${running_mode:-unknown}' mode, not perception mode." >&2
      echo "Restart it with: ./scripts/run_vision.sh start [--gui]" >&2
      exit 1
    }
    "${compose[@]}" exec robot /usr/local/bin/omokai-entrypoint \
      ros2 run omokai_following omokai-vision "${command}" "$@"
    ;;
  validate)
    shift
    "${compose[@]}" exec robot /usr/local/bin/omokai-entrypoint \
      ros2 run omokai_bringup vision_live_validation "$@"
    ;;
  accept)
    shift
    "${compose[@]}" exec robot /usr/local/bin/omokai-entrypoint \
      ros2 run omokai_following vision_acceptance "$@"
    ;;
  stop)
    "${root}/scripts/stop.sh"
    ;;
  *)
    echo "Usage: $0 start [--gui] [--stationary-actor] [--red-actor] | preview [options] | run [options] | status | cancel | validate [options] | accept [options] | stop" >&2
    exit 2
    ;;
esac
