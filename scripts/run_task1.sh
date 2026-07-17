#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner="gemini"
assume_yes=false

while (( $# > 0 )); do
  case "$1" in
    --planner)
      planner="${2:-}"
      shift 2
      ;;
    --yes)
      assume_yes=true
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--planner fake|gemini|openai] [--yes] [mission text]"
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

case "${planner}" in
  fake|gemini|openai) ;;
  *)
    echo "Planner must be fake, gemini, or openai." >&2
    exit 2
    ;;
esac

prompt="$*"
if [[ -z "${prompt// }" ]]; then
  read -r -p "Mission: " prompt
fi
[[ -n "${prompt// }" ]] || {
  echo "Mission cannot be blank." >&2
  exit 2
}

health="$(docker inspect --format '{{.State.Health.Status}}' omokai-task1 2>/dev/null || true)"
[[ "${health}" == "healthy" ]] || {
  echo "Start the simulation first with ./scripts/start.sh or ./scripts/start.sh --gui." >&2
  exit 1
}

mission_id="task1-$(date -u +%Y%m%dT%H%M%SZ)"
command=(
  docker compose --project-directory "${root}" -f "${root}/compose.yaml"
  exec robot /usr/local/bin/omokai-entrypoint
  ros2 run omokai_pipeline omokai-operator run
  --planner "${planner}"
  --mission-id "${mission_id}"
  --artifact-root /data/artifacts
)
if [[ "${assume_yes}" == true ]]; then
  command+=(--yes)
fi
command+=("${prompt}")

"${command[@]}"
echo "Mission evidence: runtime/artifacts/${mission_id}/"
