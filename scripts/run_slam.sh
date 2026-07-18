#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(
  docker compose
  --project-directory "${root}"
  -f "${root}/compose.yaml"
)

usage() {
  echo "Usage: $0 start [--gui] | explore [options] | status [id] | cancel | save-map [name] | stop"
}

command="${1:-}"
case "${command}" in
  start)
    shift
    "${root}/scripts/start.sh" --slam "$@"
    ;;
  explore)
    shift
    slam_state="$(
      "${compose[@]}" exec -T robot /usr/local/bin/omokai-entrypoint \
        ros2 lifecycle get /slam_toolbox 2>/dev/null || true
    )"
    [[ "${slam_state}" == active* ]] || {
      echo "Start online mapping first with ./scripts/start.sh --slam." >&2
      exit 1
    }
    exploration_id=""
    arguments=("$@")
    for ((index = 0; index < ${#arguments[@]}; index++)); do
      if [[ "${arguments[index]}" == "--exploration-id" ]]; then
        exploration_id="${arguments[index + 1]:-}"
        break
      fi
    done
    if [[ -z "${exploration_id}" ]]; then
      exploration_id="slam-$(date -u +%Y%m%dT%H%M%SZ)"
      arguments=(--exploration-id "${exploration_id}" "${arguments[@]}")
    fi
    echo "Exploration ID: ${exploration_id}"
    result=0
    "${compose[@]}" exec -T robot /usr/local/bin/omokai-entrypoint \
      ros2 run omokai_exploration explore \
      --artifact-root /data/artifacts \
      "${arguments[@]}" || result=$?
    echo "Exploration evidence: runtime/artifacts/${exploration_id}/"
    exit "${result}"
    ;;
  status)
    exploration_id="${2:-}"
    status_file=""
    if [[ -n "${exploration_id}" ]]; then
      [[ "${exploration_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$ \
         && "${exploration_id}" != *..* ]] || {
        echo "Exploration ID contains unsupported characters." >&2
        exit 2
      }
      status_file="${root}/runtime/artifacts/${exploration_id}/exploration_status.json"
    else
      shopt -s nullglob
      status_files=(
        "${root}"/runtime/artifacts/*/exploration_status.json
      )
      for candidate in "${status_files[@]}"; do
        if [[ -z "${status_file}" || "${candidate}" -nt "${status_file}" ]]; then
          status_file="${candidate}"
        fi
      done
    fi
    [[ -n "${status_file}" && -s "${status_file}" ]] || {
      echo "No exploration status artifact was found." >&2
      exit 1
    }
    cat "${status_file}"
    ;;
  cancel)
    "${compose[@]}" exec -T robot /usr/local/bin/omokai-entrypoint \
      timeout 10 ros2 service call \
      /omokai_exploration/cancel \
      std_srvs/srv/Trigger \
      '{}'
    ;;
  save-map)
    map_name="${2:-map-$(date -u +%Y%m%dT%H%M%SZ)}"
    [[ "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
      echo "Map name may contain only letters, numbers, underscores, and hyphens." >&2
      exit 2
    }

    slam_state="$(
      "${compose[@]}" exec -T robot /usr/local/bin/omokai-entrypoint \
        ros2 lifecycle get /slam_toolbox 2>/dev/null || true
    )"
    [[ "${slam_state}" == active* ]] || {
      echo "Start online mapping first with ./scripts/start.sh --slam." >&2
      exit 1
    }

    mkdir -p "${root}/runtime/maps"
    "${compose[@]}" exec -T robot /usr/local/bin/omokai-entrypoint \
      ros2 run nav2_map_server map_saver_cli \
      -f "/data/maps/${map_name}" \
      --ros-args -p save_map_timeout:=20.0

    map_yaml="${root}/runtime/maps/${map_name}.yaml"
    [[ -s "${map_yaml}" ]] || {
      echo "Map metadata was not created: ${map_yaml}" >&2
      exit 1
    }
    if [[ ! -s "${root}/runtime/maps/${map_name}.pgm" \
       && ! -s "${root}/runtime/maps/${map_name}.png" ]]; then
      echo "Map image was not created for ${map_name}." >&2
      exit 1
    fi

    echo "Saved map: runtime/maps/${map_name}.yaml"
    ;;
  stop)
    "${root}/scripts/stop.sh"
    ;;
  --help|-h)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
