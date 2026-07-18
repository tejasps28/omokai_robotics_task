#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(
  docker compose
  --project-directory "${root}"
  -f "${root}/compose.yaml"
)

usage() {
  echo "Usage: $0 save-map [map-name]"
}

command="${1:-}"
case "${command}" in
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
  --help|-h)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
