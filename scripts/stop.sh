#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker compose --project-directory "${root}" -f "${root}/compose.yaml" down --remove-orphans
