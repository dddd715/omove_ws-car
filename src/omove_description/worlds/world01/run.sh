#!/usr/bin/env bash
set -euo pipefail
bundle_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${bundle_root}/setup.bash"
# All models are embedded and all material resources are local.
export GAZEBO_MODEL_DATABASE_URI=""
simulator=gazebo
if [[ "${1:-}" == "--headless" ]]; then
    simulator=gzserver
    shift
fi
if ! command -v "$simulator" >/dev/null 2>&1; then
    echo "需要安装 Gazebo Classic 11（缺少 ${simulator}）。" >&2
    exit 127
fi
exec "$simulator" --verbose "${bundle_root}/worlds/playground.world" "$@"
