#!/usr/bin/env bash
# Start the MuJoCo simulator.
#
# Usage:
#   ./scripts/run_mujoco.sh

echo "====================================================="
## Strict mode(exit immediately on any command failure) ##
set -eo pipefail

## cd root ##
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# source uv environment ##
if [ -z "${VIRTUAL_ENV:-}" ] && [ -f "$REPO_ROOT/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$REPO_ROOT/.venv/bin/activate"
fi
if [ -z "${VIRTUAL_ENV:-}" ]; then
    echo "!! no uv env active -- create it: uv venv --python 3.10 && source .venv/bin/activate" >&2
    exit 1
fi
echo "==> venv: $VIRTUAL_ENV"

## source ros ##
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
echo "==> ROS 2: $ROS_SETUP"
source "$ROS_SETUP"

## run mujoco ##
cd "$SCRIPT_DIR"
echo "==> python run_mujoco.py $*"
exec python run_mujoco.py "$@"
