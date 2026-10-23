#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
    python3 "$ROOT/scripts/configure.py" --help
    echo 'bootstrap also installs system dependencies, fetches locked source and builds. --dry-run performs no writes.'
    exit 0
fi
for arg in "$@"; do
    if [[ "$arg" == --dry-run ]]; then
        bash "$ROOT/scripts/install_system.sh" --dry-run
        python3 "$ROOT/scripts/fetch_dependencies.py" --dry-run
        echo 'Then: configure, Isaac Python dependencies and Pegasus editable install, Agent/PX4/ROS build, doctor.'
        exit 0
    fi
done
# Validate arguments before changing the machine.
python3 "$ROOT/scripts/configure.py" --validate-only "$@"
bash "$ROOT/scripts/install_system.sh"
python3 "$ROOT/scripts/fetch_dependencies.py"
python3 "$ROOT/scripts/configure.py" "$@"
source "$ROOT/simulation/environment.sh"
if [[ "$ZAV_SIM_PLATFORM" == wsl ]]; then
    (cd /mnt/c && /mnt/c/Windows/System32/cmd.exe /c call "$(wslpath -m "$ISAAC_WINDOWS_PYTHON_BAT")" -m pip install pymavlink==2.4.50 scipy==1.15.3 pyyaml==6.0.2)
else
    env -u PYTHONPATH "$ISAAC_PYTHON" -m pip install pymavlink==2.4.50 scipy==1.15.3 pyyaml==6.0.2
fi
bash "$ROOT/scripts/build.sh"
python3 "$ROOT/scripts/doctor.py"
echo "Install complete. Open new terminals, or: source '$ROOT/simulation/ros_setup.sh'"
