#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "${repo_dir}/../.." && pwd)"

isaac_input="${ISAAC_SIM_DIR:-}"
if [[ -z "${isaac_input}" ]]; then
    read -r -p "Path to Isaac Sim 6.0 root directory (ISAAC_SIM_DIR): " isaac_input
fi

if [[ "${isaac_input}" == "~" ]]; then
    isaac_input="${HOME}"
elif [[ "${isaac_input:0:2}" == "~/" ]]; then
    isaac_input="${HOME}/${isaac_input:2}"
fi

isaac_input="${isaac_input%/}"
candidates=(
    "${isaac_input}/python.sh"
    "${isaac_input}/_build/linux-x86_64/release/python.sh"
)

isaac_python=""
for candidate in "${candidates[@]}"; do
    if [[ -x "${candidate}" ]]; then
        isaac_python="${candidate}"
        break
    fi
done

if [[ -z "${isaac_python}" ]]; then
    echo "error: could not find an executable Isaac Sim python.sh under:" >&2
    printf '  %s\n' "${candidates[@]}" >&2
    exit 1
fi

isaac_dir="$(dirname "${isaac_python}")"
printf 'isaac_sim_dir: "%s"\n' "${isaac_dir}" > "${repo_dir}/share/isaac_sim.yaml"
echo "Configured Isaac Sim: ${isaac_dir}"
echo "Saved configuration: ${repo_dir}/share/isaac_sim.yaml"

# ROS setup hooks may reference optional unset variables, so temporarily
# disable nounset while sourcing the environment.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${workspace_dir}"
python3.12 -m colcon build --symlink-install "$@"
