#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
extension_source="${repo_root}/extensions/lawn.terrain.generator"
isaac_release="${ISAAC_SIM_RELEASE:-/root/isaacsim/_build/linux-x86_64/release}"
extension_link="${isaac_release}/exts/lawn.terrain.generator"

if [[ ! -f "${extension_source}/config/extension.toml" ]]; then
    echo "Error: extension manifest not found: ${extension_source}/config/extension.toml" >&2
    exit 1
fi
if [[ ! -d "${isaac_release}/exts" ]]; then
    echo "Error: Isaac Sim extension directory not found: ${isaac_release}/exts" >&2
    echo "Set ISAAC_SIM_RELEASE to the Isaac Sim release directory." >&2
    exit 1
fi

if [[ -L "${extension_link}" ]]; then
    current_target="$(readlink "${extension_link}")"
    if [[ "${current_target}" != "${extension_source}" ]]; then
        echo "Error: existing link has an unexpected target: ${extension_link} -> ${current_target}" >&2
        exit 1
    fi
elif [[ -e "${extension_link}" ]]; then
    echo "Error: refusing to replace non-symlink path: ${extension_link}" >&2
    exit 1
else
    ln -s "${extension_source}" "${extension_link}"
fi

echo "Registered lawn.terrain.generator: ${extension_link} -> ${extension_source}"
echo "Launch with: ${isaac_release}/isaac-sim.sh --enable lawn.terrain.generator"

