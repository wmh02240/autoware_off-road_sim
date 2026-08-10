#!/usr/bin/env bash
set -euo pipefail

readonly LARIAD_REPOSITORY="https://github.com/LARIAD/Offroad-Nav.git"
readonly LARIAD_COMMIT="69640cf19eec1d90214ed4a3615788dd5dc1e3c4"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
destination="${1:-${repo_root}/external_assets/lariad_offroad_nav}"

# The project is commonly bind-mounted into a container as root while the
# checkout is owned by the host user. Scope Git's ownership exception to this
# exact pinned checkout and to each command; do not mutate global Git config.
git_in_checkout() {
    git -c "safe.directory=${destination}" -C "${destination}" "$@"
}

ensure_legacy_extension_link() {
    local link_parent="${destination}/isaac/extsUser"
    local link_path="${link_parent}/terrain.generator"
    local expected_target="../../terrain.generator"

    mkdir -p "${link_parent}"
    if [[ -L "${link_path}" ]]; then
        if [[ "$(readlink "${link_path}")" != "${expected_target}" ]]; then
            echo "Error: unexpected terrain.generator symlink target: ${link_path}" >&2
            exit 1
        fi
    elif [[ -e "${link_path}" ]]; then
        echo "Error: expected a symlink but found another file type: ${link_path}" >&2
        exit 1
    else
        ln -s "${expected_target}" "${link_path}"
    fi
}

if [[ -e "${destination}" ]]; then
    if [[ ! -d "${destination}/.git" ]]; then
        echo "Error: destination exists but is not a Git checkout: ${destination}" >&2
        exit 1
    fi

    actual_commit="$(git_in_checkout rev-parse HEAD)"
    if [[ "${actual_commit}" != "${LARIAD_COMMIT}" ]]; then
        echo "Error: existing checkout is at ${actual_commit}; expected ${LARIAD_COMMIT}." >&2
        echo "Move the directory aside and run this script again. No files were changed." >&2
        exit 1
    fi

    ensure_legacy_extension_link
    echo "LARIAD assets already match pinned commit ${LARIAD_COMMIT}."
    exit 0
fi

mkdir -p "$(dirname "${destination}")"
git clone --filter=blob:none --no-checkout "${LARIAD_REPOSITORY}" "${destination}"
git_in_checkout sparse-checkout init --cone
git_in_checkout sparse-checkout set assets terrain.generator LICENSE README.md
git_in_checkout checkout --detach "${LARIAD_COMMIT}"

actual_commit="$(git_in_checkout rev-parse HEAD)"
if [[ "${actual_commit}" != "${LARIAD_COMMIT}" ]]; then
    echo "Error: checkout verification failed: ${actual_commit}" >&2
    exit 1
fi

ensure_legacy_extension_link
echo "LARIAD environment assets installed at: ${destination}"
echo "Pinned commit: ${actual_commit}"
echo "Policy: internal evaluation only; see third_party/LARIAD_ASSET_MANIFEST.md"
