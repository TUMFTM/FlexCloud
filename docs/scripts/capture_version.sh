#!/usr/bin/env bash
# Capture CLI help output and generate a Version Details page for one tag.
#
# Usage:
#   capture_version.sh <tag>
#
# <tag> is a Docker image tag of ghcr.io/tumftm/flexcloud, e.g. "latest"
# or "v1.0.1". The script introspects the image to determine the Ubuntu release,
# ROS 2 distro, Python version, and architecture, so older releases (e.g. those
# built on a different Ubuntu / ROS combination) are documented correctly without
# any version-specific logic in this script.
#
# Writes:
#   docs/version_details/snippets/help_keyframe_interpolation_<tag>.txt
#   docs/version_details/snippets/help_georeferencing_<tag>.txt
# For non-"latest" tags, also writes:
#   docs/version_details/<tag>.md
set -euo pipefail

TAG="${1:?usage: capture_version.sh <tag>}"
IMAGE="ghcr.io/tumftm/flexcloud:${TAG}"

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SNIP_DIR="${ROOT}/docs/version_details/snippets"
PAGE_DIR="${ROOT}/docs/version_details"
mkdir -p "${SNIP_DIR}"

echo "Pulling ${IMAGE}..."
docker pull "${IMAGE}" >/dev/null

# Introspect the image: Ubuntu release, ROS distro, Python version, arch, glibc.
META=$(docker run --rm --entrypoint /bin/bash "${IMAGE}" -c '
    . /etc/os-release
    ROS_DISTRO=$(ls /opt/ros 2>/dev/null | head -n1)
    PY=$(python3 --version 2>&1 | awk "{print \$2}")
    ARCH=$(uname -m)
    GLIBC=$(ldd --version 2>/dev/null | head -n1 | awk "{print \$NF}")
    echo "UBUNTU_VERSION=${VERSION_ID}"
    echo "UBUNTU_NAME=${VERSION_CODENAME^}"
    echo "ROS_DISTRO=${ROS_DISTRO}"
    echo "PY_VERSION=${PY}"
    echo "ARCH=${ARCH}"
    echo "GLIBC=${GLIBC}"
')
eval "${META}"

PY_MAJMIN=$(echo "${PY_VERSION}" | cut -d. -f1,2)
PY_TAG="cp$(echo "${PY_MAJMIN}" | tr -d .)"

run_help() {
    local cmd="$*"
    docker run --rm --entrypoint /bin/bash "${IMAGE}" \
        -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
            && source /ros_ws/install/setup.bash \
            && ${cmd} --help"
}

echo "Capturing help output (Ubuntu ${UBUNTU_VERSION}, ROS ${ROS_DISTRO}, Python ${PY_VERSION})..."
run_help ros2 run flexcloud keyframe_interpolation > "${SNIP_DIR}/help_keyframe_interpolation_${TAG}.txt"
run_help ros2 run flexcloud georeferencing         > "${SNIP_DIR}/help_georeferencing_${TAG}.txt"

if [[ "${TAG}" == "latest" ]]; then
    echo "Skipping page generation for 'latest' (page is committed)."
    exit 0
fi

PAGE="${PAGE_DIR}/${TAG}.md"
cat > "${PAGE}" <<EOF
# ${TAG}

Frozen snapshot captured from \`${IMAGE}\` at release time.

## Supported platforms

| Component        | Requirement                                              |
|------------------|----------------------------------------------------------|
| Operating system | Ubuntu ${UBUNTU_VERSION} (${UBUNTU_NAME}), ${ARCH}       |
| glibc            | ${GLIBC}                                                 |
| ROS 2 distro     | ${ROS_DISTRO^}                                           |
| Python           | CPython ${PY_VERSION}                                    |
| Wheel tag        | \`${PY_TAG}-${PY_TAG}-manylinux_*_${ARCH}\`              |
| Architectures    | \`${ARCH}\` only                                         |

## \`flexcloud-keyframe-interpolation\`

\`\`\`text
--8<-- "version_details/snippets/help_keyframe_interpolation_${TAG}.txt"
\`\`\`

## \`flexcloud-georeferencing\`

\`\`\`text
--8<-- "version_details/snippets/help_georeferencing_${TAG}.txt"
\`\`\`
EOF
echo "Wrote ${PAGE}"

# Rebuild the version table in the index AND the mkdocs.yml nav section,
# both sorted newest-first by version (sort -Vr).
INDEX="${PAGE_DIR}/index.md"
MKDOCS="${ROOT}/mkdocs.yml"
VERSIONS=$(find "${PAGE_DIR}" -maxdepth 1 -type f -name '*.md' \
        ! -name 'index.md' ! -name 'latest.md' 2>/dev/null \
    | xargs -n1 basename 2>/dev/null \
    | sed 's/\.md$//' \
    | sort -Vr || true)

{
    awk '/^## Available versions/{exit} {print}' "${INDEX}"
    echo "## Available versions"
    echo
    echo "| Version  | Status   | Page                                |"
    echo "|----------|----------|-------------------------------------|"
    echo "| latest   | rolling  | [Latest](latest.md)                 |"
    for v in ${VERSIONS}; do
        echo "| ${v} | released | [${v}](${v}.md) |"
    done
    echo "<!-- VERSION_TABLE_INSERT -->"
    awk 'found{print} /<!-- VERSION_TABLE_INSERT -->/{found=1}' "${INDEX}"
} > "${INDEX}.new"
mv "${INDEX}.new" "${INDEX}"
echo "Rebuilt version table in ${INDEX}"

# Rebuild the mkdocs.yml nav block between # VERSION_NAV_START / # VERSION_NAV_END.
{
    awk '
        /# VERSION_NAV_START/ { print; in_block=1; next }
        /# VERSION_NAV_END/   { in_block=0 }
        !in_block             { print }
    ' "${MKDOCS}"
} > "${MKDOCS}.new"

# Insert version entries just before the END marker.
awk -v versions="${VERSIONS}" '
    /# VERSION_NAV_END/ {
        n = split(versions, v, "\n")
        for (i = 1; i <= n; i++) {
            if (v[i] != "") {
                printf "      - version_details/%s.md\n", v[i]
            }
        }
    }
    { print }
' "${MKDOCS}.new" > "${MKDOCS}.new2"
mv "${MKDOCS}.new2" "${MKDOCS}"
rm -f "${MKDOCS}.new"
echo "Rebuilt Version Details nav in ${MKDOCS}"
