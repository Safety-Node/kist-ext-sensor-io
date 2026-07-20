#!/bin/bash
# Build the container image.

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
    -t kist-ext-sensor-io \
    -f "${REPO_DIR}/docker/Dockerfile" \
    "${REPO_DIR}"
