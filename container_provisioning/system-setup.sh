#!/bin/bash
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
${SCRIPT_DIR}/scripts/install-tools.sh
${SCRIPT_DIR}/scripts/install-nvidia-driver.sh
${SCRIPT_DIR}/scripts/install-docker.sh
${SCRIPT_DIR}/scripts/init-directory.sh