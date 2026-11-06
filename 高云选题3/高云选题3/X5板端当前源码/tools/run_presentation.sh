#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
# This service bridges the real robot's ROS topics and command router only.
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source ./env.sh
exec python3 tools/presentation_server.py --allow-local-addresses "$@"
