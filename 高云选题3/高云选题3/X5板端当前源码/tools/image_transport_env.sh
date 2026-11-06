#!/usr/bin/env bash
# Source only in the large-image producers, before ROS initializes Fast DDS.
# Small-message nodes keep their normal pools. Honor explicit operator profiles.
if [ "${RESCUE_LARGE_IMAGE_SHM:-1}" = 1 ] && \
   [ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" = rmw_fastrtps_cpp ] && \
   [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && \
   [ -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]; then
  rescue_image_profile="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)/config/fastdds_large_images.xml"
  if [ ! -r "$rescue_image_profile" ]; then
    echo "Missing image transport profile: $rescue_image_profile" >&2
    return 1
  fi
  export FASTRTPS_DEFAULT_PROFILES_FILE="$rescue_image_profile"
  unset rescue_image_profile
fi
