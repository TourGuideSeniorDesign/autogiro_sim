#!/usr/bin/env bash
# Save the current slam_toolbox map to disk in both nav2 (.yaml/.pgm)
# and slam_toolbox (.posegraph/.data) formats.
#
# Usage: save_map.sh <map_name> [output_dir]
#   output_dir defaults to $PWD/maps if that directory exists, else $PWD.
#
# Example:
#   cd ~/ros2_ws/autogiro_sim/autogiro
#   ros2 run autogiro save_map.sh acopian
#   # -> maps/acopian.yaml, acopian.pgm, acopian.posegraph, acopian.data

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <map_name> [output_dir]" >&2
    exit 1
fi

NAME="$1"
DIR="${2:-}"

if [ -z "$DIR" ]; then
    if [ -d "$PWD/maps" ]; then
        DIR="$PWD/maps"
    else
        DIR="$PWD"
    fi
fi

mkdir -p "$DIR"
BASE="$(cd "$DIR" && pwd)/$NAME"

echo "Saving map to: $BASE.{yaml,pgm,posegraph,data}"

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$BASE'}}"

ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$BASE'}"

echo "Done."
