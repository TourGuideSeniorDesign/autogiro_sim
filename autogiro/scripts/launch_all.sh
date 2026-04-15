#!/usr/bin/env bash
# All-in-one launcher for the autogiro simulation.
#   - builds the workspace, sources it
#   - starts floorplan sim, static TF, SLAM, rviz, teleop each in their own xterm
#
# Requires: xterm, a working display (WSLg/X11).

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

RVIZ_CONFIG="${PKG_DIR}/config/main.rviz"
[ -f "${RVIZ_CONFIG}" ] || RVIZ_CONFIG="${PKG_DIR}/config/view_bot.rviz"

PIDFILE="${TMPDIR:-/tmp}/autogiro_launch.${USER:-user}.pids"

# Use a TrueType font so xterm doesn't whine about missing bitmap fonts.
XTERM_FONT_ARGS=(-fa "DejaVu Sans Mono" -fs 10)

# name:pgrep-pattern — authoritative list used by prelaunch and cleanup.
COMPONENTS=(
  "floorplan:ros2 launch autogiro launch_floorplan.launch.py"
  "static_tf:static_transform_publisher .*laser_frame"
  "slam:slam_toolbox .*online_async_launch"
  "rviz:rviz2"
  "teleop:teleop_twist_keyboard"
  "nav2:nav2_bringup .*navigation_launch"
  "gazebo:(gz sim|ign gazebo|ruby .*gz sim)"
  "bridge:ros_gz_bridge parameter_bridge"
  "pc2ls:pointcloud_to_laserscan_node"
  "rsp:robot_state_publisher"
  "spawner:controller_manager .*spawner"
)

FORCE=0
NO_BUILD=0
SKIP_DEP_CHECK=0
usage() {
    cat <<EOF
Usage: $(basename "$0") [--force|-y] [--no-build] [--skip-dep-check] [--help]

  --force, -y         Kill any running sim components without prompting.
  --no-build          Skip 'colcon build'.
  --skip-dep-check    Skip the apt dependency precheck.
  --help              Show this message.
EOF
}
while [ $# -gt 0 ]; do
    case "$1" in
        --force|-y)       FORCE=1 ;;
        --no-build)       NO_BUILD=1 ;;
        --skip-dep-check) SKIP_DEP_CHECK=1 ;;
        --help|-h)        usage; exit 0 ;;
        *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
    esac
    shift
done

REQUIRED_PKGS=(
    ros-humble-gz-ros2-control
    ros-humble-ros-gz-sim
    ros-humble-ros-gz-bridge
    ros-humble-slam-toolbox
    ros-humble-nav2-bringup
    ros-humble-pointcloud-to-laserscan
    ros-humble-twist-mux
    ros-humble-teleop-twist-keyboard
    ros-humble-robot-state-publisher
    ros-humble-tf2-ros
    ros-humble-ros2-control
    ros-humble-ros2-controllers
)

dep_check() {
    local missing=()
    local p
    for p in "${REQUIRED_PKGS[@]}"; do
        dpkg -s "${p}" >/dev/null 2>&1 || missing+=("${p}")
    done
    if [ ${#missing[@]} -gt 0 ]; then
        echo "[launch_all] Missing apt packages:" >&2
        printf '  %s\n' "${missing[@]}" >&2
        echo "" >&2
        echo "Install with:" >&2
        echo "  sudo apt update && sudo apt install -y ${missing[*]}" >&2
        echo "" >&2
        echo "Or rerun with --skip-dep-check to bypass this check." >&2
        exit 1
    fi
}

PIDS=()
CLEANED=0

kill_pids() {
    local sig="$1"; shift
    local p
    for p in "$@"; do
        [ -n "${p}" ] && kill -"${sig}" "${p}" 2>/dev/null || true
    done
}

# Collect currently-running PIDs matching any COMPONENTS pattern.
collect_component_pids() {
    local entry name pat hits all=()
    for entry in "${COMPONENTS[@]}"; do
        name="${entry%%:*}"
        pat="${entry#*:}"
        hits="$(pgrep -f "${pat}" 2>/dev/null || true)"
        if [ -n "${hits}" ]; then
            while read -r p; do all+=("${p}"); done <<< "${hits}"
        fi
    done
    printf "%s\n" "${all[@]}"
}

# Print a human-readable summary of what's currently running.
report_running() {
    local entry name pat hits any=0
    for entry in "${COMPONENTS[@]}"; do
        name="${entry%%:*}"
        pat="${entry#*:}"
        hits="$(pgrep -af "${pat}" 2>/dev/null || true)"
        if [ -n "${hits}" ]; then
            any=1
            echo "  [${name}]"
            echo "${hits}" | sed 's/^/    /'
        fi
    done
    return $([ ${any} -eq 1 ] && echo 0 || echo 1)
}

prelaunch_check() {
    # 1. Stale PIDs from a previous run.
    if [ -f "${PIDFILE}" ]; then
        while read -r p; do
            [ -n "${p}" ] && kill -TERM "${p}" 2>/dev/null || true
        done < "${PIDFILE}"
    fi

    # 2. Pattern-based discovery.
    if report_running >/dev/null; then
        echo "[launch_all] Existing sim processes detected:"
        report_running || true
        if [ "${FORCE}" -ne 1 ]; then
            read -r -p "Stop them and continue? [y/N] " ans
            case "${ans}" in
                y|Y|yes|YES) ;;
                *) echo "Aborted."; exit 1 ;;
            esac
        fi
        local entry pat
        for entry in "${COMPONENTS[@]}"; do
            pat="${entry#*:}"
            pkill -TERM -f "${pat}" 2>/dev/null || true
        done
        sleep 2
        for entry in "${COMPONENTS[@]}"; do
            pat="${entry#*:}"
            pkill -KILL -f "${pat}" 2>/dev/null || true
        done
    fi

    : > "${PIDFILE}"
}

cleanup() {
    [ "${CLEANED}" -eq 1 ] && return
    CLEANED=1
    echo "[launch_all] Shutting down..."

    # SIGTERM tracked xterms.
    kill_pids TERM "${PIDS[@]}"
    # SIGTERM each component pattern.
    local entry pat
    for entry in "${COMPONENTS[@]}"; do
        pat="${entry#*:}"
        pkill -TERM -f "${pat}" 2>/dev/null || true
    done
    sleep 2
    # SIGKILL stragglers.
    kill_pids KILL "${PIDS[@]}"
    for entry in "${COMPONENTS[@]}"; do
        pat="${entry#*:}"
        pkill -KILL -f "${pat}" 2>/dev/null || true
    done
    pkill -P $$ 2>/dev/null || true
    rm -f "${PIDFILE}"
}
trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT

if ! command -v xterm >/dev/null; then
    echo "xterm is required (apt install xterm)." >&2
    exit 1
fi
if [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is not set — start WSLg / an X server first." >&2
    exit 1
fi

if [ "${SKIP_DEP_CHECK}" -eq 0 ]; then
    dep_check
else
    echo "[launch_all] Skipping apt dependency check (--skip-dep-check)"
fi

prelaunch_check

echo "[launch_all] Workspace: ${WS_DIR}"
echo "[launch_all] Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ "${NO_BUILD}" -eq 0 ]; then
    echo "[launch_all] colcon build (in ${WS_DIR})"
    ( cd "${WS_DIR}" && colcon build --symlink-install ) || {
        echo "colcon build failed." >&2
        exit 1
    }
else
    echo "[launch_all] Skipping colcon build (--no-build)"
fi

WS_SETUP="${WS_DIR}/install/setup.bash"
if [ -f "${WS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${WS_SETUP}"
else
    echo "[launch_all] Warning: ${WS_SETUP} not found (is this a colcon workspace?)" >&2
fi

SOURCE_CMD="source /opt/ros/${ROS_DISTRO}/setup.bash; [ -f ${WS_SETUP} ] && source ${WS_SETUP}"

# Run a command in the background, prefixing each stdout/stderr line with [name]
# so all components share this single terminal.
run_bg() {
    local name="$1"; shift
    { stdbuf -oL -eL "$@" 2>&1 | sed -u "s/^/[${name}] /"; } &
    local pid=$!
    PIDS+=("${pid}")
    echo "${pid}" >> "${PIDFILE}"
}

echo "[launch_all] Starting floorplan sim..."
run_bg "floorplan" ros2 launch autogiro launch_floorplan.launch.py
sleep 6

echo "[launch_all] Starting rviz2..."
run_bg "rviz2" rviz2 -d "${RVIZ_CONFIG}"

echo "[launch_all] Starting SLAM toolbox..."
run_bg "slam" ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

echo "[launch_all] Starting teleop_twist_keyboard (focus its window to drive)..."
# Teleop needs its own terminal because it reads stdin directly.
# stderr is redirected to /dev/null to hide xterm's cosmetic bitmap-font probe
# warning ("cannot load font -misc-fixed-...") — the -fa TrueType font is used.
xterm "${XTERM_FONT_ARGS[@]}" -title "autogiro teleop — focus to drive" \
      -geometry 80x24 -hold \
      -e bash -lc "${SOURCE_CMD}; exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
              --ros-args -r cmd_vel:=cmd_vel_joy" 2>/dev/null &
TELEOP_PID=$!
PIDS+=("${TELEOP_PID}")
echo "${TELEOP_PID}" >> "${PIDFILE}"

echo "[launch_all] All components started. Focus the teleop xterm to drive."
echo "[launch_all] Press Ctrl-C here to tear everything down."
wait
