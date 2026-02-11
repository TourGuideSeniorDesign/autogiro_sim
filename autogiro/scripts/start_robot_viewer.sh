#!/bin/bash

# This script streamlines the startup process for the autogiro robot viewer in RViz2.

# --- Configuration ---
CONDA_ENV_NAME="ros2"
ROS2_INSTALL_PATH="/opt/homebrew/Caskroom/miniforge/base/envs/ros2"
WORKSPACE_ROOT="/Users/ben/ros2_ws"
AUTOGIRO_PACKAGE_PATH="${WORKSPACE_ROOT}/src/autogiro"
RVIZ_CONFIG_PATH="${AUTOGIRO_PACKAGE_PATH}/config/view_bot.rviz"
RSP_LAUNCH_FILE="rsp.launch.py"

# --- Correctly activate Conda Environment for scripting ---
echo "Activating conda environment: ${CONDA_ENV_NAME}"
# Initialize conda for bash scripts
source /opt/homebrew/Caskroom/miniforge/base/etc/profile.d/conda.sh
conda activate "${CONDA_ENV_NAME}" || { echo "Error: Failed to activate conda environment '${CONDA_ENV_NAME}'. Please ensure it exists."; exit 1; }

# --- Navigate to Workspace ---
echo "Navigating to workspace: ${WORKSPACE_ROOT}"
cd "${WORKSPACE_ROOT}" || { echo "Error: Failed to navigate to workspace root."; exit 1; }

# --- Build Package (optional, uncomment to enable) ---
echo "Building autogiro package..."
if ! colcon build --packages-select autogiro; then
    echo "Error: colcon build failed."
    exit 1
fi

# --- Source Workspace ---
echo "Sourcing ROS 2 environment and workspace..."
# Source main ROS 2 installation
source "${ROS2_INSTALL_PATH}/setup.bash"
# Source local workspace setup
source "install/setup.bash"

# --- Launch Robot State Publisher (RSP) ---
echo "Launching Robot State Publisher (${RSP_LAUNCH_FILE})..."
ros2 launch autogiro "${RSP_LAUNCH_FILE}" & # Run in background

# Give RSP a moment to start
sleep 2

# --- Launch RViz2 ---
echo "Launching RViz2 with configuration: ${RVIZ_CONFIG_PATH}"
rviz2 -d "${RVIZ_CONFIG_PATH}"

echo "Startup script finished."
