#!/bin/bash

# This script starts the robot_state_publisher for the autogiro project.

# Source the main ROS 2 installation
source /opt/homebrew/Caskroom/miniforge/base/envs/ros2/setup.bash

# Source the local workspace setup
# This assumes the script is run from the root of the ros2_ws or that 'install/setup.bash' is otherwise accessible.
# For simplicity, assuming it's run from ros2_ws root or the script is called from within ros2_ws
source install/setup.bash

# Launch the rsp.launch.py file
ros2 launch autogiro rsp.launch.py
