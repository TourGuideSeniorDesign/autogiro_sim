# cmake_minimum_required(VERSION 3.12)
# project(autogiro)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# install(
#   DIRECTORY config description launch worlds
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()


#NEW CMAKE FOR UBUNTU 22.04 VM 
cmake_minimum_required(VERSION 3.12)
project(autogiro)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# ROS 2 Humble defaults to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find build-time dependencies
find_package(ament_cmake REQUIRED)

# If you add C++ nodes later, you'll add find_package(rclcpp REQUIRED) here

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# This is the most important part for your URDF and Launch files
install(
  DIRECTORY config description launch maps worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/crop_self_hits.py
    scripts/save_map.sh
    scripts/tour_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
