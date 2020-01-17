################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(meiro_runner_robot)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(
  catkin REQUIRED COMPONENTS
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
install(PROGRAMS
  scripts/run_turtlebot_controller.sh
    scripts/run_local_rl_agent.sh
    scripts/run_rollout_rl_agent.sh
    scripts/run_evaluation_rl_agent.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
