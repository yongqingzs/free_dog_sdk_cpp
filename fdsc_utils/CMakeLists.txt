cmake_minimum_required(VERSION 3.8)
project(fdsc_utils)

set(CMAKE_CXX_FLAGS "-std=c++2a")
set(CMAKE_BUILD_TYPE "Release")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
  include
)

set(FDSCSOURCE
 src/common.cpp
 src/unitreeConnectBoost.cpp
 src/lowState.cpp
 src/lowCmd.cpp
 src/highCmd.cpp
 src/highState.cpp
 )

# Create library
add_library(${PROJECT_NAME} SHARED
 ${FDSCSOURCE}
)

##############
## Highlevel ##
##############
add_executable(${PROJECT_NAME}_pushup_node
        ${FDSCSOURCE}
        src/examples/exp_pushups_highlevel.cpp
        )

add_executable(${PROJECT_NAME}_rotate90_node
      ${FDSCSOURCE}
      src/examples/exp_rotate_90_highlevel.cpp
        )

add_executable(${PROJECT_NAME}_walk_node
      ${FDSCSOURCE}
      src/examples/exp_walk_highlevel.cpp
        )

##############
## Lowlevel ##
##############
add_executable(${PROJECT_NAME}_pd_node
        ${FDSCSOURCE}
        src/examples/exp_joint_position_control_lowlevel.cpp
        )

add_executable(${PROJECT_NAME}_vel_node
        ${FDSCSOURCE}
        src/examples/exp_joint_velocity_control_lowlevel.cpp
        )

add_executable(${PROJECT_NAME}_tau_node
        ${FDSCSOURCE}
        src/examples/exp_joint_torque_control_lowlevel.cpp
        )

add_executable(${PROJECT_NAME}_tau_tracking_node
        ${FDSCSOURCE}
        src/examples/exp_joint_torquetracking_control_lowlevel.cpp
        )

add_executable(${PROJECT_NAME}_stand_up_node
        ${FDSCSOURCE}
        src/examples/exp_stand_up.cpp
        )

add_executable(${PROJECT_NAME}_state_node
        ${FDSCSOURCE}
        src/examples/exp_state_only.cpp
        )

###########
## Test ##
###########
add_executable(${PROJECT_NAME}_test_node
        ${FDSCSOURCE}
        # src/test/test_boost_udp.cpp
        src/test/test_common.cpp
        )

#############
## Install ##
#############

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install executables
install(TARGETS
  ${PROJECT_NAME}_pushup_node
  ${PROJECT_NAME}_rotate90_node
  ${PROJECT_NAME}_walk_node
  ${PROJECT_NAME}_pd_node
  ${PROJECT_NAME}_vel_node
  ${PROJECT_NAME}_tau_node
  ${PROJECT_NAME}_tau_tracking_node
  ${PROJECT_NAME}_stand_up_node
  ${PROJECT_NAME}_state_node
  ${PROJECT_NAME}_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs)

ament_package()
