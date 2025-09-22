cmake_minimum_required(VERSION 3.8)
project(fdsc_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
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
add_library(${PROJECT_NAME}
 ${FDSCSOURCE}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)
target_compile_options(${PROJECT_NAME} PUBLIC -W -Wall -Wextra -pedantic -pedantic-errors)

##############
## Highlevel ##
##############
add_executable(${PROJECT_NAME}_pushup_node
        ${FDSCSOURCE}
        src/examples/exp_pushups_highlevel.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_pushup_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_pushup_node
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_rotate90_node
      ${FDSCSOURCE}
      src/examples/exp_rotate_90_highlevel.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_rotate90_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_rotate90_node
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_walk_node
      ${FDSCSOURCE}
      src/examples/exp_walk_highlevel.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_walk_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_walk_node
  Eigen3::Eigen
)

##############
## Lowlevel ##
##############
add_executable(${PROJECT_NAME}_pd_node
        ${FDSCSOURCE}
        src/examples/exp_joint_position_control_lowlevel.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_pd_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_pd_node
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_vel_node
        ${FDSCSOURCE}
        src/examples/exp_joint_velocity_control_lowlevel.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_vel_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_vel_node
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_tau_node
        ${FDSCSOURCE}
        src/examples/exp_joint_torque_control_lowlevel.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_tau_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_tau_node
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_tau_tracking_node
        ${FDSCSOURCE}
        src/examples/exp_joint_torquetracking_control_lowlevel.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_tau_tracking_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_tau_tracking_node
  Eigen3::Eigen
)

###########
## Test ##
###########
add_executable(${PROJECT_NAME}_test_node
        ${FDSCSOURCE}
        # src/test/test_boost_udp.cpp
        src/test/test_common.cpp
        )
ament_target_dependencies(${PROJECT_NAME}_test_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_test_node
  Eigen3::Eigen
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
  ${PROJECT_NAME}_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs)

ament_package()
