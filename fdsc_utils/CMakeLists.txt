cmake_minimum_required(VERSION 3.0.2)
project(fdsc_utils)

set(CMAKE_CXX_FLAGS "-std=c++2a")
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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

#  add_library(${PROJECT_NAME}
#  ${FDSCSOURCE}
# )

# Detect architecture and set library suffix
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(LIB_SUFFIX "_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(LIB_SUFFIX "_amd64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 生成共享库（动态库）
add_library(${PROJECT_NAME}${LIB_SUFFIX} SHARED
 ${FDSCSOURCE}
)

target_compile_options(${PROJECT_NAME}${LIB_SUFFIX} PUBLIC -W -Wall -Wextra -pedantic -pedantic-errors)

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

add_executable(${PROJECT_NAME}_state_node
        ${FDSCSOURCE}
        src/examples/exp_state_only.cpp
        )

add_executable(${PROJECT_NAME}_stand_up_node
        ${FDSCSOURCE}
        src/examples/exp_stand_up.cpp
        )

###########
## Test ##
###########
add_executable(${PROJECT_NAME}_test_node
        ${FDSCSOURCE}
        # src/test/test_boost_udp.cpp
        src/test/test_common.cpp
        )
