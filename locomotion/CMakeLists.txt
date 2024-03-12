cmake_minimum_required(VERSION 3.5.1)
project(quadruped_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMON_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/custom/include)
set(COMMON_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/custom/lib)
set(COMMON_INSTALL_DIR ${CMAKE_SOURCE_DIR}/custom/install)

include_directories(
    ${COMMON_INCLUDE_DIR}
    ${COMMON_INSTALL_DIR}/include
    /opt/homebrew/include
)

list(APPEND CMAKE_PREFIX_PATH ${COMMON_INSTALL_DIR})

list(APPEND CMAKE_INSTALL_RPATH ${COMMON_INSTALL_DIR}/lib)
list(APPEND CMAKE_INSTALL_RPATH ${COMMON_LIBRARY_DIR})
if($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    list(APPEND CMAKE_INSTALL_RPATH /opt/$ENV{ROS_DISTRO}/foxy/lib)
endif()

# Add subdirectories
add_subdirectory(src/utils)
add_subdirectory(src/dynamics)
add_subdirectory(src/communication)
add_subdirectory(src/execution)
add_subdirectory(src/robot_simulation)