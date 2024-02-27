cmake_minimum_required(VERSION 3.5.1)
project(QuadrupedControl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

set(OpenGL_GL_PREFERENCE GLVND)

# set(mujoco_DIR ${COMMON_INSTALL_DIR}/lib/cmake/mujoco)
# set(THREADS_PREFER_PTHREAD_FLAG ON)

# find_package(mujoco CONFIG REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${COMMON_INSTALL_DIR}/include
    ${COMMON_INSTALL_DIR}/install/include
    include
)
link_directories(${COMMON_INSTALL_DIR}/lib)

install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)
add_executable(simulate src/sim_main.cpp)
if (DEFINED ENV{USE_ROS_COMM})
    add_definitions(-DUSE_ROS_COMM)
endif()
add_dependencies(simulate quad_communication)
target_link_libraries(simulate
    # mujoco::mujoco
    mujoco
    glfw
    Threads::Threads
    quad_communication
    # $<$<BOOL:$ENV{USE_ROS_COMM}>:ros_comm>
)
if (DEFINED ENV{USE_ROS_COMM})
    target_link_libraries(simulate
        ros_comm
    )
endif()
install(TARGETS simulate
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)