<<<<<<< HEAD
=======
cmake_minimum_required(VERSION 3.5.1)
project(QuadrupedControl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


>>>>>>> 32acfb0ac560277c40ad17f56e1ea6f88c8f8b96
# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

set(OpenGL_GL_PREFERENCE GLVND)

# set(mujoco_DIR ${COMMON_INSTALL_DIR}/lib/cmake/mujoco)
# set(THREADS_PREFER_PTHREAD_FLAG ON)

# find_package(mujoco CONFIG REQUIRED)
<<<<<<< HEAD
# find_package(glfw3 REQUIRED)
=======
find_package(glfw3 REQUIRED)
>>>>>>> 32acfb0ac560277c40ad17f56e1ea6f88c8f8b96
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
<<<<<<< HEAD

add_executable(quad_sim src/main.cpp)
add_dependencies(quad_sim quad_communication)
target_link_libraries(quad_sim
=======
add_executable(simulate src/sim_main.cpp)
if (DEFINED ENV{USE_ROS_COMM})
    add_definitions(-DUSE_ROS_COMM)
endif()
add_dependencies(simulate quad_communication)
target_link_libraries(simulate
>>>>>>> 32acfb0ac560277c40ad17f56e1ea6f88c8f8b96
    # mujoco::mujoco
    mujoco
    glfw
    Threads::Threads
    quad_communication
<<<<<<< HEAD
)
install(TARGETS quad_sim
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)

add_executable(sim_ros src/main_ros.cpp)
add_dependencies(sim_ros ros_comm)
target_link_libraries(sim_ros
    # mujoco::mujoco
    mujoco
    glfw
    Threads::Threads
    # quad_communication
    ros_comm
)
install(TARGETS sim_ros
=======
    # $<$<BOOL:$ENV{USE_ROS_COMM}>:ros_comm>
)
if (DEFINED ENV{USE_ROS_COMM})
    target_link_libraries(simulate
        ros_comm
    )
endif()
install(TARGETS simulate
>>>>>>> 32acfb0ac560277c40ad17f56e1ea6f88c8f8b96
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)