# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

set(OpenGL_GL_PREFERENCE GLVND)

# set(mujoco_DIR ${COMMON_INSTALL_DIR}/lib/cmake/mujoco)
# set(THREADS_PREFER_PTHREAD_FLAG ON)

# find_package(mujoco CONFIG REQUIRED)
# find_package(glfw3 REQUIRED)
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

add_executable(quad_sim src/main.cpp)
add_dependencies(quad_sim quad_communication)
target_link_libraries(quad_sim
    # mujoco::mujoco
    mujoco
    glfw
    Threads::Threads
    quad_communication
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
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)