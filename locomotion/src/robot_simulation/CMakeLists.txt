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
# find_package(lodepng REQUIRED)

include_directories(
    ${COMMON_INSTALL_DIR}/include
    ${COMMON_INSTALL_DIR}/install/include
    ${lodepng_HEADERS}
    include
)
link_directories(${COMMON_INSTALL_DIR}/lib)

if(INSTALL_HEADERS_ONLY)
if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    add_definitions(-DUSE_ROS_COMM)
endif()
install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)

else()
add_executable(simulate src/sim_main.cpp)

if ($ENV{USE_COMM_TYPE} STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
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
if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    target_link_libraries(simulate
        dds_comm
    )    
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    target_link_libraries(simulate
        ros_comm
    )
endif()
install(TARGETS simulate
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)

# add_library(platform_ui_adapter)
# target_sources(platform_ui_adapter
#     PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
# )

add_executable(simulate_pv 
    src/sim_passive_viewer.cpp 
    src/simulate.cc
    src/glfw_adapter.cc
    src/glfw_dispatch.cc
    src/platform_ui_adapter.cc
    src/lodepng.cpp
)
# if (APPLE)
#     target_sources(simulate_pv
#         # PUBLIC include/glfw_corevideo.h
#         PRIVATE src/glfw_corevideo.mm src/macos_gui.mm
#     )
# endif()
if ($ENV{USE_COMM_TYPE} STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
add_definitions(-DUSE_ROS_COMM)
endif()
add_dependencies(simulate_pv quad_communication)
target_link_libraries(simulate_pv
    # mujoco::mujoco
    mujoco
    glfw
    Threads::Threads
    quad_communication
)
if ($ENV{USE_COMM_TYPE} STREQUAL "dds")
    target_link_libraries(simulate_pv
        dds_comm
    )
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    target_link_libraries(simulate_pv
    ros_comm
    )
endif()
install(TARGETS simulate_pv
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)
endif()