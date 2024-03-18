find_package(iir REQUIRED)
find_package(Threads REQUIRED)
find_package(qpOASES REQUIRED)

include_directories(
    include
    ${COMMON_INCLUDE_DIR}
    ${qpOASES_INCLUDE_DIR}
)

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

add_library(pd_controller STATIC
    src/Controller/Controller.cpp
)
target_compile_options(pd_controller PRIVATE "-fPIC")
target_link_libraries(pd_controller
    quad_dynamics
    iir::iir
    ${qpOASES_LIBRARY}
)
add_library(didc STATIC
    src/Controller/DistributedIDC.cpp
)
target_compile_options(didc PRIVATE "-fPIC")
target_link_libraries(didc
    quad_dynamics
    pd_controller
)

add_library(quad_estimator SHARED
    src/Estimator/Estimator.cpp
    src/Estimator/QuadEstimator.cpp
)
target_link_libraries(quad_estimator
    quad_dynamics
    iir::iir
)

add_library(base_planner SHARED
    src/Planner/Planner.cpp
    src/Planner/Gait.cpp
    src/Planner/FootSwingTrajectory.cpp
)
target_link_libraries(base_planner
    quad_dynamics
)
add_library(reactive_planner SHARED
    src/Planner/ReactivePlanner.cpp
)
target_link_libraries(reactive_planner
    base_planner
)
install(TARGETS 
    pd_controller
    didc
    # robust_didc
    quad_estimator
    base_planner
    reactive_planner
    ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
    LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
)

add_executable(execute src/execute.cpp src/Executor.cpp)

if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    add_definitions(-DUSE_ROS_COMM)
endif()

add_dependencies(execute
    quad_communication
    # pd_controller
    didc
    # robust_didc
    quad_dynamics
    quad_estimator
    # base_planner
    reactive_planner
)
target_link_libraries(execute
    # pd_controller
    didc
    # robust_didc
    quad_dynamics
    quad_estimator
    base_planner
    reactive_planner
    Threads::Threads
    quad_communication
)
if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    target_link_libraries(execute
        dds_comm
    )
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    target_link_libraries(execute
        ros_comm
    )
endif()

install(TARGETS execute
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)
endif()