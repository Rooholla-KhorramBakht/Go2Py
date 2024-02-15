include_directories(
    include
    ${COMMON_INCLUDE_DIR}
)

install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)

add_library(pd_controller
    src/Controller/Controller.cpp
)
target_link_libraries(pd_controller
    quad_dynamics
)
add_library(didc
    src/Controller/DistributedIDC.cpp
)
target_link_libraries(didc
    quad_dynamics
    pd_controller
)

add_library(quad_estimator
    src/Estimator/Estimator.cpp
    src/Estimator/QuadEstimator.cpp
)
target_link_libraries(quad_estimator
    quad_dynamics
)

add_library(base_planner
    src/Planner/Planner.cpp
    src/Planner/Gait.cpp
    src/Planner/FootSwingTrajectory.cpp
)
target_link_libraries(base_planner
    quad_dynamics
)
add_library(reactive_planner
    src/Planner/ReactivePlanner.cpp
)
target_link_libraries(reactive_planner
    base_planner
)
install(TARGETS 
    pd_controller
    didc
    quad_estimator
    base_planner
    reactive_planner
    ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
    LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
)

add_executable(quad_execute src/execute.cpp src/Executor.cpp)
add_dependencies(quad_execute
    # quad_communication
    ros_comm
    # pd_controller
    didc
    quad_dynamics
    quad_estimator
    # base_planner
    reactive_planner
)
target_link_libraries(quad_execute
    # quad_communication
    ros_comm
    # pd_controller
    didc
    quad_dynamics
    quad_estimator
    base_planner
    reactive_planner
)
install(TARGETS quad_execute
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)

add_executable(execute_shm src/execute_shm.cpp src/ExecutorSHM.cpp)
add_dependencies(execute_shm
    quad_communication
    # ros_comm
    # pd_controller
    didc
    # quad_dynamics
    quad_estimator
    # base_planner
    reactive_planner
)
target_link_libraries(execute_shm
    quad_communication
    # ros_comm
    # pd_controller
    didc
    # quad_dynamics
    quad_estimator
    # base_planner
    reactive_planner
)
install(TARGETS execute_shm
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)