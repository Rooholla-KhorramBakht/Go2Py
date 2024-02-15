find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
    include
    /opt/ros/foxy/include
)

install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)

add_library(quad_communication
    src/CommunicationManager.cpp
    src/SHM.cpp
)
install(TARGETS quad_communication
    ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
    LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
)

add_library(ros_comm
  src/QuadROSComm.cpp
)
target_link_libraries(ros_comm
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
  quad_communication
)
install(TARGETS
  ros_comm
  ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
  LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
)

add_executable(test_ros_comm
    tests/test_ros_comm.cpp
)
target_link_libraries(test_ros_comm
    ros_comm
)
install(TARGETS
    test_ros_comm
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)