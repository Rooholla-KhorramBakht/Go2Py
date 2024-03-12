if($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
endif()

if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    find_package(CycloneDDS-CXX REQUIRED)
endif()

find_package(Threads REQUIRED)

include_directories(
    # ${COMMON_INSTALL_DIR}/include/ddscxx
    msgs
    include
    ${COMMON_INSTALL_DIR}/include
)
if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    # include_directories(${COMMON_INSTALL_DIR}/include/ddscxx)
    include_directories(${CycloneDDS-CXX_INCLUDE_DIR})
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    set(ROS_INCLUDE /opt/ros/$ENV{ROS_DISTRO}/include)
    include_directories(${ROS_INCLUDE})

    # find_package(catkin REQUIRED)
    # find_package(genmsg REQUIRED)

    find_package(rosidl_default_generators REQUIRED)

    # Specify that your package belongs to the rosidl_interface_packages group
    # set(ROSIDL_INTERFACE_PACKAGES_GROUP "rosidl_interface_packages" CACHE INTERNAL "")
    # set_property(GLOBAL APPEND PROPERTY ${ROSIDL_INTERFACE_PACKAGES_GROUP} ${PROJECT_NAME})

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msgs/ros/LowCmd.msg"
    )
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

    add_custom_target(${PROJECT_NAME}_ros_msgs ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp/${PROJECT_NAME}
    )
    target_link_libraries(${PROJECT_NAME}_ros_msgs "${cpp_typesupport_target}") 
    # install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp/${PROJECT_NAME}
    #         DESTINATION ${COMMON_INCLUDE_DIR}
    # )
endif()

if(INSTALL_HEADERS_ONLY)
    install(DIRECTORY include/
        DESTINATION ${COMMON_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h*"
    )
    # install(DIRECTORY msgs/
    #     DESTINATION ${COMMON_INCLUDE_DIR}
    #     FILES_MATCHING PATTERN "*.h*"
    # )
else()

    add_library(quad_communication SHARED
        src/CommunicationManager.cpp
        src/SHM.cpp
    )
    install(TARGETS quad_communication
        ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
        LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
    )

    if ($ENV{USE_COMM_TYPE} STREQUAL "dds")
        idlcxx_generate(TARGET quad_comm_msgs FILES msgs/idl/Imu.idl msgs/idl/LowCmd.idl msgs/idl/Go2pyLowCmd.idl msgs/idl/Go2pyLowState.idl msgs/idl/LowState.idl WARNINGS no-implicit-extensibility)
        add_library(dds_comm SHARED
            src/QuadDDSComm.cpp
        )
        # target_link_options(dds_comm PRIVATE "-Wl,-rpath,${COMMON_INSTALL_DIR}/lib")
        target_link_libraries(dds_comm
            # ${CycloneDDS-CXX_LIBRARIES}
            quad_comm_msgs
            CycloneDDS-CXX::ddscxx
            quad_communication
            Threads::Threads
        )
        install(TARGETS
            dds_comm
            ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
            LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
        )

        # add_executable(test_dds_comm
        #     tests/test_dds_comm.cpp
        # )
        # target_link_libraries(test_dds_comm
        #     dds_comm
        # )
        # install(TARGETS
        #     test_dds_comm
        #     ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
        #     RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
        # )
    endif()
    
    if($ENV{USE_COMM_TYPE} STREQUAL "ros2")
        add_library(ros_comm SHARED
        src/QuadROSComm.cpp
        )
        target_link_libraries(ros_comm
            ${rclcpp_LIBRARIES}
            ${sensor_msgs_LIBRARIES}
            ${nav_msgs_LIBRARIES}
            ${geometry_msgs_LIBRARIES}
            ${std_msgs_LIBRARIES}
            quad_communication
            Threads::Threads
        )
        install(TARGETS
            ros_comm
            ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
            LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
        )
    
        # add_executable(test_ros_comm
        #     tests/test_ros_comm.cpp
        # )
        # target_link_libraries(test_ros_comm
        #     ros_comm
        # )
        # install(TARGETS
        #     test_ros_comm
        #     ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
        #     RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
        # )
    endif()
endif()