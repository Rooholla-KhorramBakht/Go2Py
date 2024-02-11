#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hesai_ros_driver::hesai_ros_driver__rosidl_generator_c" for configuration "Release"
set_property(TARGET hesai_ros_driver::hesai_ros_driver__rosidl_generator_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hesai_ros_driver::hesai_ros_driver__rosidl_generator_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhesai_ros_driver__rosidl_generator_c.so"
  IMPORTED_SONAME_RELEASE "libhesai_ros_driver__rosidl_generator_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hesai_ros_driver::hesai_ros_driver__rosidl_generator_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_hesai_ros_driver::hesai_ros_driver__rosidl_generator_c "${_IMPORT_PREFIX}/lib/libhesai_ros_driver__rosidl_generator_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
