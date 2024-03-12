# set(pinocchio_DIR ${COMMON_INSTALL_DIR}/lib/cmake/pinocchio)
set(CMAKE_PREFIX_PATH ${COMMON_INSTALL_DIR} ${CMAKE_PREFIX_PATH})
# list(APPEND ${COMMON_INSTALL_DIR} ${CMAKE_PREFIX_PATH})

# message("python path: ${PYTHONPATH}")

find_package(pinocchio CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

message("eigen include: ${eigen3_INCLUDE_DIRS}")

include_directories(
    include
    ${COMMON_INCLUDE_DIR}
)

if(INSTALL_HEADERS_ONLY)
install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)
else()

add_library(quad_dynamics SHARED
    src/kinodynamics.cpp
)
target_link_libraries(quad_dynamics
    pinocchio::pinocchio
)
# target_compile_options(quad_dynamics PRIVATE -O3)
install(TARGETS quad_dynamics
    ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
    LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
)

add_executable(test_kinodynamics
    src/test_kinodynamics.cpp
)
target_link_libraries(test_kinodynamics
    quad_dynamics
)
install(TARGETS test_kinodynamics
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)

add_executable(profile_dynamics
    tests/profile_dynamics.cpp
)
# target_compile_options(profile_dynamics PRIVATE -O3)
target_link_libraries(profile_dynamics
    quad_dynamics
)
install(TARGETS profile_dynamics
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)
endif()