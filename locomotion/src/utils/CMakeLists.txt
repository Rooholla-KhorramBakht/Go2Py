# CMakeLists.txt in src/utils
# file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h*")

# Install header files
# install(FILES ${HEADER_FILES} DESTINATION ${COMMON_INCLUDE_DIR})
if(INSTALL_HEADERS_ONLY)
install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)
endif()