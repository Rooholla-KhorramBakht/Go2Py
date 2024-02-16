cmake_minimum_required(VERSION 3.5)
project(go2_description)
find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
  launch dae config meshes urdf xacro
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()