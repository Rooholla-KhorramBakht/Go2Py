##############################################
## Bash Script
##############################################

PROMPT_COMMAND='history -a'
HISTFILE=/home/dev/.bash_history
LOG_WORKSPACE=/workspace/logging
ROS_WORKSPACE=/workspace/go2-devcontainer

export CC=clang
export CXX=clang++
export MAKEFLAGS="-j 8"

export RCUTILS_COLORIZED_OUTPUT=1
export ROS_LOG_DIR=/workspace/logging/logs
export ROS_DATA_DIR=/workspace/logging/data
export ROS_BAG_MAX_SIZE=1073741824
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}:{line_number}]: {message}"

# unitree_ros2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp118s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

# https://github.com/ament/ament_cmake/issues/382
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
export PYTHONWARNINGS

echo '/workspace/logging/coredump/core.%e.%p.%h.%t' | sudo tee /proc/sys/kernel/core_pattern >/dev/null

_show_workspace_logging_usage() {
  # For each dir under logging workspace, show disk usages.
  echo "==================================================="
  echo "Disk usage for each directory under $LOG_WORKSPACE:"
  for dir in $(find $LOG_WORKSPACE -maxdepth 1 -mindepth 1 -type d); do
    du -sh $dir
  done
  echo "Make sure to clean up old logs to save disk space."
  echo "==================================================="
}

_update_dependencies() {
  source /opt/ros/humble/setup.bash
  sudo apt-get update
  cd $ROS_WORKSPACE \
      && rosdep update \
      && rosdep install --from-paths src --ignore-src -y
}

_ensure_not_in_workspace_src() {
  # If current dir ends with "src", prompt user to switch to workspace dir.
  # colcon build will stupidly build inside src folder.
  if [[ "$(basename "$(pwd)")" == "src" ]]; then
    echo "It looks like you're in a 'src' directory. Please switch to ${ROS_WORKSPACE}."
    return 1
  fi
  return 0
}

COMMON_CMAKE_ARGS="\
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -GNinja \
    -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld \
    -DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=lld \
    -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld
    "
COMMON_COLCON_ARGS="--symlink-install --event-handlers console_cohesion+"
RELEASE_CMAKE_ARGS="$COMMON_CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo"
DEBUG_CMAKE_ARGS="$COMMON_CMAKE_ARGS -DCMAKE_BUILD_TYPE=Debug"

_colcon_release_build() {
  _ensure_not_in_workspace_src || return 1

  # Run 'colcon build' with passed arguments
  colcon build \
      ${COMMON_COLCON_ARGS} \
      --cmake-args $RELEASE_CMAKE_ARGS \
      "$@"
}

_colcon_debug_build() {
  _ensure_not_in_workspace_src || return 1

  # Run 'colcon build' with passed arguments
  colcon build \
      ${COMMON_COLCON_ARGS} \
      --cmake-args $DEBUG_CMAKE_ARGS \
      "$@"
}

_colcon_test() {
  _ensure_not_in_workspace_src || return 1

  # Run 'colcon test' with passed arguments
  colcon test --event-handlers console_cohesion+ "$@"
}

_remove_directories() {
  if [ -d "build" ]; then
    rm -r build
  fi
  if [ -d "install" ]; then
    rm -r install
  fi
  if [ -d "log" ]; then
    rm -r log
  fi
}

colcon_clean() {
  _ensure_not_in_workspace_src || return 1

  if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
    while true; do
      read -p "Do you wish to remove build, install, and log directories? [y/n] " yn
      case $yn in
        [Yy]* ) _remove_directories; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
      esac
    done
  fi
}

# Colcon build aliases
alias update_dep=_update_dependencies
alias source_ws='source install/setup.bash'
alias colcon_build='_colcon_release_build'
alias colcon_debug_build='_colcon_debug_build'
alias colcon_build_package='_colcon_release_build --packages-select'
alias colcon_debug_build_package='_colcon_debug_build --packages-select'
alias colcon_build_up_to='_colcon_release_build --packages-up-to'
alias colcon_debug_build_up_to='_colcon_debug_build --packages-up-to'
alias colcon_test='_colcon_test'
alias colcon_test_package='_colcon_test --packages-select'
alias colcon_test_up_to='_colcon_test --packages-up-to'

source /opt/ros/humble/setup.bash
# Source ROS workspace if it exists
if [ -f "/workspace/go2-devcontainer/install/setup.bash" ]; then
    source /workspace/go2-devcontainer/install/setup.bash
fi

# colcon_cd
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/

_show_workspace_logging_usage