#!/bin/bash
set -e

REQUIRED_DIRS=(
  "$HOME/go2-devcontainer/logging/coredump"
  "$HOME/go2-devcontainer/logging/logs"
  "$HOME/go2-devcontainer/logging/data"
  "$HOME/go2-devcontainer/logging/plan/data/scene"
  "$HOME/go2-devcontainer/logging/plan/data/plan_results"
  "$HOME/go2-devcontainer/models/yolov8"
)

for dir in "${REQUIRED_DIRS[@]}"; do
  if [ ! -d "$dir" ]; then
    echo "Creating directory $dir"
    mkdir -p $dir
  fi
done

if [ ! -f "$HOME/go2-devcontainer/.bash_history" ]; then
  echo "Creating file $HOME/go2-devcontainer/.bash_history"
  touch $HOME/go2-devcontainer/.bash_history
fi