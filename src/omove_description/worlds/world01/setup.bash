# Source this file before loading the world in an existing Gazebo/ROS environment.
_world01_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GAZEBO_MODEL_PATH="${_world01_root}/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
export GAZEBO_RESOURCE_PATH="${_world01_root}${GAZEBO_RESOURCE_PATH:+:${GAZEBO_RESOURCE_PATH}}"
unset _world01_root
