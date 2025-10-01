#!/bin/bash
ROS_DISTRO=humble

# VENV_ACTIVATE_PATH=$HOME/act/bin/activate
# ROS_SETUP_PATH=/opt/ros/$ROS_DISTRO/setup.bash
# WORKSPACE_SETUP_PATH=$HOME/interbotix_ws/install/setup.bash
RECORD_EPISODES="$HOME/interbotix_ws/src/aloha-ros2/scripts/jaka_record_episodes.py"

# source $VENV_ACTIVATE_PATH || exit 1
# source $ROS_SETUP_PATH || exit 1
# source $WORKSPACE_SETUP_PATH || exit 1

print_usage() {
  echo "USAGE:"
  echo "jaka_auto_record.sh task num_episodes ip_robot"
}

nargs="$#"

if [ $nargs -lt 4 ]; then
  echo "Passed incorrect number of arguments"
  print_usage
  exit 1
fi

if [ "$2" -lt 0 ]; then
  echo "# of episodes not valid"
  exit 1
fi

echo "Task: $1"
for (( i=0; i<$2; i++ ))
do
  episode_idx=$(( $4 + $i ))
  echo "Starting episode $i"
  python3 "$RECORD_EPISODES" --task "$1" --ip "$3" --episode_idx "$episode_idx"
  if [ $? -ne 0 ]; then
    echo "Failed to execute command. Returning"
    exit 1
  fi
done
