#!/bin/bash

if [ $_ = $0 ]
then
  echo 'This script should not be executed as program'
  echo 'Please, source it'
  echo
  echo 'Example:'
  echo "$ source ${BASH_SOURCE[0]}"
  echo
  exit 1
fi

DEF_PATH="$HOME/catkin_ws"
ORIG_PATH=$( pwd )

echo
echo 'Starting ros-pacman server...'
echo

if pgrep -x "roscore" > /dev/null
then
  echo "'roscore' process found, please close it."
  echo
  return
fi

echo 'Type the Pacman workspace path:'
read -p "[default (Enter): ${DEF_PATH}]: " WS_PATH
if [ -z "$WS_PATH" ]
then
  WS_PATH=$DEF_PATH
fi

echo

if [ -d "$WS_PATH" ]; then
  echo "Folder '$WS_PATH' found"
else
  echo "Folder '$WS_PATH' not found"
  echo "Aborting"
  return
fi

echo

export ROS_IP=$( hostname -I )
export ROS_IP=${ROS_IP//[[:blank:]]/}

echo 'Server ready.'
echo "Use the IP address ${ROS_IP} in your Pacman Controller"
read -p "Press Enter to start roscore and the Pacman World"
echo

gnome-terminal -e 'roscore'

export ROS_MASTER_URI="http://${ROS_IP}:11311"
export ROS_MASTER_URI=${ROS_MASTER_URI//[[:blank:]]/}

cd $WS_PATH
source devel/setup.bash
rosrun pacman pacman_world game

cd $ORIG_PATH
