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
echo 'Starting network environment...'
echo

if pgrep -x "roscore" > /dev/null
then
  echo "'roscore' process should not be executed in this computer"
  echo "Aborting"
  return
fi

read -p "Type IP address of server running 'roscore': " IP_ROSCORE
echo

export ROS_IP=$( hostname -I )
export ROS_IP=${ROS_IP//[[:blank:]]/}
export ROS_MASTER_URI="http://${IP_ROSCORE}:11311"
export ROS_MASTER_URI=${ROS_MASTER_URI//[[:blank:]]/}

echo 'Type the ROS NODE workspace path:'
read -p "[default (Enter): ${DEF_PATH}] " WS_PATH
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

echo 'Network environment ready.'
# read -p "Press Enter to start the Pacman controller"
echo

cd $WS_PATH
source devel/setup.bash
#rosrun pacman pacman_controller

cd $ORIG_PATH
