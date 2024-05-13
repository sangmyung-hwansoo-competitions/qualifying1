#!/bin/bash
echo '프로젝트 설정...'

WORKSPACE=$1

cd $WORKSPACE

pip3 install pygame==2.4.0 pillow==6.2.2

source /opt/ros/noetic/setup.bash
catkin_make

{
  echo ""
  echo "alias cm=\"cd $WORKSPACE && catkin_make\""
  echo "source /opt/ros/noetic/setup.bash"
  echo "source $WORKSPACE/devel/setup.bash"
  echo "export ROS_MASTER_URI=http://localhost:11311"
  echo "export ROS_HOSTNAME=localhost"
} >> ~/.bashrc


echo '프로젝트 설정이 완료되었습니다.'
