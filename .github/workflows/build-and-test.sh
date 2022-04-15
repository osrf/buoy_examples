#!/bin/bash
set -ev

export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

mkdir -p $COLCON_WS_SRC

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
apt-get install -y git \
                   python3-colcon-common-extensions \
                   python3-rosdep

cd $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC

rosdep init
rosdep update
rosdep install --from-paths $COLCON_WS_SRC -i -y --rosdistro $ROS_DISTRO

# Build
source /opt/ros/$ROS_DISTRO/setup.bash
cd $COLCON_WS
colcon build --event-handlers console_direct+

# Test just buoy_examples
colcon test --packages-select buoy_examples --event-handlers console_direct+
colcon test-result
