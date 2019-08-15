#!/bin/bash

if [[ $UID != 0 ]]; then
	echo "Please run this as root"
	exit 1
fi

# Needed for UUV Simulator
apt-get install protobuf-compiler protobuf-c-compiler

cd ~/osu-uwrt/
mkdir dependencies
cd dependencies
mkdir src
cd src
git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
git clone --recursive https://github.com/osu-uwrt/darknet_ros.git
git clone https://github.com/osu-uwrt/nortek_dvl.git
git clone https://github.com/uuvsimulator/uuv_simulator.git
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install