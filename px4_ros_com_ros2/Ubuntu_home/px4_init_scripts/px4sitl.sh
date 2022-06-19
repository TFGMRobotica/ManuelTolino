#!/bin/bash

./QGroundControl.AppImage &

cd ~/PX4-Autopilot/
make px4_sitl_rtps gazebo
