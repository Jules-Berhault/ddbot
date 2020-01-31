#!/bin/bash

cd ~/workspaceRos/build;
make;
rviz &
roslaunch ddbot setup.launch

