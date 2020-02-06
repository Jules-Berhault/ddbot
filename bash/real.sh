#!/bin/bash
roslaunch ddbot $1&
cd ..;
cd logs;
sleep 15;
rosbag record -a;

