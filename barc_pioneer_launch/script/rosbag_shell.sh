#!/bin/sh
roslaunch barc_rockin_launch rosbag$1.launch &
sleep 60
rosnode kill /rosbag1
