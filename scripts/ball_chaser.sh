#!/bin/sh
xterm  -e  " cd /home/workspace/catkin_ws; source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 8
xterm  -e  " cd /home/workspace/catkin_ws; source devel/setup.bash; roslaunch ball_chaser ball_chaser.launch"
