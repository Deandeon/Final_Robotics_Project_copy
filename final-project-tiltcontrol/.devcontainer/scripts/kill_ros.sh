#!/bin/bash

#
# File: kill_ros.sh
# Project: scripts
# File Created: Tuesday, 27th January 2026 7:32:55 PM
# Author: nknab
# Email: kboateng@ashesi.edu.gh
# Version: 1.0.0
# Brief: Kill all ROS and Gazebo processes
# -----
# Last Modified: Tuesday, 27th January 2026 6:33:49 PM
# Modified By: Eyiram Gaze
# -----
# Copyright ©2026 nknab
#



ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
echo "All ROS Processes Killed"

ps aux | grep gazebo | grep -v grep | awk '{ print "kill -9", $2 }' | sh
ps aux | grep gz | grep -v grep | awk '{ print "kill -9", $2 }' | sh
echo "All Gazebo Processes Killed"
