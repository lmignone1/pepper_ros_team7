#!/bin/bash
catkin create pkg $1 --catkin-deps std_msgs message_generation rospy rosconsole
