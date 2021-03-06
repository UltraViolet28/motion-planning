#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 15:01:21 2022

@author: dheeraj
"""

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

#import numpy as np

# changing colour of pixels in the path

def plot(route):
    #Initializing node
    path= Path()
    path.header.frame_id = "map"
    res= 0.030054
    #path= poses
    for i in route:
        row , col = i
        y = res*row - 50.01
        x = res*col - 50.01
        point= PoseStamped()
        point.pose.position.x , point.pose.position.y = x , y

        #add point to route
        path.poses.append(point)


    #rospy.init_node('Path',anonymous=True)
    
    path_topic= 'TrajectoryPlannerROS/global_plan'
    rate = rospy.Rate(5)
    #creating a publisher for path
    path_pub= rospy.Publisher(path_topic, Path, queue_size=10)
    while not rospy.is_shutdown():
        #publishing the topic to rviz
        path_pub.publish(path)
        rate.sleep()