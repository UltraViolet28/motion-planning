#!/usr/bin/env python
import rospy


from nav_msgs.msg import Path
from  nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import priority_dict
import time


# The Map
map = OccupancyGrid()

# Ground Truth pose
start = Odometry()
# 2D nav Goal
goal = PoseStamped()  

# path
path = Path()


def map_info(msg):
    global map
    map = msg

def goal_info(msg):
    global goal
    goal = msg

def start_info(msg):
    global start
    start = msg

def make_map(occupancy_map):
    map_arr=np.array(occupancy_map.data)
    #print(map_list)
    map = np.reshape(map_arr,(3328,3328))
    #print(map_table)
    return map


if __name__ == '__main__':
    try:

        rospy.init_node("path_find",anonymous=True)


        # Subscribe map 
        #rospy.wait_for_message("/map",OccupancyGrid)
        map_sub = rospy.Subscriber("/map",OccupancyGrid,map_info)

        # Subscribe ground state position of robot
        #rospy.wait_for_message("/ground_truth/state",Odometry)
        start_sub=rospy.Subscriber("/ground_truth/state",Odometry,start_info)

        # Subscribe to 2D nav goal topic
        #rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        start_sub=rospy.Subscriber("/move_base_simple/goal",PoseStamped,goal_info)
            
        path_pub=rospy.Publisher("/NavfnROS/plan",Path,queue_size=10)
            
        time.sleep(0.5)

        while not rospy.is_shutdown():

            occcupancy_grid = make_map(map)
            print(occcupancy_grid.shape)

            # Start Position
            xi = int((start.pose.pose.position.x + 50.010000)/0.03)
            yi = int((start.pose.pose.position.y + 50.010000)/0.03)

            start_element = (xi,yi)
            print("start",start_element)

            # Goal position
            xf = int((goal.pose.position.x + 50.010000)/0.03)
            yf = int((goal.pose.position.y + 50.010000)/0.03)

            goal_element = (xf,yf)
            print("goal",goal_element)
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")