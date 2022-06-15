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





#------------------------------------------------------------------------
# A* part
def dist(a,b):
    return round(np.sqrt( (a[0] - b[0])**2 + (a[1] - b[1])**2 ),4)

# heuristic function - euclidean distance from goal
def hn(node,goal):
    return(dist(node,goal))

# Get an array of path coordinates
def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
        
    return path

# Get all possible valid child nodes of given node
def child_node(node,map):
    neighbour = {}

    for i in range(-1,2):
        for j in range(-1,2):
            #print(1)

            y = node[0] + i
            x = node[1] + j

            if not((i,j) == (0,0)) and (y in range(map.shape[0]) and x in range(map.shape[1])):
                
                # Map free
                if map[y,x] == 0 and map[y,x] != -1:
                    #map[y,x] = 255
                    neighbour[(y,x)] = dist(node,(y,x))
                # elif map[y,x] == 255:
                #     neighbour[(y,x)] = 999999
    
    return neighbour
#---------------------------------------------------------------------
# The A* Algorithm
def a_star_search(origin_key, goal_key, map):
    
    open_queue = priority_dict.priority_dict({})
    
    # The dictionary of closed vertices we've processed.
    closed_dict = {}
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    
    # The dictionary that stores the best cost to reach each
    # vertex found so far.
    costs = {}
    
    
    # Add the origin to the open queue and the costs dictionary.
    costs[origin_key] = 0.0
    open_queue[origin_key] = hn(origin_key, goal_key)
    #print(open_queue)
    #u = min(open_queue, key = open_queue.get)


    goal_found = False
    #return child_node(u , map)
    while (len(open_queue) != 0):
        u , u_l = open_queue.pop_smallest()
        
        print(u)
        #time.sleep(2)
        u_cost = costs[u]
        # print(open_queue)
        
        
        # if goal is reached break the loop
        if u == goal_key:
            goal_found = True
            break
        
        
        # iterate over the neighbouring vertices
        child_nodes = child_node(u,map)
        #print(child_nodes)
        for edge in child_nodes:
            
            # Get the edge and edgeweight
            v , uv_cost = edge , child_nodes[edge]
            
            # Check if v is in the closed dict
            if v in closed_dict:
                continue
            
            
            
            if v not in open_queue:
                open_queue[v] = u_cost + uv_cost + hn(v, goal_key)
                costs[v] = u_cost + uv_cost
                predecessors[v] = u
            else:
                v_cost_initial = open_queue[v]
                if v_cost_initial > u_cost + uv_cost + hn(v, goal_key):
                    open_queue[v] = u_cost + uv_cost + hn(v, goal_key)
                    costs[v] = u_cost + uv_cost
                    predecessors[v] = u
        
        # add vertex in closed dict after all possible neighbours are evaluated
        closed_dict[u] = True
        #print(closed_dict)

    if not goal_found:
        
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    #return predecessors
    return get_path(origin_key, goal_key, predecessors)
#--------------------------------------------------------------------------------


# Mapping Part

# resolution: 0.030000
# origin: [-50.010000, -50.010000, 0.000000]  
# elements in map  11075584 ; 3328 X 3328

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

# def maptable(occupancy_map):
#     map_list=occupancy_map.data
#     rows, cols = (occupancy_map.info.height,occupancy_map.info.width )
#     map_table= np.ones((3328,3328))
#     for i in range(rows):
#         for j in range(cols) :
#             map_table[i][j]=map_list[i*cols+j]
#     map_table= cv2.GaussianBlur(map_table,(5,5),0)
#     #print(map_table.shape)
#     return map_table



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
        rospy.wait_for_message("/map",OccupancyGrid)
        map_sub = rospy.Subscriber("/map",OccupancyGrid,map_info)

        # Subscribe ground state position of robot
        #rospy.wait_for_message("/ground_truth/state",Odometry)
        start_sub=rospy.Subscriber("/ground_truth/state",Odometry,start_info)

        # Subscribe to 2D nav goal topic
        #rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        start_sub=rospy.Subscriber("/move_base_simple/goal",PoseStamped,goal_info)
            
        path_pub=rospy.Publisher("/NavfnROS/plan",Path,queue_size=10)
            
        time.sleep(0.5)

        # Goal position
        xf = int((goal.pose.position.x + 50.010000)/0.03)
        yf = int((goal.pose.position.y + 50.010000)/0.03)

        goal_element = (xf,yf)
        print(goal_element)

        while not rospy.is_shutdown():

        # converting map array to Occupancy grid 
            occcupancy_grid = make_map(map)

            # Start Position
            xi = int((start.pose.pose.position.x + 50.010000)/0.03)
            yi = int((start.pose.pose.position.y + 50.010000)/0.03)

            start_element = (xi,yi)
            print(start_element)

            # Goal position
            xf = int((goal.pose.position.x + 50.010000)/0.03)
            yf = int((goal.pose.position.y + 50.010000)/0.03)

            goal_element = (xf,yf)
            print(goal_element)

            path_1 = a_star_search(start_element,goal_element,occcupancy_grid)

            path.header.frame_id="map"
            point_list=[] 
            for i in range(len(path_1)):
                
                col,row=path_1[i]
                x = col*0.030000-50.010000
                y = row*0.030000-50.010000
                
                pos = PoseStamped()
                
                pos.pose.position.x = x
                pos.pose.position.y = y
                
                point_list.append(pos)
                #print(list)
                path.poses = point_list
                #print(path,"Yes")
            
            path_pub.publish(path) 
            print(start_element, goal_element)
            print('Done')


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")







