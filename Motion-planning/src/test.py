import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from math import atan2,sqrt
import math
from nav_msgs.msg import Path
import time
from tf.transformations import euler_from_quaternion

import numpy as np


#---------------------------------------------------------------

x = 0.0
y = 0.0 
theta = 0.0
route = []

#---------------------------------------------------------------
def call_back(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

#---------------------------------------------------------------

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

#---------------------------------------------------------------

def dist(a,b):
    return(sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2))

def normalizeAngle(angle):
        while angle < -np.pi:
            angle += 2* np.pi
        while angle > np.pi:
            angle -= 2* np.pi
        return angle




#---------------------------------------------------------------

if __name__ == '__main__':
    try:
        
        rospy.init_node('sahayak_teleop_node', anonymous=True)
        print('Commencing Operation')

        #---------------------------------------------------------------
        # # Location Of robot
        # start = rospy.Subscriber("/ground_truth/state",Odometry, call_back)

        # res= 0.030054
        # # converting real-time coordinates to occupancy grid indices 
        # col1, row1= int((x +50.01)/res) , int((y +50.01)/res)
        # origin = (row1, col1)

        #---------------------------------------------------------------
        # Path
        path =  rospy.wait_for_message("/global_plan/rdp", Path)
        print(len(path.poses))


        # Storing path Points as (row,col) in list
        path1 = []
        #res= 0.030054
        for i in range(len(path.poses)):
            xpt = path.poses[i].pose.position.x
            ypt = path.poses[i].pose.position.y

            path1.append((xpt,ypt))
        print("Path Loaded")
        print(path1[1])

            
        #---------------------------------------------------------------
        #declare velocity publisher
        vel_topic_1='/joint1_vel_controller/command'
        velocity_publisher_1 = rospy.Publisher(vel_topic_1, Float64, queue_size=10)

        vel_topic_2='/joint2_vel_controller/command'
        velocity_publisher_2 = rospy.Publisher(vel_topic_2, Float64, queue_size=10)

        vel_topic_3='/joint3_vel_controller/command'
        velocity_publisher_3 = rospy.Publisher(vel_topic_3, Float64, queue_size=10)

        vel_topic_4='/joint4_vel_controller/command'
        velocity_publisher_4 = rospy.Publisher(vel_topic_4, Float64, queue_size=10)

        rate= rospy.Rate(5)  

        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        #---------------------------------------------------------------


        R= 0.08
        L= 0.41

        point_path = 2


        z = 1
        old_error = 0
        old_phi = 0
        print('Loop Started')
        while not rospy.is_shutdown():

            #---------------------------------------------------------------
            # Location Of robot
            start = rospy.Subscriber("/ground_truth/state",Odometry, call_back)

            # res= 0.030054
            # converting real-time coordinates to occupancy grid indices 
            # col1, row1= int((x +50.01)/res) , int((y +50.01)/res)
            # origin = (row1, col1)
            origin = (x,y)
            #---------------------------------------------------------------

            
            #time.sleep(2)

            path_pose = path1[point_path]
            print("Current Goal : ",path_pose)


            bot_pose = origin # (row , col)
            print("Current Pose",origin)
            dis = dist(bot_pose , path_pose)
            print("dist ",dis)

            inc_x = path_pose[0] - bot_pose[0] 
            inc_y = path_pose[1] - bot_pose[1]

            error = sqrt(inc_x**2 + inc_y**2)
            phi = atan2(inc_y,inc_x) - theta - np.pi/2 + 0.30
            print("Angle ",phi)

            # phi =  atan2(y2-y1,x2-x1) - theta - np.pi/2 + 0.30
            # phi = normalizeAngle(phi)
            # # print('Angle', phi)
            # print('Angle', phi, '\t', 'Dist',error)
            # # differential term
            D_err = error - old_error
            D_phi = phi - old_phi

            # Controller #Not Tuned

            v_des = 0.4*error + 0.1*D_err
            w_des = 5*(phi) - 2.3*D_phi

            # update old errors
            old_error = error
            old_phi = phi

            v_r = v_des + L*w_des
            v_l = v_des - L*w_des
            v_l = v_l/R
            v_r = v_r/R

                
                # Stop Condition
            if abs(phi)< 0.0005 and (abs(error)< 0.1 or (abs(error)<0.23 and D_err > 0)): 

                v_l = 0
                v_r = 0
                print('done')
                #break

            # Select next node
            # if dis_final < 0.5: 

            #     v_l = 0
            #     v_r = 0
            #     break
                # point_path = point_path + int(len(path1)/5)
                # # Stop Condition
                # if point_path > len(path1): 
                #     point_path = len(path1) - 1
                #     v_l = 0
                #     v_r = 0
                    
            
            # Giving Velocity to bot
            j1.data = v_r    # Possible bug ##################################
            j2.data = -v_l
            j3.data = v_r
            j4.data = -v_l

            velocity_publisher_1.publish(j1)
            velocity_publisher_2.publish(j2)
            velocity_publisher_3.publish(j3)
            velocity_publisher_4.publish(j4)
            rate.sleep()


            print('Iteration ', z)
            print('\n')
            z = z+1
            

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")




