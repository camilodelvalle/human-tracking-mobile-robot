#!/usr/bin/python

import rospy
import os
import sys
import numpy as np
from geometry_msgs.msg import Twist
from controller_model import Controller
from communication_model import Communication
 
if __name__ == '__main__':
    
    com = Communication()
    ctr = Controller()  
    
    # Initial Pose (for lab2 and lab4)
    offx, offy, offz, theta = -1, 0, 0, np.pi
    htm = ctr.get_htm(offx, offy, offz, theta)
    
    """
    option = 
        -lab2
        -lab4 px_path py_path
        -project
    """
    option = sys.argv[1]
    
    if option == '-lab2':
        obj_pose = ctr.get_path_lab2()
        
    elif option == '-lab4':
        px_path = sys.argv[2]
        py_path = sys.argv[3]
        
        px = np.loadtxt(sys.path[0] + '/paths/' + px_path, delimiter=',')
        py = np.loadtxt(sys.path[0] + '/paths/' + py_path, delimiter=',')
        
        px = np.flip(px)
        py = np.flip(py)
        
        path_array = np.column_stack((px, py))
        
        obj_pose = ctr.set_path_lab4(path_array)
        
    rospy.init_node('controller_node', anonymous=True)
    rospy.loginfo("Kobuki Controller Node initialized")
    
    rate = rospy.Rate(20)
    
    # Main Loop
    while(not rospy.is_shutdown()):
          
        if option == '-lab2' or option == '-lab4':
            robot_global_pose = com.get_robot_global_pose(htm,theta)
            v,w = ctr.get_speeds_labs(robot_global_pose, obj_pose)
            
        elif option == '-project':
            v,w = ctr.get_speeds_project(com.human_distance, com.human_lateral)
            
        # Publish ROS msg
        msg=Twist()
        msg.linear.x = v
        msg.angular.z = w
        com.pub_velocities.publish(msg)
            
        rate.sleep()
    
