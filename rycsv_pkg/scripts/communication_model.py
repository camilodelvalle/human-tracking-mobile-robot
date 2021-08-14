#!/usr/bin/python

import rospy
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Empty 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D 
from cv_bridge import CvBridge, CvBridgeError

class Communication:
    """
    Class to handle communication through topics
    """
    
    def __init__(self):
    
        self.robot_pos = np.zeros([4,1])
        self.robot_pos[3] = 1
        self.robot_yaw = 0

        self.human_distance = 0.0 # depth distance to human [meters]
        self.human_lateral = 0 # lateral deviation to human [pixels]
       
        # ROS Topics
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.pub_velocities = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.pub_pose2d = rospy.Publisher("pose2d", Pose2D, queue_size=10)
        
        try:
            self.sub_human_distance = rospy.Subscriber("/rycsv/human_distance", Float32, self.callback_human_distance)
            self.sub_human_lateral = rospy.Subscriber("/rycsv/human_lateral", Int16, self.callback_human_lateral)
        except:
            print("Deviations topic not found (For RyCSV Project)")
            pass
        
    def callback_odom(self,data):
        """
        Updates robot pose
        """
        self.robot_pos[0] = data.pose.pose.position.x
        self.robot_pos[1] = data.pose.pose.position.y
        self.robot_pos[2] = data.pose.pose.position.z
        
        quat = data.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        _, _, self.robot_yaw = euler_from_quaternion(q)

    def callback_human_distance(self,data):
        self.human_distance = data.data
        
    def callback_human_lateral(self,data):
        self.human_lateral = data.data
        
    def get_human_deviations(self):
        return self.human_distance, self.human_lateral
        
    def get_robot_global_pose(self,htm,theta):
        '''
        Calculates robot global pose (2D)
        htm: Homogeneous Transformation Matrix
        theta: Robot orientation [rad]
        '''
        global_pose = np.zeros([3,1])
        global_pose[0:2] = np.dot(htm,self.robot_pos)[0:2]
        global_pose[2] = self.robot_yaw + theta
        
        if global_pose[2] <= -np.pi:
            global_pose[2] += 2*np.pi
        elif global_pose[2] > np.pi:
            global_pose[2] -= 2*np.pi
            
        pose2d = Pose2D()
        pose2d.x = global_pose[0]
        pose2d.y = global_pose[1]
        pose2d.theta = global_pose[2]
        self.pub_pose2d.publish(pose2d)
        return global_pose
    