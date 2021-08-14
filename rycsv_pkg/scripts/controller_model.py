#!/usr/bin/python

import numpy as np
import sys

class Controller:
    """
    Class for motion control and vision-based control
    """

    def __init__(self):

        self.point = 0
        self.num_points = 0
        
        self.path_array = 0
        self.htm = 0

        # Polar Controller - Labs 2 and 4
        # For stability: kr>0, kb<0, ka-kr>0
        self.kr = 3.0
        self.ka = 3.1
        self.kb = -0.1
        self.v_max = 0.3
        self.w_max = np.pi/3
        
        # Vision-based Controller
        self.kr_sv = 1.5 
        self.ka_sv = 0.006
        self.v_max_sv = 1.5
        self.w_max_sv = np.pi/2
        
        
    def get_path_lab2(self): 
        """
        Path for Lab 2
        """
        self.path_array = np.array([[-3.5, 0],
                                    [-3.5, 3.5], 
                                    [1.5, 3.5], 
                                    [1.5, -1.5], 
                                    [3.5, -1.5],
                                    [3.5, -8.0],
                                    [-2.5, -8.0],
                                    [-2.5, -5.5], 
                                    [1.5, -5.5],
                                    [1.5, -3.5],
                                    [-1.0, -3.5]])
        self.num_points = len(self.path_array)
        return self.path_array


    def set_path_lab4(self, path_array): 
        """
        Path for Lab 4
        """
        self.path_array = path_array
        self.num_points = len(self.path_array)
        return self.path_array

    
    def get_htm(self, offx, offy, offz, theta): 
        """
        Calculates Homogeneous Transformation Matrix
        """
        self.htm = np.array([[np.cos(theta), -np.sin(theta), 0, offx],
                        [np.sin(theta),  np.cos(theta), 0, offy],
                        [            0,              0, 1, offz],
                        [            0,              0, 0,    1]])  
        return self.htm


    def get_speeds_labs(self, robot_global_pose, obj_pose):
        """
        Control Law to compute linear and angular velocities of the robot chassis
        """
        rob_x = robot_global_pose[0]
        rob_y = robot_global_pose[1]
        theta = robot_global_pose[2]
        
        if self.point < self.num_points:
            obj_x = obj_pose[self.point][0]
            obj_y = obj_pose[self.point][1]
        else:
            obj_x = obj_pose[self.num_points-1][0]
            obj_y = obj_pose[self.num_points-1][1]
        
        dx = obj_x-rob_x
        dy = obj_y-rob_y
        
        rho = np.sqrt(dx**2+dy**2)
        alpha = -theta+np.arctan2(dy,dx)
        
        if alpha <= -np.pi:
            alpha += 2*np.pi
        elif alpha > np.pi:
            alpha -= 2*np.pi
        
        beta = -theta-alpha
   
        if rho < 0.1:
            self.point += 1
            
        if self.point < self.num_points:
            v = self.kr*rho
            w = self.ka*alpha+self.kb*beta
        else:
            v = 0
            w = 0
   
        # Saturation
        if v > self.v_max:
            v = self.v_max
        if w > self.w_max:
            w = self.w_max
        if w < -self.w_max:
            w = -self.w_max
        
        return v,w
    
    
    def get_speeds_project(self, human_distance, human_lateral):
        """
        Vision-based Controller
        Control Law to compute linear and angular velocities of the robot chassis
        """
        if human_distance > 2: # meters
            v = self.kr_sv*human_distance
        else:
            v = 0
        
        if np.abs(human_lateral) > 5: # pixels 
            w = self.ka_sv*human_lateral
        else:
            w = 0
            
        # Saturation
        if v > self.v_max_sv:
            v = self.v_max_sv
        if w > self.w_max_sv:
            w = self.w_max_sv
        if w < -self.w_max_sv:
            w = -self.w_max_sv
        
        # Console output
        print("\n")
        print("Distance to human [m]: " + str (np.round(human_distance,1)))
        print("Lateral deviation [pixels]: " + str (human_lateral))
        print("Linear velocity [m/s]: " + str (np.round(v,2)))
        print("Angular velocity [deg/s]: " + str (np.round(np.rad2deg(w),1)))
        
        return v,w

    

   