#!/usr/bin/python

import numpy as np

class Kobuki:
    """
    Class to define Kobuki mobile robot parameters
    Not used in current version of rycsv_pkg ROS Package
    """

    def __init__(self):

        # Wheels parameters
        radius = 0.035
        alpha_l = np.pi/2 # left
        beta_l  = 0
        alpha_r = -np.pi/2 # right
        beta_r  = np.pi
        l = 0.23/2 # 2l = distance between wheels

        # J1 and J2 matrixes
        J1 = np.array([(np.sin(alpha_r+beta_r), -np.cos(alpha_r + beta_r), -l*np.cos(beta_r)),
                       (np.sin(alpha_l+beta_l), -np.cos(alpha_l + beta_l), -l*np.cos(beta_l)),
                       (np.cos(alpha_r+beta_r), np.sin(alpha_r + beta_r), l*np.sin(beta_r))])

        J2 = radius*np.identity(3)

        #Jacob_inv
        self.jacob_inv = np.matmul(np.linalg.pinv(J2),J1)


    def wheel_velocities(self,Vx,Vy,Wz):
        '''
        Calculate angular velocity [rad/s] of the wheels 
        from linear and angular velocities of the chassis
        '''
        vel = np.array([(Vx),(Vy),(Wz)])
        vel_wheels = np.matmul(self.jacob_inv, vel)

        return vel_wheels