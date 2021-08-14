#!/usr/bin/python

import rospy
from detector_model import Detector
 
if __name__ == '__main__':
    
    dt = Detector()
    
    rospy.init_node('detector_node', anonymous=True)
    rospy.loginfo("Detector Node initialized")
       
    rate = rospy.Rate(20)
    
    while(not rospy.is_shutdown()):
        rate.sleep()
    