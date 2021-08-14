#!/usr/bin/python

import rospy
import cv2
import imutils 
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32 
from std_msgs.msg import Int16
from sensor_msgs.msg import Image

class Detector:
    """
    Class to detect humans in images
    """
    
    def __init__(self):
    
        self.hog = cv2.HOGDescriptor() 
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector()) 
        
        self.bridge = CvBridge()
        
        self.cam_rgb = 0
        self.cam_depth = 0
        self.width_min = 300
        
        self.x_rec = 0
        self.y_rec = 0
        self.w_rec = 0
        self.h_rec = 0
        
        self.human_center = np.array([0, 0]) # x,y coordinates of human center
        self.human_distance = 0.0 # depth distance to human [meters]
        self.human_lateral = 0 # lateral deviation to human [pixels]
        
        self.sub_cam_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_cam_rgb)
        self.sub_cam_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback_cam_depth)
        
        self.pub_human_distance = rospy.Publisher("/rycsv/human_distance", Float32, queue_size=10)
        self.pub_human_lateral = rospy.Publisher("/rycsv/human_lateral", Int16, queue_size=10)
        self.pub_img_detector = rospy.Publisher("/camera/rgb/img_detector", Image, queue_size=10)
        
        self.human_lateral_msg = Int16()
        self.human_distance_msg = Float32()
        
        
    def callback_cam_rgb(self,data):
        """
        Updates rgb image and lateral deviation
        """
        try:
            self.cam_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8") 
            self.cam_rgb = self.detect_human(self.cam_rgb)
            self.update_human_lateral()
            img_person_msg = self.bridge.cv2_to_imgmsg(self.cam_rgb, "bgr8")
            self.pub_img_detector.publish(img_person_msg)
            
        except CvBridgeError as e:
            print(e)
            
                    
    def callback_cam_depth(self,data):
        """
        Updates depth image and distance to human
        """
        try:
            self.cam_depth = self.bridge.imgmsg_to_cv2(data) 
            self.update_human_distance()
            
        except CvBridgeError as e:
            print(e)
            
            
    def resize_img(self, image, width_min):
        return imutils.resize(image, width=min(width_min, image.shape[1])) 


    def detect_human(self,image):
        """
        Detect human in image
        """
        # Detect human
        (regions, _) = self.hog.detectMultiScale(image, winStride=(8, 8), padding=(8, 8), scale=1.05) 
        
        if len(regions) != 0:
            
            for (x_rec, y_rec, w_rec, h_rec) in regions: 
                
                self.x_rec = x_rec
                self.y_rec = y_rec
                self.w_rec = w_rec
                self.h_rec = h_rec
                
            # Draw rectangle that encloses the human
            cv2.rectangle(image, (self.x_rec,self.y_rec), 
                                    (self.x_rec+self.w_rec,self.y_rec+self.h_rec), (0,0,255), 2) 
            # Draw green point that indicates lateral deviation
            cv2.circle(image, (int(self.x_rec+self.w_rec/2),int(self.y_rec+self.h_rec/3)), 
                                radius=2, color=(0, 255, 0), thickness=-1)
            # Compute x,y coordinates of human center
            self.human_center = np.array([int(self.y_rec+self.h_rec/3), int(self.x_rec+self.w_rec/2)])
        
        else:
            self.human_center = np.array([0, 0])
        
        return image
    
    
    def update_human_distance(self):
        """
        Computes distance to human [meters]
        """
        if self.human_center[0]!=0 and self.human_center[1]!=0:
            self.human_distance = self.cam_depth[self.human_center[0], self.human_center[1]]
        else:
            self.human_distance = 0.0
            
        self.human_distance_msg.data = self.human_distance
        self.pub_human_distance.publish(self.human_distance_msg)
    
    
    def update_human_lateral(self):
        """
        Computes lateral deviation [pixels]
        """
        w_img = self.cam_rgb.shape[1]
        
        if self.human_center[0]!=0 and self.human_center[1]!=0:
            self.human_lateral = int(w_img/2-self.human_center[1])
        else:
            self.human_lateral  = 0.0
            
        self.human_lateral_msg.data = self.human_lateral
        self.pub_human_lateral.publish(self.human_lateral_msg)
            