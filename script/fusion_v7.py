#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow
#rosbag filter input.bag output.bag "t.secs >= 1531425960 and t.secs <= 1531426140"

from __future__ import print_function
from KF_library import KF
# ROS related imports
import rospy
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import roslib
import math
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ti_mmwave_rospkg.msg import RadarScan
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import Float32MultiArray
import time
from geometry_msgs.msg import PoseStamped
import keyboard
import tf as transferfunction
import threading
import os
import numpy as np

from deepsort_fusion import detector_tracker

def radar_input(point):#radar update
    filtered_radar=np.array([[round(point.markers[0].pose.position.y,3),round(point.markers[0].pose.position.x,3)],
                             [round(point.markers[1].pose.position.y,3),round(point.markers[1].pose.position.x,3)],
                             [round(point.markers[2].pose.position.y,3),round(point.markers[2].pose.position.x,3)],
                             [round(point.markers[3].pose.position.y,3),round(point.markers[3].pose.position.x,3)],
                             [round(point.markers[4].pose.position.y,3),round(point.markers[4].pose.position.x,3)],
                             [round(point.markers[5].pose.position.y,3),round(point.markers[5].pose.position.x,3)]])
    #print (filtered_radar)
prev_received_image=0
def image_cb(frame):
    if(time.time()-prev_received_image)>0.11:
        print("calling image tracker")
        bridge = CvBridge()
        
        cv_image = bridge.imgmsg_to_cv2(frame, "bgr8")            
        
        image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
        image_tracker.image_callback(image)

def main(args):
    global pose_pub,kf_marker,radar_kf,m_array,image_tracker
    rospy.init_node('detector_node')
    #pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)
    #kf_marker= rospy.Publisher('kf_marker', MarkerArray, queue_size=1)
    m_array=MarkerArray()
    #rospy.Subscriber("/ti_mmwave/radar_scan",  RadarScan, callback2)
    rospy.Subscriber("/viz",MarkerArray,radar_input)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_cb, queue_size=1, buff_size=2**24)
    #radar_kf=KF()
    #kf_thread = threading.Thread(target=kf_predict)
    #kf_thread.start()

    #key_thread = threading.Thread(target=keyboard_pressed)
    #key_thread.start()
    image_tracker=detector_tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)