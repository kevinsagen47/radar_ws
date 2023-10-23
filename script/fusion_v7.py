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
filtered_radar=np.array([[0.,0.,0.]])
kf_timer=0
last_radar=0
last_update=0
#CONFIGS
logging=True
image_recognition=True
rate=0.5
tol=5#tolerance
def cartesian_to_uv(cartesian):
    #print(cartesian)
    uv=cartesian.astype(int)
    for i in range(len(cartesian)):
        if cartesian[i][0]!=0 and cartesian[i][1]!=0:
            degree_tracked=math.degrees(math.atan(cartesian[i][0]/cartesian[i][1]))
            uv[i][0]=round(640-(degree_tracked*14.5+pow(degree_tracked,3)*0.0045))
            uv[i][1]=round(50*cartesian[i][0]+350)
    #print (uv)
    return uv.astype(int)
prev_raw_radar=[[0,0],[0,0]]
def radar_input(point):#radar update
    global filtered_radar,raw_radar,tracked_cam_radar,radar_updating,last_radar,prev_raw_radar,rate
    filtered_radar=np.array([[0.,0.,0.]])
    #filtered_radar=[]
    raw_radar=np.array([
        [round(point.markers[0].pose.position.y,3),round(point.markers[0].pose.position.x,3)],
        [round(point.markers[1].pose.position.y,3),round(point.markers[1].pose.position.x,3)],
        [round(point.markers[2].pose.position.y,3),round(point.markers[2].pose.position.x,3)],
        [round(point.markers[3].pose.position.y,3),round(point.markers[3].pose.position.x,3)],
        [round(point.markers[4].pose.position.y,3),round(point.markers[4].pose.position.x,3)],
        [round(point.markers[5].pose.position.y,3),round(point.markers[5].pose.position.x,3)]])
    #print(raw_radar)
    for i in range(6):
        if raw_radar[i][0]!=0. or raw_radar[i][1]!=0.:
            #print(raw_radar[i])
            #'''
            if filtered_radar[0][0]==0. and filtered_radar[0][1]==0.:
                filtered_radar[0]= np.append(raw_radar[i],i)    
            else:
                filtered_radar=np.vstack([filtered_radar,np.append(raw_radar[i],i)])
            #'''
            #filtered_radar.append([raw_radar[i][0],raw_radar[i][1],i])
    #for i in range(len(filtered_radar)):##########################################radar only
    #    print(filtered_radar[i][0],filtered_radar[i][1]," ", end='')
    #print(" ")

    if(tracked_cam_radar[0][1]!=-1 and tracked_cam_radar[1][1]!=-1):
        radar_updating=1
        
        '''
        last_radar_delta=(time.time()-last_radar)#*rate
        #range_speed=(tracked_range-prev_tracked_range)/last_radar_delta#range in meters
        #azimuth_speed=(tracked_azimuth-prev_tracked_azimuth)/last_radar_delta#azimuth in meters
        tracked_radar1=[[raw_radar[tracked_cam_radar[0][1]][0]],#Px
                        [raw_radar[tracked_cam_radar[0][1]][1]],#Py
                        [((raw_radar[tracked_cam_radar[0][1]][0]-prev_raw_radar[0][0])/last_radar_delta)],#Vx
                        [((raw_radar[tracked_cam_radar[0][1]][1]-prev_raw_radar[0][1])/last_radar_delta)]]#Vy
        #print("tracked_radar1",tracked_radar1) 
        tracked_radar2=[[raw_radar[tracked_cam_radar[1][1]][0]],#Px
                        [raw_radar[tracked_cam_radar[1][1]][1]],#Py
                        [((raw_radar[tracked_cam_radar[1][1]][0]-prev_raw_radar[1][0])/last_radar_delta)],#Vx
                        [((raw_radar[tracked_cam_radar[1][1]][1]-prev_raw_radar[1][1])/last_radar_delta)]]#Vy
        #print("tracked_radar2",tracked_radar2)
        x_update1=radar_kf1.update(tracked_radar1,10.)
        x_update2=radar_kf2.update(tracked_radar2,10.)
        prev_raw_radar=[raw_radar[tracked_cam_radar[0][1]],raw_radar[tracked_cam_radar[1][1]]]
        
        publish_object([[x_update1[0][0],x_update1[1][0]],[x_update2[0][0],x_update2[1][0]]])
        '''
        publish_object([raw_radar[tracked_cam_radar[0][1]],raw_radar[tracked_cam_radar[1][1]]])
        
        print(raw_radar[tracked_cam_radar[0][1]][0],raw_radar[tracked_cam_radar[0][1]][1],#radar camera
                raw_radar[tracked_cam_radar[1][1]][0],raw_radar[tracked_cam_radar[1][1]][1])
        #if logging:
            #print("ori",raw_radar[tracked_cam_radar[0][1]][0],raw_radar[tracked_cam_radar[0][1]][1],
            #    raw_radar[tracked_cam_radar[1][1]][0],raw_radar[tracked_cam_radar[1][1]][1])
            #print(x_update1[0][0],x_update1[1][0],
            #        x_update2[0][0],x_update2[1][0])
            #for i in range(len(filtered_radar)):
            #    print(filtered_radar[i][0],filtered_radar[i][1]," ", end='')
            #print(" ")
        last_radar=time.time()
        radar_updating=0
    
time_since_start=time.time()
prev_received_image=0
radar_updating=0
tracked_cam_radar=[[-1,-1,0,-1],[-1,-1,0,-1]]
check_running=0
#[cam ID, radar ID,count not radar in range,prev locked radar]
def image_cb(frame):
    global prev_received_image,tracked_cam_radar,check_running
    if((time.time()-prev_received_image)>0.25 ):
        check_running=1
        prev_received_image=time.time()
        #print("calling image tracker")
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(frame, "bgr8")            

        image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
        current_filtered_radar=filtered_radar
        current_raw_radar=raw_radar
        radar_uv=cartesian_to_uv(current_filtered_radar)
        tracked_bboxes=image_tracker.image_callback(image,radar_uv)


        '''
        #radar_in_bbox1=np.array([[0,0]])
        #radar_in_bbox2=np.array([[0,0]])
        for i in range(len(tracked_bboxes)):
            #print (tracked_bboxes[i])
            for j in range(len(radar_uv)):
                if radar_uv[j][0]>tracked_bboxes[i][0] and radar_uv[j][0]<tracked_bboxes[i][2]:
                    #print(radar_uv[j][2],"is locked with",tracked_bboxes[i][4]," ", end='')
                    break
        '''
        ###############################FIRST ID PAIRING############################
        if len(tracked_bboxes)==2:
            #check if the first tracked bbox ID is in the paired array
            if(tracked_cam_radar[0][0]!=tracked_bboxes[0][4] or tracked_cam_radar[0][1]==-1):# and tracked_cam_radar[0][0]!=tracked_bboxes[1][4]):
                #if no, search for a pair
                for j in range(len(radar_uv)):
                    #if radar is within range, check if radar is locked with another bbox
                    if (radar_uv[j][0]>(tracked_bboxes[0][0]-tol) and radar_uv[j][0]<(tracked_bboxes[0][2]+tol)):
                        if(radar_uv[j][2]!=tracked_cam_radar[1][1]):

                            if(tracked_cam_radar[1][0]!=tracked_bboxes[0][4]):#check if another bbox is assigned to the first radar
                                
                                if(tracked_cam_radar[1][1]==-1 and tracked_cam_radar[0][1]==-1 and radar_uv[j][2]!=tracked_cam_radar[0][3]):#when both -1, both must have switched
                                    tracked_cam_radar[1][1]=tracked_cam_radar[0][3]
                                    tracked_cam_radar[1][3]=tracked_cam_radar[0][3]
                                    if logging==False:
                                        print("SWITCHEROO 0")

                                tracked_cam_radar[0]=[tracked_bboxes[0][4],radar_uv[j][2],0,radar_uv[j][2]]#pair IDs
                            else:#if yes then assign to the second bbox
                                tracked_cam_radar[0]=[tracked_bboxes[0][4],radar_uv[j][2],0,radar_uv[j][2]]#pair IDs
                                tracked_cam_radar[1]=[-1,-1,0]#empty second radar
                            break
            else:#check if radar is within range
                radar1_ID=tracked_cam_radar[0][1]
                radar1_uv=cartesian_to_uv(current_raw_radar)
                if (radar1_uv[radar1_ID][0]<(tracked_bboxes[0][0]-tol) or radar1_uv[radar1_ID][0]>(tracked_bboxes[0][2]+tol)):
                    tracked_cam_radar[0][2]=tracked_cam_radar[0][2]+1
                    if(tracked_cam_radar[0][2]>=1):
                        tracked_cam_radar[0][1]=-1#unpair from bbox
                else:
                    tracked_cam_radar[0][2]=0
                
            #check if the first tracked bbox ID is in the paired array
            if(tracked_cam_radar[1][0]!=tracked_bboxes[1][4] or tracked_cam_radar[1][1]==-1):#and tracked_cam_radar[1][0]!=tracked_bboxes[1][4]):
                #if no, search for a pair
                for j in range(len(radar_uv)):
                    #if radar is within range, check if radar is locked with another bbox
                    if (radar_uv[j][0]>(tracked_bboxes[1][0]-tol) and radar_uv[j][0]<(tracked_bboxes[1][2]+tol)):
                        if(radar_uv[j][2]!=tracked_cam_radar[0][1]):
                            if(tracked_cam_radar[0][0]!=tracked_bboxes[1][4]):#check if another bbox is assigned to the second radar

                                if(tracked_cam_radar[0][1]==-1 and tracked_cam_radar[1][1]==-1 and radar_uv[j][2]!=tracked_cam_radar[1][3]):
                                    tracked_cam_radar[0][1]=tracked_cam_radar[1][3] 
                                    tracked_cam_radar[0][3]=tracked_cam_radar[1][3]
                                    if logging==False:
                                        print("SWITCHEROO 1")

                                tracked_cam_radar[1]=[tracked_bboxes[1][4],radar_uv[j][2],0,radar_uv[j][2]]#pair IDs
                                
                            else:#if yes then assign to the second bbox
                                tracked_cam_radar[1]=[tracked_bboxes[1][4],radar_uv[j][2],0,radar_uv[j][2]]#pair IDs
                                tracked_cam_radar[0]=[-1,-1,0]#empty second radar
                            break        
            else:#check if radar is within range
                radar2_ID=tracked_cam_radar[1][1]
                radar2_uv=cartesian_to_uv(current_raw_radar)
                if (radar2_uv[radar2_ID][0]<(tracked_bboxes[1][0]-tol) or radar2_uv[radar2_ID][0]>(tracked_bboxes[1][2]+tol)):
                    tracked_cam_radar[1][2]=tracked_cam_radar[1][2]+1
                    if(tracked_cam_radar[1][2]>=1):
                        tracked_cam_radar[1][1]=-1#unpair from bbox
                else:
                    tracked_cam_radar[1][2]=0
            if logging==False:
                print (tracked_cam_radar)
            #if(tracked_cam_radar[0][1]!=-1 or tracked_cam_radar[1][1]!=-1):
            #    print(current_raw_radar[tracked_cam_radar[0][1]][0],current_raw_radar[tracked_cam_radar[0][1]][1],
            #        current_raw_radar[tracked_cam_radar[1][1]][0],current_raw_radar[tracked_cam_radar[1][1]][1])
        ############################################################################

        
        #print(" ")
        #print(radar_uv)
 
def publish_object(point_coordinate):            
    #global kf_marker,cam_to_x,cam_to_y,persistence_id
    global m_array
    #print(point_coordinate)
    #if logging:
    #    print(round(point_coordinate[0][0],3),
    #          round(point_coordinate[0][1],3),
    #          round(point_coordinate[1][0],3),
    #          round(point_coordinate[1][1],3))
    m=Marker()
    m.header.frame_id="ti_mmwave"
    persistence_id=1
    m.id = persistence_id
    m.type = m.SPHERE
    m.action = m.ADD
    m.scale.x = 0.3
    m.scale.y = 0.3
    m.scale.z = 0.3
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.pose.orientation.w = 1.0
    m.pose.position.x = point_coordinate[0][1]
    m.pose.position.y = point_coordinate[0][0]
    m.pose.position.z = 0.0
    
    #m_array=MarkerArray()
    m_array.markers.append(m)

    m2=Marker()
    m2.header.frame_id="ti_mmwave"
    m2.id = 2
    m2.type = m.SPHERE
    m2.action = m.ADD
    m2.scale.x = 0.3
    m2.scale.y = 0.3
    m2.scale.z = 0.3
    m2.color.a = 1.0
    m2.color.r = 0.0
    m2.color.g = 0.0
    m2.color.b = 1.0
    m2.pose.orientation.w = 1.0
    m2.pose.position.x =point_coordinate[1][1]
    m2.pose.position.y = point_coordinate[1][0]
    m2.pose.position.z = 0.0
    m_array.markers.append(m2)
    #if(persistence_id>50):
    #    m_array.markers.pop(0)
    kf_marker.publish(m_array)


last_running=0
def kf_predict():    
    global kf_timer,last_update,check_running
    
    update_rate=0.05
    while True:
        #try:
        #print ("time",time.time()-kf_timer,last_radar)
        if(time.time()-kf_timer>update_rate and time.time()-last_radar<0.5):
            kf_timer=time.time()
            
            if(radar_updating==0):#if kf is not radar_updating
                x1_predicted=radar_kf1.predict((time.time()-last_update)*rate)#0.25 is rate played
                x2_predicted=radar_kf2.predict((time.time()-last_update)*rate)#0.25 is rate played
                #print("predict",[x1_predicted[0][0],x1_predicted[1][0],x2_predicted[0][0],x2_predicted[1][0]])
                publish_object([[x1_predicted[0][0],x1_predicted[1][0]],[x2_predicted[0][0],x2_predicted[1][0]]])
                
                last_update=time.time()
                #print("updatinggggggggggggggggg")
            if(check_running==1):
                check_running=0
                last_running=time.time()
            elif time.time()-last_running>5:
                print("exiting thread")
                break
        else:
            time.sleep(0.05)
            if(exit==1):
                break
    sys.exit()


def main(args):
    global pose_pub,image_tracker,kf_marker,m_array,radar_kf1,radar_kf2
    rospy.init_node('detector_node')
    #pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)#arrow
    kf_marker= rospy.Publisher('kf_marker', MarkerArray, queue_size=1)
    m_array=MarkerArray()
    #rospy.Subscriber("/ti_mmwave/radar_scan",  RadarScan, callback2)
    rospy.Subscriber("/viz",MarkerArray,radar_input)
    
    
    
    if(image_recognition):
        #image_sub = 
        rospy.Subscriber("/camera/color/image_raw", Image, image_cb, queue_size=1, buff_size=2**24)
        image_tracker=detector_tracker()

    radar_kf1=KF()
    radar_kf2=KF()
    kf_thread = threading.Thread(target=kf_predict)
    #kf_thread.start()

    #key_thread = threading.Thread(target=keyboard_pressed)
    #key_thread.start()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)

'''

[435.97138225642766, 254.95260620297884, 569.4106872236023, 645.8194439318775, 2, 0]
[783.4783671439526, 258.3875842240639, 941.498101180968, 687.6977966161417, 9, 0]
[[  862   305     3]
 [  444   399     4]
 [-1051   489     5]]
 '''

'''
p = PoseStamped()
    p.header.frame_id="ti_mmwave"
    p.pose.position.x = x[0][0]#tracked_range_predict
    p.pose.position.y = x[1][0]#tracked_azimuth_predict
    p.pose.position.z = 0.0
    print(x[1][0],x[0][0])
    #determine quadrant
    if(x[3][0]<0):
        yaw=90-math.degrees(math.atan(x[2][0]/(-0.01*x[3][0])))
    elif (x[3][0]>0):
        yaw=270-math.degrees(math.atan(x[2][0]/(-0.01*x[3][0])))
    else:
        yaw=0.0

    #euler angles to 3d vector
    yaw=math.radians(-1*yaw)
    pitch=0.0
    roll=0.0
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    z=sy * cp * cr - cy * sp * sr
    w=cy * cp * cr + sy * sp * sr

    p.pose.orientation.y = 0.0#vy/norm
    p.pose.orientation.x = 0.0#vx/norm
    p.pose.orientation.z = z
    p.pose.orientation.w = w
    pose_pub.publish(p)
'''