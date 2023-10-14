#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow

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
from motrackers import CentroidTracker, CentroidKF_Tracker, SORT, IOUTracker
from motrackers.utils import draw_tracks

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ti_mmwave_rospkg.msg import RadarScan
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import Float32MultiArray
import time
from geometry_msgs.msg import PoseStamped

import tf as transferfunction
import threading
import os
import numpy as np
clicked_x=50
clicked_y=50
radar_to_x=0
locked =0
tracked_range=0.0
tracked_azimuth=0.0
degree_tracked=0.0
data_velocity=0.0
data_range=0.0
last_radar=0
prev_tracked_range=0
prev_tracked_azimuth=0
prev_radar_to_x=0
last_update=0
Vx=0
azimuth_speed=0
range_speed=0
kf_timer=0
#######################################################################
#                        DETECTION                                    #
#######################################################################
#'''
try:
    import tensorflow.compat.v1  as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)



# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.4

######### Set model here ############
MODEL_NAME =  'ssd_mobilenet_v1_coco_11_06_2017'
# By default models are stored in data/models/
MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'data','models' , MODEL_NAME)
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### Set the label map file here ###########
LABEL_NAME = 'mscoco_label_map.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]),'data','labels', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 90

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Setting the GPU options to use fraction of gpu that has been set
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION
#'''
# Detection

class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        #self.pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.sess = tf.Session(graph=detection_graph,config=config)
        self.tracker = CentroidTracker(max_lost=100, tracker_output_format='mot_challenge')
        print("INITIATED<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,<<<<<<<<<<<<")

    def image_cb(self, data):
        global radar_to_x,locked,locked_with,tracked_azimuth,radar_to_y
        global data_velocity, data_range,tracked_range
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = np.asarray(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = detection_graph.get_tensor_by_name('detection_scores:0')
        classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        (boxes, scores, classes, num_detections) = self.sess.run([boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        objects=vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2)

        objArray.detections =[]
        objArray.header=data.header
        object_count=1

        for i in range(len(objects)):
            object_count+=1
            objArray.detections.append(self.object_predict(objects[i],data.header,image_np,cv_image))

        self.object_pub.publish(objArray)

        bboxes, confidences, class_ids = [], [], []
        for i in range(len(objects)):
            bboxes.append([(objArray.detections[i].bbox.center.x-(objArray.detections[i].bbox.size_x/2)), 
                          (objArray.detections[i].bbox.center.y-(objArray.detections[i].bbox.size_y/2)), #0,0]))
                        objArray.detections[i].bbox.size_x, objArray.detections[i].bbox.size_y])
            confidences.append(scores[0][i])
            #print ()
            class_ids.append(int(classes[0][i]))
        #print (bboxes)
        tracks = self.tracker.update(np.array(bboxes).astype('int'), np.array(class_ids).astype('int'), np.array(confidences))
        #'''

        img=cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        img=draw_tracks(img, tracks)


        #states 
        #0 not locked
        #1 locked


        '''
        if(len(tracks)>0):
            tracked_image=np.array(tracks).astype('int')
            #print(tracked_image[0][1:6])
            x_min=tracked_image[0][2]
            x_max=tracked_image[0][2]+tracked_image[0][4]
            print("min ",x_min," x_max ",x_max)
            print (radar_to_x)
        '''
        current_frame_found=0
        tracked_image=np.array(tracks).astype('int')
        if(locked==0):
            for i in range(len(tracked_image)):
                x_min=tracked_image[i][2]
                x_max=tracked_image[i][2]+tracked_image[0][4]
                if(x_min<radar_to_x and x_max>radar_to_x):
                    #print("radar_to_x locked with ID",tracked_image[i][1])
                    locked=1
                    locked_with=tracked_image[i][1]
            if(locked==0 and 'locked_with' in locals()):
                for i in range(len(tracked_image)):
                    if(tracked_image[i][1]==locked_with):
                        current_frame_found=1
                        locked=1
                        print("USING PREVIOUS LOCKED INFORMARTION<<<<<<<<<<<<<<<<<<<")
                print("GONSKIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")

        
        if(locked==1):
            for i in range(len(tracked_image)):
                if(tracked_image[i][1]==locked_with):
                    current_frame_found=1
                    current_ground_truth_x=tracked_image[i][2]+tracked_image[0][4]*0.5
                    current_ground_truth_y=tracked_image[i][3]+tracked_image[0][5]*0.5
                    #x_min=tracked_image[i][2]
                    #x_max=tracked_image[i][2]+tracked_image[0][4]
                    
                    #'''
                    x_min_cali=tracked_image[i][2]
                    x_max_cali=tracked_image[i][2]+tracked_image[0][4]
                    if(x_min_cali<radar_to_x and x_max_cali>radar_to_x and (time.time()-last_radar)<0.5):
                        #print(current_ground_truth_y," ",tracked_range)
                        print(x_max_cali-x_min_cali," ",tracked_range)
                    #'''

        #current_ground_truth_x=tracked_image[i][2]+tracked_image[0][4]*0.5
        #current_ground_truth_y=tracked_image[i][3]+tracked_image[0][5]*0.5
        '''
        degree_tracked=math.degrees(math.atan(tracked_azimuth/tracked_range))
        radar_to_x=round(640-(degree_tracked*14.5+pow(degree_tracked,3)*0.0045))
        radar_to_y=round(50*tracked_range+350)
        '''

        if(current_frame_found==0):
            locked=0
            #print("depending on radar")
        #else:
        #    print("radar_to_x locked with ID",locked_with," error ",current_ground_truth_x-radar_to_x)
        
        

        cv2.circle(img, (radar_to_x,radar_to_y), 5, (0,255,255), -1)
        color = (128, 0, 0)#navy
        #color =(255,150,0)#bright blue
        '''
        if data_velocity<0:
            #cv2.putText(img, f'velocity:{data_velocity}m/s',(10,620),cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            self.draw_text(img, f'velocity:{data_velocity}m/s', font_scale=1, pos=(10, 630), text_color_bg=(255, 255, 255))
        else:
            #cv2.putText(img, f'velocity:  {data_velocity}m/s',(10,620),cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            self.draw_text(img, f'velocity:  {data_velocity}m/s', font_scale=1, pos=(10, 630), text_color_bg=(255, 255, 255))
        '''
        

        #cv2.putText(img,     f'range  :{data_range}m',(10,660),cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        #self.draw_text(img, f'range  : {data_range}m', font_scale=1, pos=(10, 660), text_color_bg=(255, 255, 255))
        
        #cv2.imshow("Image window", img)

        cv2.waitKey(3)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        self.image_pub.publish(image_out)

    def object_predict(self,object_data, header, image_np,image):
        global radar_to_x
        image_height,image_width,channels = image.shape
        obj=Detection2D()
        obj_hypothesis= ObjectHypothesisWithPose()

        object_id=object_data[0]
        object_score=object_data[1]
        dimensions=object_data[2]

        local_size_x=int((dimensions[3]-dimensions[1] )*image_width)
        local_center_x=int((dimensions[1] + dimensions [3])*image_height/2)

        obj.header=header
        obj_hypothesis.id = object_id
        obj_hypothesis.score = object_score
        obj.results.append(obj_hypothesis)
        obj.bbox.size_y   = int((dimensions[2]-dimensions[0])*image_height)
        obj.bbox.size_x   = int((dimensions[3]-dimensions[1] )*image_width)
        obj.bbox.center.x = int(((dimensions[1] + dimensions [3])*image_width)/2)
        obj.bbox.center.y = int(((dimensions[0] + dimensions[2])*image_height)/2)
        
        return obj
    def draw_text(img, text,
          font=cv2.FONT_HERSHEY_SIMPLEX,
          pos=(0, 0),
          font_scale=3,
          font_thickness=2,
          text_color=(128, 0, 0),
          text_color_bg=(0, 0, 0)
          ):

        x, y = pos
        text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
        text_w, text_h = text_size
        cv2.rectangle(img, (x-5,y-5), (x + text_w+10, y + text_h+10), text_color_bg, -1)
        #cv2.putText(img, text, (x, y + text_h + font_scale - 1), font, font_scale, text_color, font_thickness)
        cv2.putText(img, text, (x, y + text_h + font_scale - 1), font, font_scale, text_color, font_thickness)

        return text_size
####################################################################################################

updating=0
tracked_range_predict=0
tracked_azimuth_predict=0
def radar_input(point):#radar update
    global tracked_range,tracked_azimuth,degree_tracked,data_range,radar_to_x,last_radar,radar_to_y,prev_tracked_range,prev_tracked_azimuth,prev_radar_to_x,last_update,Vx,range_speed
    global pose_pub,azimuth_speed,tracked_range_predict,tracked_azimuth_predict,kf_marker,updating,tracked_range_predict,radar_kf
    tracked_range=round(point.markers[0].pose.position.x,3)
    tracked_azimuth=round(point.markers[0].pose.position.y,3)
    
    if(prev_tracked_azimuth!=tracked_azimuth or prev_tracked_range!=tracked_range):#if there is new data, update kf
        updating=1
        degree_tracked=math.degrees(math.atan(tracked_azimuth/tracked_range))
        radar_to_x=round(640-(degree_tracked*14.5+pow(degree_tracked,3)*0.0045))
        radar_to_y=round(50*tracked_range+350)
        data_range=round(tracked_range,3)

        last_radar_delta=time.time()-last_radar
        range_speed=(tracked_range-prev_tracked_range)/last_radar_delta#range in meters
        azimuth_speed=(tracked_azimuth-prev_tracked_azimuth)/last_radar_delta#azimuth in meters
        Vx=(radar_to_x-prev_radar_to_x)/last_radar_delta#azimuth but in pixel
        #print("Vx ",Vx," range_speed ",range_speed, "delta time",last_radar_delta)
        #print((radar_to_x-prev_radar_to_x))
        #print("Vx ",Vx)
        prev_radar_to_x=radar_to_x
        prev_tracked_range=tracked_range
        prev_tracked_azimuth=tracked_azimuth
        tracked_range_predict=tracked_range
        tracked_azimuth_predict=tracked_azimuth

        x=np.array([[tracked_range_predict],[tracked_azimuth_predict],[range_speed],[azimuth_speed]])
        
        #print("input",np.transpose(x))
        #print("output",np.transpose(radar_kf.update(x)))
        #print(" ")

        x_update=radar_kf.update(x)
        tracked_range_predict=x_update[0][0]
        tracked_azimuth_predict=x_update[1][0]
        
        last_radar=time.time()
        last_update=time.time()
        publish_object(x_update)
    updating=0
    '''
    else:#if there is no new data, predict with velocity, kf update
        predict_add=Vx*(time.time()-last_update)
        #print("predict add",predict_add," Vx ",Vx," time ",time.time())
        radar_to_x=round(radar_to_x+0.1*Vx*(time.time()-last_update))
        tracked_range_predict=tracked_range_predict+0.5*range_speed*(time.time()-last_update)
        tracked_azimuth_predict=tracked_azimuth_predict+0.5*azimuth_speed*(time.time()-last_update)
        last_update=time.time()
    '''
    #print(azimuth_speed," ",range_speed)
    #publish predicted
    
    #self.image_pub.publish(image_out)
def kf_predict():
    global tracked_range,tracked_azimuth,degree_tracked,data_range,radar_to_x,last_radar,radar_to_y,prev_tracked_range,prev_tracked_azimuth,prev_radar_to_x,last_update,Vx,range_speed
    global pose_pub,azimuth_speed,tracked_range_predict,tracked_azimuth_predict,kf_marker,updating,kf_timer,radar_kf
    
    while True:
        if(time.time()-kf_timer>0.033):
            kf_timer=time.time()
            if(updating==0):#if kf is not updating
                radar_to_x=round(radar_to_x+0.1*Vx*(time.time()-last_update))
                x_predicted=radar_kf.predict(time.time()-last_update)
                tracked_range_predict=x_predicted[0][0]
                tracked_azimuth_predict=x_predicted[1][0]
                publish_object(x_predicted)
                last_update=time.time()
        else:
            time.sleep(0.02)

def publish_object(x):            
    global kf_marker
    p = PoseStamped()
    p.header.frame_id="ti_mmwave"
    p.pose.position.x = x[0][0]#tracked_range_predict
    p.pose.position.y = x[1][0]#tracked_azimuth_predict
    p.pose.position.z = 0.0
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

    m=Marker()
    m.header.frame_id="ti_mmwave"
    m.type = m.SPHERE
    m.action = m.ADD
    m.scale.x = 0.6
    m.scale.y = 0.6
    m.scale.z = 0.6
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.pose.orientation.w = 1.0
    m.pose.position.x = x[0][0]
    m.pose.position.y = x[1][0]
    m.pose.position.z = 0.0
    m_=MarkerArray()
    m_.markers.append(m)
    kf_marker.publish(m_)

def main(args):
    global pose_pub,kf_marker,radar_kf
    rospy.init_node('detector_node')
    pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)
    kf_marker= rospy.Publisher('kf_marker', MarkerArray, queue_size=1)
    #rospy.Subscriber("/ti_mmwave/radar_scan",  RadarScan, callback2)
    rospy.Subscriber("/viz",MarkerArray,radar_input)
    #rospy.Subscriber("/velo_range_array",Float32MultiArray,velo_range_callback)
    radar_kf=KF()
    kf_thread = threading.Thread(target=kf_predict)
    kf_thread.start()
    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
