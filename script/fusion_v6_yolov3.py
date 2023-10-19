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
from motrackers import CentroidTracker#, CentroidKF_Tracker, SORT, IOUTracker
from motrackers.utils import draw_tracks

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
os.environ["CUDA_VISIBLE_DEVICES"]="0"
import numpy as np
from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker
from deep_sort import generate_detections as gdet
from yolov3.utils import draw_bbox
from yolov3.utils import Load_Yolo_model, image_preprocess, postprocess_boxes, nms, c, read_class_names
from yolov3.configs import *
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
cam_to_x=0
cam_to_y=0

last_cam_update=0
prev_cam_range=0
prev_cam_azimuth=0
exit=0
cam_updating=0
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
gamma=1.0
class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        #self.pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.sess = tf.Session(graph=detection_graph,config=config)
        #self.tracker = CentroidTracker(max_lost=100, tracker_output_format='mot_challenge')

        max_cosine_distance = 0.7
        nn_budget = None
        #initialize deep sort object
        model_filename = 'model_data/mars-small128.pb'
        self.encoder = gdet.create_box_encoder(model_filename, batch_size=1)
        metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        self.tracker = Tracker(metric)
        print("INITIATED<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,<<<<<<<<<<<<")

    def image_cb(self, data):
        global radar_to_x,locked,locked_with,tracked_azimuth,radar_to_y
        global data_velocity, data_range,tracked_range,cam_to_y,cam_to_x
        global last_cam_update,prev_cam_azimuth,prev_cam_range,gamma,cam_updating,radar_updating
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            print(e)
        image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
        image = self.adjust_gamma(image, gamma)
        image_ori = cv_image
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

        bboxes = np.array(bboxes).astype('int')
        class_ids = np.array(class_ids)
        confidences = np.array(confidences)
        features = np.array(self.encoder(image_ori, bboxes))
        detections = [Detection(bboxes, confidences, class_name, feature) 
                      for bboxes, confidences, class_name, feature 
                      in zip(bboxes, confidences, class_ids, features)]
        #print (bboxes)
        #tracks = self.tracker.update(np.array(bboxes).astype('int'), np.array(class_ids).astype('int'), np.array(confidences))
        #'''
        self.tracker.predict()
        self.tracker.update(detections)

        # Obtain info from the tracks
        tracked_bboxes = []
        for track in self.tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 5:
                continue 
            bbox = track.to_tlbr() # Get the corrected/predicted bounding box
            class_name = track.get_class() #Get the class name of particular object
            tracking_id = track.track_id # Get the ID for the particular track
            index = class_name#key_list[val_list.index(class_name)] # Get predicted object index by object name
            tracked_bboxes.append(bbox.tolist() + [tracking_id, index]) # Structure data, that we could use it with our draw_bbox function

        
        #print(tracked_bboxes)
        #img = draw_bbox(image_ori, tracked_bboxes, tracking=True)

        img=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #img=draw_tracks(img, tracks)
        cv2.imshow("Image window", img)
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
    def adjust_gamma(self,image, gamma):

        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")

        return cv2.LUT(image, table)
####################################################################################################

radar_updating=0
tracked_range_predict=0
tracked_azimuth_predict=0
def radar_input(point):#radar update
    global tracked_range,tracked_azimuth,degree_tracked,data_range,radar_to_x,last_radar,radar_to_y,prev_tracked_range,prev_tracked_azimuth,prev_radar_to_x,last_update,Vx,range_speed
    global pose_pub,azimuth_speed,tracked_range_predict,tracked_azimuth_predict,kf_marker,radar_updating,tracked_range_predict,radar_kf,cam_updating
    tracked_range=round(point.markers[0].pose.position.x,3)
    tracked_azimuth=round(point.markers[0].pose.position.y,3)
    #print(tracked_azimuth,tracked_range)
    if(cam_updating==0):
        if(prev_tracked_azimuth!=tracked_azimuth or prev_tracked_range!=tracked_range ):#if there is new data, update kf
            radar_updating=1
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

            x_update=radar_kf.update(x,10.)
            tracked_range_predict=x_update[0][0]
            tracked_azimuth_predict=x_update[1][0]
            
            last_radar=time.time()
            last_update=time.time()
            publish_object(x_update)
    radar_updating=0
    if(exit==1):
        sys.exit()
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
    global pose_pub,azimuth_speed,tracked_range_predict,tracked_azimuth_predict,kf_marker,radar_updating,kf_timer,radar_kf,gamma
    
    while True:
        #try:
        if(time.time()-kf_timer>0.033 and time.time()-last_radar<1):
            kf_timer=time.time()
            if(radar_updating==0 and cam_updating==0):#if kf is not radar_updating
                radar_to_x=round(radar_to_x+0.1*Vx*(time.time()-last_update))
                x_predicted=radar_kf.predict(time.time()-last_update)
                tracked_range_predict=x_predicted[0][0]
                tracked_azimuth_predict=x_predicted[1][0]
                publish_object(x_predicted)
                last_update=time.time()
        else:
            time.sleep(0.05)
            if(exit==1):
                break
    sys.exit()
                
                
                
            
            
        #except KeyboardInterrupt:
        #    break
    
persistence_id=0
def publish_object(x):            
    global kf_marker,cam_to_x,cam_to_y,m_array,persistence_id
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

    m=Marker()
    m.header.frame_id="ti_mmwave"
    persistence_id=1
    m.id = persistence_id
    m.type = m.SPHERE
    m.action = m.ADD
    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.2
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.pose.orientation.w = 1.0
    m.pose.position.x = x[0][0]
    m.pose.position.y = x[1][0]
    m.pose.position.z = 0.0
    
    #m_array=MarkerArray()
    m_array.markers.append(m)

    m2=Marker()
    m2.header.frame_id="ti_mmwave"
    m2.id = 2
    m2.type = m.SPHERE
    m2.action = m.ADD
    m2.scale.x = 0.2
    m2.scale.y = 0.2
    m2.scale.z = 0.2
    m2.color.a = 1.0
    m2.color.r = 0.0
    m2.color.g = 1.0
    m2.color.b = 0.0
    m2.pose.orientation.w = 1.0
    m2.pose.position.x = cam_to_y
    m2.pose.position.y = cam_to_x
    m2.pose.position.z = 0.0
    m_array.markers.append(m2)
    if(persistence_id>50):
        m_array.markers.pop(0)
    kf_marker.publish(m_array)

def getchar():
   #Returns a single character from standard input
   import tty, termios, sys
   fd = sys.stdin.fileno()
   old_settings = termios.tcgetattr(fd)
   try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
   finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
   return ch
def keyboard_pressed():
    global gamma,exit
    while True:
        ch=getchar()
        if (ch=='d'):  
            gamma=0.1
            print("GOING DARKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK")
            print(" ")
            print(" ")
            print(" ")
            print(" ")
        if (ch=='s'):  
            gamma=1.0
        if (ch=='a'):  
            gamma=10.0
        if (ch=='q'): 
            time.sleep(1) 
            exit=1
            break
def main(args):
    global pose_pub,kf_marker,radar_kf,m_array
    rospy.init_node('detector_node')
    pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)
    kf_marker= rospy.Publisher('kf_marker', MarkerArray, queue_size=1)
    m_array=MarkerArray()
    #rospy.Subscriber("/ti_mmwave/radar_scan",  RadarScan, callback2)
    #rospy.Subscriber("/viz",MarkerArray,radar_input)
    #rospy.Subscriber("/velo_range_array",Float32MultiArray,velo_range_callback)
    radar_kf=KF()
    kf_thread = threading.Thread(target=kf_predict)
    #kf_thread.start()

    key_thread = threading.Thread(target=keyboard_pressed)
    #key_thread.start()
    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)



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
            #print("radar_to_x locked with ID",locked_with)
            
            x_min_cali=tracked_image[i][2]
            x_max_cali=tracked_image[i][2]+tracked_image[0][4]

            u_to_degree=(640-current_ground_truth_x)/14.5
            degree_to_radian=u_to_degree*math.pi/180
            tan_degree=math.tan(degree_to_radian)
            if(u_to_degree!=0):
                cam_to_x=tracked_range*tan_degree

            cam_to_y=tracked_range
            
            last_cam_delta=time.time()-last_cam_update
            #cam_range_speed=(cam_to_y-prev_cam_range)/last_cam_delta#range in meters
            #cam_azimuth_speed=(cam_to_x-prev_cam_azimuth)/last_cam_delta#azimuth in meters
            if(radar_updating==0):
                cam_updating=1
                x=np.array([[cam_to_y],[cam_to_x],[cam_range_speed],[azimuth_speed]])
                #print(x[1][0],x[0][0])
                #x_update=radar_kf.update(x,1000.)

            #if((time.time()-last_radar)>0.5):
            #    publish_object(x_update)
            cam_updating=0
            last_cam_update=time.time()
            prev_cam_range=cam_to_y
            prev_cam_azimuth=cam_to_x
            
            


            #if(x_min_cali<radar_to_x and x_max_cali>radar_to_x and (time.time()-last_radar)<0.5):
            #    #print(current_ground_truth_y," ",tracked_range)
            #    print(x_max_cali-x_min_cali," ",tracked_range)
            
'''
#current_ground_truth_x=tracked_image[i][2]+tracked_image[0][4]*0.5
#current_ground_truth_y=tracked_image[i][3]+tracked_image[0][5]*0.5
'''
degree_tracked=math.degrees(math.atan(tracked_azimuth/tracked_range))
radar_to_x=round(640-(degree_tracked*14.5+pow(degree_tracked,3)*0.0045))
radar_to_y=round(50*tracked_range+350)
'''

#if(current_frame_found==0):
#    locked=0
#    print("depending on radar")
#else:
#    print("radar_to_x locked with ID",locked_with," error ",current_ground_truth_x-radar_to_x)
