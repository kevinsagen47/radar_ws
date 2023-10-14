#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow
from __future__ import print_function

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

import os
import numpy as np
clicked_x=50
clicked_y=50
fusion=0
tracked_x=0.0
tracked_y=0.0
degree_tracked=0.0
data_velocity=0.0
data_range=0.0
#######################################################################
#                        DETECTION                                    #
#######################################################################
try:
    import tensorflow.compat.v1  as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

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

# Detection

class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.sess = tf.Session(graph=detection_graph,config=config)
        print("INITIATED<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,<<<<<<<<<<<<")

    def image_cb(self, data):
        global fusion
        global data_velocity, data_range
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

        img=cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header

        cv2.circle(img, (fusion,300), 5, (0,255,255), -1)
        color = (128, 0, 0)#navy
        #color =(255,150,0)#bright blue
        if data_velocity<0:
            #cv2.putText(img, f'velocity:{data_velocity}m/s',(10,620),cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            draw_text(img, f'velocity:{data_velocity}m/s', font_scale=1, pos=(10, 630), text_color_bg=(255, 255, 255))
        else:
            #cv2.putText(img, f'velocity:  {data_velocity}m/s',(10,620),cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            draw_text(img, f'velocity:  {data_velocity}m/s', font_scale=1, pos=(10, 630), text_color_bg=(255, 255, 255))
        
        

        #cv2.putText(img,     f'range  :{data_range}m',(10,660),cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        draw_text(img, f'range  : {data_range}m', font_scale=1, pos=(10, 660), text_color_bg=(255, 255, 255))
        cv2.imshow("Image window", img)
        cv2.waitKey(3)
        self.image_pub.publish(image_out)

    def object_predict(self,object_data, header, image_np,image):
        global fusion
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
        obj.bbox.size_y = int((dimensions[2]-dimensions[0])*image_height)
        obj.bbox.size_x = int((dimensions[3]-dimensions[1] )*image_width)
        obj.bbox.center.x = int((dimensions[1] + dimensions [3])*image_height/2)
        obj.bbox.center.y = int((dimensions[0] + dimensions[2])*image_width/2)

        return obj
####################################################################################################

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
def callback2(data):
    if(data.point_id==0):
        global fusion,degree_tracked
        fusion=round(640-(degree_tracked*14.5+pow(degree_tracked,3)*0.0045))
        
def marker_array_callback(point):
    global tracked_x,tracked_y,degree_tracked,data_range
    tracked_x=point.markers[0].pose.position.x
    tracked_y=point.markers[0].pose.position.y
    degree_tracked=math.degrees(math.atan(tracked_y/tracked_x))
    data_range=round(tracked_x,3)
    
def velo_range_callback(data):
    global data_velocity#, data_range
    print("velocity: ",data.data[0]," range ",data.data[1])
    data_velocity=round(data.data[0],3)
    #data_range=round(data.data[1],3)

def main(args):
    rospy.init_node('detector_node')

    rospy.Subscriber("/ti_mmwave/radar_scan",  RadarScan, callback2)
    rospy.Subscriber("/viz",MarkerArray,marker_array_callback)
    rospy.Subscriber("/velo_range_array",Float32MultiArray,velo_range_callback)

    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
