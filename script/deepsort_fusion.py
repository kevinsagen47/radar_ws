import os
#os.environ['CUDA_VISIBLE_DEVICES'] = '0'
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
tf.keras.utils.disable_interactive_logging()
tf.debugging.disable_traceback_filtering()
from yolov3.utils import Load_Yolo_model, image_preprocess, postprocess_boxes, nms, draw_bbox, read_class_names
from yolov3.configs import *
import time

from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker
from deep_sort import generate_detections as gdet
import sys
import rospy
from sensor_msgs.msg import Image
import math
video_path   = "test.mp4"

#def Object_tracking(Yolo, video_path, output_path, input_size=416, show=False, CLASSES=YOLO_COCO_CLASSES, score_threshold=0.3, iou_threshold=0.45, rectangle_colors='', Track_only = []):
class detector_tracker:
    def __init__(self):
        #self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.yolo = Load_Yolo_model()
        self.input_size=416
        self.show=False
        self.publish_ros=True
        self.draw_radar_points=True
        #self.verbose
        self.CLASSES=YOLO_COCO_CLASSES
        self.score_threshold=0.3
        self.iou_threshold=0.1
        self.rectangle_colors=(255,0,0)
        self.Track_only = ["person"]
        # Definition of the parameters
        self.max_cosine_distance = 0.7
        self.nn_budget = None
        self.bridge = CvBridge()
        #initialize deep sort object
        self.model_filename = 'model_data/mars-small128.pb'
        self.encoder = gdet.create_box_encoder(self.model_filename, batch_size=1)
        self.metric = nn_matching.NearestNeighborDistanceMetric("cosine", self.max_cosine_distance, self.nn_budget)
        self.tracker = Tracker(self.metric)

        self.times, self.times_2 = [], []

        # by default VideoCapture returns float instead of int
        self.width = 1280#int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = 720#int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #fps = int(vid.get(cv2.CAP_PROP_FPS))
        #codec = cv2.VideoWriter_fourcc(*'XVID')
        #out = cv2.VideoWriter(output_path, codec, fps, (width, height)) # output_path must be .mp4

        self.NUM_CLASS = read_class_names(self.CLASSES)
        self.key_list = list(self.NUM_CLASS.keys()) 
        self.val_list = list(self.NUM_CLASS.values())
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)

    def image_callback(self,frame,radar_uv):
        #_, frame = vid.read()

        
        original_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        original_frame = cv2.cvtColor(original_frame, cv2.COLOR_BGR2RGB)
        
        image_data = image_preprocess(np.copy(original_frame), [self.input_size, self.input_size])
        #image_data = tf.expand_dims(image_data, 0)
        image_data = image_data[np.newaxis, ...].astype(np.float32)

        t1 = time.time()
        if YOLO_FRAMEWORK == "tf":
            pred_bbox = self.yolo.predict(image_data,verbose=0)
        elif YOLO_FRAMEWORK == "trt":
            batched_input = tf.constant(image_data)
            result = self.yolo(batched_input)
            pred_bbox = []
            for key, value in result.items():
                value = value.numpy()
                pred_bbox.append(value)
        
        #t1 = time.time()
        #pred_bbox = Yolo.predict(image_data)
        t2 = time.time()
        
        pred_bbox = [tf.reshape(x, (-1, tf.shape(x)[-1])) for x in pred_bbox]
        pred_bbox = tf.concat(pred_bbox, axis=0)

        bboxes = postprocess_boxes(pred_bbox, original_frame, self.input_size, self.score_threshold)
        bboxes = nms(bboxes, self.iou_threshold, method='nms')

        # extract bboxes to boxes (x, y, width, height), scores and names
        boxes, scores, names = [], [], []
        for bbox in bboxes:
            if len(self.Track_only) !=0 and self.NUM_CLASS[int(bbox[5])] in self.Track_only or len(self.Track_only) == 0:
                boxes.append([bbox[0].astype(int), bbox[1].astype(int), bbox[2].astype(int)-bbox[0].astype(int), bbox[3].astype(int)-bbox[1].astype(int)])
                scores.append(bbox[4])
                names.append(self.NUM_CLASS[int(bbox[5])])

        # Obtain all the detections for the given frame.
        boxes = np.array(boxes) 
        names = np.array(names)
        scores = np.array(scores)
        features = np.array(self.encoder(original_frame, boxes))
        detections = [Detection(bbox, score, class_name, feature) for bbox, score, class_name, feature in zip(boxes, scores, names, features)]

        # Pass detections to the deepsort object and obtain the track information.
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
            index = self.key_list[self.val_list.index(class_name)] # Get predicted object index by object name
            tracked_bboxes.append(bbox.tolist() + [tracking_id, index]) # Structure data, that we could use it with our draw_bbox function

        # draw detection on frame
        image = draw_bbox(original_frame, tracked_bboxes, CLASSES=self.CLASSES, tracking=True)
        #print (tracked_bboxes)
        '''
        t3 = time.time()
        self.times.append(t2-t1)
        times_2.append(t3-t1)
        
        times = times[-20:]
        times_2 = times_2[-20:]

        ms = sum(self.times)/len(self.times)*1000
        fps = 1000 / ms
        fps2 = 1000 / (sum(times_2)/len(times_2)*1000)
        '''
        #image = cv2.putText(image, "Time: {:.1f} FPS".format(fps), (0, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)

        # draw original yolo detection
        #image = draw_bbox(image, bboxes, CLASSES=self.CLASSES, show_label=False, rectangle_colors=self.rectangle_colors, tracking=True)
        
        #print("Time: {:.2f}ms, Detection FPS: {:.1f}, total FPS: {:.1f}".format(ms, fps, fps2))
        #if output_path != '': out.write(image)
        image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        for i in range(len(radar_uv)):
            r= 255 if radar_uv[i][2]%2 else 0
            g= 255 if radar_uv[i][2]%3 else 0
            b= 255 if radar_uv[i][2]%4 else 0
            cv2.circle(image, (radar_uv[i][0],radar_uv[i][1]), 5, (b,g,r), -1)

        if self.show:
            cv2.imshow('output', image)
            
            if cv2.waitKey(25) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                sys.exit()
        if(self.publish_ros):
            image_out = Image()
            image_out = self.bridge.cv2_to_imgmsg(image,"bgr8")
            self.image_pub.publish(image_out)

        return tracked_bboxes
            
    cv2.destroyAllWindows()


#yolo = Load_Yolo_model()
#Object_tracking(yolo, video_path, "track.mp4", input_size=YOLO_INPUT_SIZE, show=False, iou_threshold=0.1, rectangle_colors=(255,0,0), Track_only = ["person"])