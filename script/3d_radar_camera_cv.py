#!/usr/bin/env python
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
clicked_x=50
clicked_y=50
fusion=0
fusion_z=0
degree_z=0
def clear_line(n=5):
    LINE_UP = '\033[1A'
    LINE_CLEAR = '\x1b[2K'
    for i in range(n):
        print(LINE_UP, end=LINE_CLEAR)

def click_event(event, x, y, flags, params):
   if event == cv2.EVENT_LBUTTONDOWN:
      global clicked_x, clicked_y
      print(f'({x},{y})')
      clicked_x=x
      clicked_y=y
      # put coordinates as text on the image
      cv2.putText(cv_image, f'({x},{y})',(x,y),
      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
      
      # draw point on the image
      cv2.circle(cv_image, (x,y), 3, (0,255,255), -1)
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    global cv_image,fusion,fusion_z
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.putText(cv_image, f'({clicked_x},{clicked_y})',(clicked_x,clicked_y),
    #cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)  
    # draw point on the image
    #cv2.circle(cv_image, (clicked_x,clicked_y), 3, (0,255,255), -1)
    cv2.circle(cv_image, (fusion,fusion_z), 5, (0,255,255), -1)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def callback2(data):
    if(data.point_id==0):
        global max_intensity,max_x,max_y,max_z,fusion,fusion_z,degree_z
        print("max ",max_intensity),
        print(" x ",max_x),
        print(" y ",max_y),
        print(" z ",max_z),
        if(max_x>0):
           degree=math.degrees(math.atan(max_y/max_x))
        else:
           degree=0

        if(max_z>0):
           degree_z=math.degrees(math.atan(max_z/max_x))
        else:
           degree_z=0
        fusion=round(640-(degree*14.5+pow(degree,3)*0.0045))
        fusion_z=round(360-(degree_z*14))#+pow(degree_z,3)*0.0045))
        print(" fusion ",round(fusion)),
        print(" fusion z",round(fusion_z)),
        max_intensity=0
        clear_line(6)

    if(data.intensity>max_intensity and data.x>0.1):
        max_intensity=data.intensity
        max_x=data.x
        max_y=data.y
        max_z=data.z

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rospy.Subscriber("/ti_mmwave/radar_scan",  RadarScan, callback2)
  cv2.setMouseCallback('Image window', click_event)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    global max_intensity,max_x,max_y,max_z
    max_intensity=0
    max_x=0
    max_y=0
    max_z=0
    main(sys.argv)