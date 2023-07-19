#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
clicked_x=50
clicked_y=50
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
    global cv_image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.putText(cv_image, f'({clicked_x},{clicked_y})',(clicked_x,clicked_y),
    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)  
    # draw point on the image
    cv2.circle(cv_image, (clicked_x,clicked_y), 3, (0,255,255), -1)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  cv2.setMouseCallback('Image window', click_event)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)