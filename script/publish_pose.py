#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import math

def publisher():
    pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    
    yaw=5.49779
    pitch=0.0
    roll=0.0
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    print(cy * cp * sr - sy * sp * cr)#x
    print(sy * cp * sr + cy * sp * cr)#y
    z=sy * cp * cr - cy * sp * sr
    w=cy * cp * cr + sy * sp * sr
    print("z ",z)#z
    print("w ",w)#w
    #norm=math.sqrt(z*z+w*w)
    while not rospy.is_shutdown():
        p = PoseStamped()
        p.header.frame_id="ti_mmwave"
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = 0
        # Make sure the quaternion is valid and normalized
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = z
        p.pose.orientation.w = w
        pub.publish(p)
        rate.sleep()
    #real 45 prog 315
    #z  0.4871745124605095
    #w  -0.8733046400935156

    #real 135 prog 180
    #z  0.8939966636005579
    #w  -0.4480736161291701
    
    #real 225 prog 90
    #z 0.8509035245341184
    #w 0.5253219888177297

    #real 315 prog 45
    #z  -0.4871745124605095
    #w  -0.8733046400935156
if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass