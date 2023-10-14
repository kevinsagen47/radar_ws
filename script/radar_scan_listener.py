#!/usr/bin/env python
 
import rospy
from ti_mmwave_rospkg.msg import RadarScan


def clear_line(n=4):
    LINE_UP = '\033[1A'
    LINE_CLEAR = '\x1b[2K'
    for i in range(n):
        print(LINE_UP, end=LINE_CLEAR)

def callback(data):
    if(data.point_id==0):
        global max_intensity,max_x,max_y,max_z
        print("max ",max_intensity),
        print(" x ",max_x),
        print(" y ",max_y),
        print(" z ",max_z),
        max_intensity=0
        clear_line()

    if(data.intensity>max_intensity and data.x>0.05):
        max_intensity=data.intensity
        max_x=data.x
        max_y=data.y
        max_z=data.z

        
    #rospy.loginfo("%s is age: %f" % (data.point_id,data.intensity))
 
def listener():
 
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("/ti_mmwave/radar_scan",  RadarScan, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__=='__main__':
    global max_intensity,max_x,max_y,max_z
    max_intensity=0
    max_x=0
    max_y=0
    max_z=0
    listener()
