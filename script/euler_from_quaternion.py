'''
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
'''
import math
'''
import numpy as np

x=0.0
y=0.0
z=0.4871
w=-0.8733

siny_cosp = 2 * (w * z + x * y)
cosy_cosp = 1 - 2 * (y * y + z * z)
yaw = math.degrees((np.arctan2(siny_cosp, cosy_cosp)))
print(yaw)
'''

yaw=-45
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
print("z ",sy * cp * cr - cy * sp * sr)#z
print("w ",cy * cp * cr + sy * sp * sr)#w
#'''