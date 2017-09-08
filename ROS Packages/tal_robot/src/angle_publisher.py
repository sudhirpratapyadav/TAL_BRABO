#!/usr/bin/env python
# license removed for brevity
import math
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

PI = 3.14
DEG2RAD = PI/180
RAD2DEG = 180/PI
accel = 0.01
vel_1 = 0
def sender():
    pub = rospy.Publisher('joint_angles', Float32MultiArray, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(100) # in hz

    angles = Float32MultiArray()
    angles.layout.dim = [MultiArrayDimension()]
    angles.layout.dim[0].size = 5
    angles.layout.dim[0].stride = 5
    angles.layout.dim[0].label = 'colum_vec'
    angles.layout.data_offset = 0
    angles.data = [0,0,0,0,0]

    t = 0
    while not rospy.is_shutdown():
        pub.publish(angles)
        rospy.loginfo(angles)
        if(t<3000):
            t = t+1
            vel_1 = 0.05
        elif(t<6000):
            t = t+1
            vel_1 = 0.05
        else:
            t = 0

        #vel_1 = 0.4*math.sin(t*2*PI/800)
        angles.data[0] = angles.data[0] - (vel_1)*DEG2RAD
        angles.data[1] = angles.data[1] + (vel_1)*DEG2RAD
        angles.data[2] = angles.data[2] - (vel_1)*DEG2RAD
        angles.data[3] = angles.data[3] - (vel_1)*DEG2RAD
        angles.data[4] = angles.data[4] + (vel_1)*DEG2RAD
        rate.sleep()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
