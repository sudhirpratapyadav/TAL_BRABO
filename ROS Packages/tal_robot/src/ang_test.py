#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension

PI = 3.14
DEG2RAD = PI/180
RAD2DEG = 180/PI

def sender():
    pub = rospy.Publisher('joint_angles', Int16MultiArray, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    angles = Int16MultiArray()
    angles.layout.dim = [MultiArrayDimension()]
    angles.layout.dim[0].size = 7
    angles.layout.dim[0].stride = 7
    angles.layout.dim[0].label = 'colum_vec'
    angles.layout.data_offset = 0
    angles.data = [7,0,0,0,0,1,1]

    #t = 0
    while not rospy.is_shutdown():
        
        pub.publish(angles)
        rospy.loginfo(angles)

        # if(t<100):
        # 	t = t+1
        # 	angles.data[0] = angles.data[0] + (0.1)*DEG2RAD;
        
        rate.sleep()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
