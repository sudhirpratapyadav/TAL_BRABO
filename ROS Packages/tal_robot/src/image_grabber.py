#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

IMG_WIDTH = 640
IMG_HEIGHT = 480



def callback(msg):

	global bridge
	global ind
	try:
		cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError as e:
		rospy.logerror(e)
	
	ind = ind + 1
	cv2.imwrite(("grabbed_images/pic"+str(ind)+".png"),cv_image)
	cv2.imshow("Image window",cv_image)
	cv2.waitKey(3)

def listener():
	global bridge
	global ind
	ind = 0
	bridge = CvBridge()
	rospy.init_node('object_tracker', anonymous=True)
	rospy.Subscriber('/usb_cam/image_raw', Image, callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.logerror("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
