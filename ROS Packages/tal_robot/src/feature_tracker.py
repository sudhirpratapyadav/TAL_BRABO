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
	global pub	

	try:
		cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")		
		#cv_image = cv2.GaussianBlur(cv_image, (9, 9), 5, 5)
		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	except CvBridgeError as e:
		rospy.logerror(e)

	#Object detection code started ----------------------------------------------------------------
	
	imgsub_b = cv2.subtract(cv_image[:,:,0],gray_image)
	imgsub_g = cv2.subtract(cv_image[:,:,1],gray_image)
	imgsub_r = cv2.subtract(cv_image[:,:,2],gray_image)	

	#Converting to binary based on thresholds and some eorde and dilate operations
	mask_b = cv2.inRange(imgsub_b, 30, 200)
	mask_b = cv2.erode(mask_b, None, iterations=3)
	mask_b = cv2.dilate(mask_b, None, iterations=3)

	mask_g = cv2.inRange(imgsub_g, 30, 200)
	mask_g = cv2.erode(mask_g, None, iterations=3)
	mask_g = cv2.dilate(mask_g, None, iterations=3)

	mask_r = cv2.inRange(imgsub_r, 30, 200)
	mask_r = cv2.erode(mask_r, None, iterations=3)
	mask_r = cv2.dilate(mask_r, None, iterations=3)	

	#inverting and blurring image for blob detection
	mask_b = cv2.GaussianBlur(cv2.bitwise_not(mask_b), (9, 9), 2, 2)
	mask_g = cv2.GaussianBlur(cv2.bitwise_not(mask_g), (9, 9), 2, 2)
	mask_r = cv2.GaussianBlur(cv2.bitwise_not(mask_r), (9, 9), 2, 2)

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
	# Change thresholds
	params.minThreshold = 0
	params.maxThreshold = 255
	# Filter by Area.
	params.filterByArea = True
	params.minArea = 0
	params.maxArea = 100000
	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.7
	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.8
	# Filter by Inertia
	params.filterByInertia = True
	params.minInertiaRatio = 0.5
	# Create a detector with the parameters
	detector = cv2.SimpleBlobDetector_create(params)

	# Detect blobs.
	keypoints_b = detector.detect(mask_b)
	keypoints_g = detector.detect(mask_g)
	keypoints_r = detector.detect(mask_r)
	
	
	center_point_b = Point(-1,-1,0)
	center_point_g = Point(-1,-1,0)
	center_point_r = Point(-1,-1,0)

	cir_size = 0
	for keypoint in keypoints_b:
		if(keypoint.size>cir_size):
			cir_size = keypoint.size
			center_point_b.x = keypoint.pt[0]
			center_point_b.y = keypoint.pt[1]

	cir_size = 0
	for keypoint in keypoints_g:
		if(keypoint.size>cir_size):
			cir_size = keypoint.size
			center_point_g.x = keypoint.pt[0]
			center_point_g.y = keypoint.pt[1]

	cir_size = 0
	for keypoint in keypoints_r:
		if(keypoint.size>cir_size):
			cir_size = keypoint.size
			center_point_r.x = keypoint.pt[0]
			center_point_r.y = keypoint.pt[1]

	#object detection code completed ----------------------------------------------------------------------
	
	
	pub.publish(center_point_b)
	rospy.loginfo(center_point_b)
	cv2.imshow("Image window",cv_image)
	cv2.waitKey(3)

def listener():
	global pub
	global bridge
	bridge = CvBridge()
	rospy.init_node('object_tracker', anonymous=True)
	rospy.Subscriber('/usb_cam/image_raw', Image, callback)
	pub = rospy.Publisher('/ball_center', Point, queue_size=10)
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
