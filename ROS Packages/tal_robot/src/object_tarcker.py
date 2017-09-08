#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

greenLower = (50, 50, 50)
greenUpper = (85, 255, 255)
IMG_WIDTH = 640
IMG_HEIGHT = 480

def callback(msg):

	global bridge
	global pub	
	global detector

	try:
		cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
		cv_image = cv2.flip(cv_image,1)
	except CvBridgeError as e:
		rospy.logerror(e)

	#Object detection code started -----------------------------------------------------------------

	#conversion to hsv for thresholding of green color
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	
	#Converting to binary based on thresholds and some eorde and dilate operations
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=3)
	mask = cv2.dilate(mask, None, iterations=3)
	dp = np.zeros([IMG_HEIGHT,IMG_WIDTH,3],dtype=np.dtype('u1'))
	
	#Filling holes in blob
	im_floodfill = mask.copy()
	mask_ff = np.zeros((IMG_HEIGHT+2, IMG_WIDTH+2), np.uint8)
	cv2.floodFill(im_floodfill, mask_ff, (0,0), 255)
	im_floodfill_inv = cv2.bitwise_not(im_floodfill)
	im = mask | im_floodfill_inv
	#filling holes done

	#inverting and blurring image for blob detection
	im = cv2.bitwise_not(im)
	im = cv2.GaussianBlur(im, (9, 9), 2, 2)
	dp[:,:,0] = mask.copy()
	dp[:,:,1] = mask.copy()
	dp[:,:,2] = mask.copy()
	
	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
	# Change thresholds
	params.minThreshold = 0
	params.maxThreshold = 255
	# Filter by Area.
	params.filterByArea = True
	params.minArea = 1000
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
	detector = cv2.SimpleBlobDetector(params)
	# Detect blobs.
	keypoints = detector.detect(im)
	
	
	center_point = Point(-1,-1,-1)
	cir_size = 0
	for keypoint in keypoints:
		if(keypoint.size>cir_size):
			cir_size = keypoint.size
			center_point.x = keypoint.pt[0]
			center_point.y = keypoint.pt[1]
		
	# Draw detected blobs as red circles. cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
	#object detection code completed ----------------------------------------------------------------------
	
	if(center_point.x == -1):
		center_point.x = -1000
		center_point.y = 0
		center_point.z = 0
	else:
		center_point.x = center_point.x - IMG_WIDTH/2
		center_point.y = IMG_HEIGHT/2 - center_point.y
		center_point.z = 30-cir_size

	pub.publish(center_point)
	cv2.imshow("Image window",np.hstack([im_with_keypoints,dp]))
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
