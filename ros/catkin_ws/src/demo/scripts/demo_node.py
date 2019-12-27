#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage, Image
import rospy
import cv2
from helper import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
from pipeline import process_image, LaneMemory
from PIL import Image
import datetime

def detect_yellow_line(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	height = image.shape[0]
	width = image.shape[1]

	gray_img = grayscale(image)
    
	darkened_img = adjust_gamma(gray_img, 0.5)
	 
	yellow_mask = isolate_color_mask(to_hls(image), np.array([10, 0, 100], dtype=np.uint8), np.array([40, 255, 255], dtype=np.uint8))

	masked_img = cv2.bitwise_and(darkened_img, darkened_img, mask=yellow_mask)
	    
	blurred_img = gaussian_blur(masked_img, kernel_size=7)
	    
	canny_img = canny(blurred_img, low_threshold=70, high_threshold=140)
	    
	hough_lines_imgs = []

	lines = get_hough_lines(canny_img)
	answerY = ""
	answerX = ""
	rightOrLeft = 0
	farOrClose = 0
	
	if(lines.any()):
		length = len(lines)
		for line in lines:
			rightOrLeft += (line[0][0] + line[0][2])/2
			farOrClose += (line[0][1] + line[0][3])/2
		answerX = "right" if rightOrLeft/length > width/2 else "left"
		answerY = "far" if farOrClose/length < height/2 else "close"
		print("length: ", length)
		print("rightOrLeft/length: ", rightOrLeft/length, " width/2" , width/2)
		print("farOrClose/length: ", farOrClose/length, " height/2" , height/2)
		print("side: ", answerX, " dist: ", answerY)
		draw_lines(image,lines)

		now = datetime.datetime.now()
		mcr = str(now.microsecond)

		#cv2.line(image, (50, 0), (50, 200), color=[0, 255, 0], thickness=3) check
		image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		#dir_path = r'/home/maks/ros-task'
		#print(dir_path+"/im"+mcr+".jpg")
		#cv2.imwrite(dir_path+"/im"+mcr+".jpg",image)
		#print(os.listdir(dir_path))

# from https://gist.github.com/awesomebytes/36581763554006f971edc070dbee4bf5#file-image_topic_opencv_skeleton-py-L56
def img_to_cv2(image_msg):
    """
    Convert the image message into a cv2 image (numpy.ndarray)
    to be able to do OpenCV operations in it.
    :param Image or CompressedImage image_msg: the message to transform
    """
    type_as_str = str(type(image_msg))
    if type_as_str.find('sensor_msgs.msg._CompressedImage.CompressedImage') >= 0:
        np_arr = np.fromstring(image_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    elif type_as_str.find('sensor_msgs.msg._Image.Image') >= 0:
        try:
            return self.bridge.imgmsg_to_cv2(image_msg, image_msg.encoding)
        except CvBridgeError as e:
            rospy.logerr("Error when converting image: " + str(e))
            return None
        else:
            rospy.logerr("We don't know how to transform image of type " + str(type(image_msg)))
            return None

class DemoNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"
        self.sub_image = rospy.Subscriber("/None/corrected_image/compressed", CompressedImage, self.cbImage, queue_size=1)
        self.pub_cmd = rospy.Publisher("/None/car_cmd", Twist2DStamped, queue_size=1)


    def cbImage(self, image_msg):
        msg = Twist2DStamped()
        detect_yellow_line(img_to_cv2(image_msg))
        msg.v = 0.1
        msg.omega = 0.1
        self.pub_cmd.publish(msg)

if __name__ == '__main__': 
    rospy.init_node('demo',anonymous=False)
    demo_node = DemoNode()
    rospy.spin()
