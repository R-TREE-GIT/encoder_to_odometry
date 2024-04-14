import sys, os
import numpy as np
import cv2, rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String, Int8
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBox, BoundingBoxes

class cam_sub:
	def __init__(self):
		self.image = rospy.Subscriber('/object_train', Image, self.show, queue_size = 1)
		self.label = rospy.Subscriber('/label', String, self.action, queue_size = 1)
		self.xgo = rospy.Publisher("/xgo", Int8, queue_size = 1)
		
		self.bridge = CvBridge()
		
	def action(self,label):
		label = label.data
		print(label)
		if label == "user":
			self.num = 17
			self.xgo.publish(self.num)
			print("action == 1")
	def show(self, image):
		#xmin = self.boundbox.xmin
		#ymin = self.boundbox.ymin
		#xmax = self.boundbox.xmax
		#ymax = self.boundbox.ymax
		
		
		image = self.bridge.imgmsg_to_cv2(image,"bgr8")
		
		#cv2.rectangle(image, (self.xmin,self.ymin), (self.xmax,self.ymax),(0,255,0), 3)
		
		cv2.imshow('test',image)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			cv2.destroyWindow('test')
			
		

	def main(self):
		rospy.spin()


if __name__=='__main__':
	rospy.init_node('cam_sub')
	node = cam_sub()
	node.main()
