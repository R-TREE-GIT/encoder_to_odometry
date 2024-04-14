#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, numpy
from cv_bridge import CvBridge
from std_msgs.msg import Int8, Float64, String
from sensor_msgs.msg import Image, CompressedImage
import time


class Follower:
	def __init__ (self):
		self.sub_image_original= rospy.Subscriber('/cam_pub', CompressedImage, self.cbFindLane, queue_size = 1)
		self.cvBridge = CvBridge()
		self.counter = 1
	
	def cbFindLane(self, image_msg):
		while (True):
			pub = rospy.Publisher('/line', String, queue_size = 1)#yellow lane detect 정제 값 publisher 
			line_pub = rospy.Publisher('/line_pub', Image, queue_size = 1)
			if self.counter % 3 != 0:#만약 counter함수가 3으로 나눴을 때 0이 아니라면 카운터 1씩 증가(딜레이 기능)
				self.counter += 1
				return
			else:			# 카운터 함수가 0이 됐을 때 1로 지정
				self.counter = 1
			cv_image = self.cvBridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")	
			hsv= cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)#색상 채도 명도
			lower_yellow= numpy.array([30-10, 100, 100])#노란색을 인식하는 최소값
			upper_yellow= numpy.array([30+10, 255, 255])#노란색을 인식하는 최댓값
			mask= cv2.inRange(hsv, lower_yellow, upper_yellow)#입력 행렬,하한값/스칼라,상한값/스칼라

			h, w, d = cv_image.shape# 높이 너비 채널
			search_top= 360           # 인식 할 부분 영역 선택
			search_bot= 380	     #
			mask[0:search_top, 0:w] = 0  # 인시 외 부분을 0으로 필터 처리 
			mask[search_bot:h, 0:w] = 0
			M = cv2.moments(mask)	  # 무게중심을 계산하기 위한  moments() 함수 이용
			#cv2.imshow('mask',mask)
			if M['m00'] > 0:
			#	rospy.loginfo(test)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(cv_image, (cx, cy), 5, (0,0,255), -1)
				err = cx - w/2                     # w/2 는 화면의 중앙, -> w/3 좌측 1/3 지점 
				test = -float(err) / 100 #검출 된 오차 값		
				rospy.loginfo(test)
				print(test)
				if test < -1:
					pub.publish("xgo.turnleft")
				elif test > 1:
					pub.publish("xgo.turnright")
				elif -1 < test and test < 1:
					pub.publish("xgo.forward")
				else:
					pub.publish("xgo.stop")
			else:
				pub.publish("xgo.stop")
				#if test
				#print(cv_image)	#self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image, "jpg"))
			#pub.publish("xgo.stop")
			line_pub.publish(self.cvBridge.cv2_to_imgmsg(cv_image,"bgr8"))
			#cv2.imshow('detectLANE', cv_image)
			#if cv2.waitKey(1) & 0xFF == ord('q'):
			#	break

	def main(self):
		
		rospy.spin()# ROS노드가 shutdown 될 때까지 block 하는 함수, shutdown신호를 받을 때까지 무한루프      
	
if __name__ == '__main__':
	rospy.init_node('line_detect_ros')
	node = Follower()
	node.main()
	
