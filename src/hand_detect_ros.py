import sys, os
import numpy as np
import cv2, rospy
import math
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, String, Int8
from cv_bridge import CvBridge

import mediapipe as mp

class HAND:
	def __init__(self):
		self.image = rospy.Subscriber('/cam_pub', CompressedImage, self.control, queue_size = 1) # raw 이미지

		rospy.Subscriber('/zero', String, self.callback0, queue_size = 1) # Qt hand 탭에서 손 인식 숫자에 따라 액션을 지정하는 콤보박스 선택 시 액션 이름을 sub
		rospy.Subscriber('/one', String, self.callback1, queue_size = 1)
		rospy.Subscriber('/two', String, self.callback2, queue_size = 1)
		rospy.Subscriber('/three', String, self.callback3, queue_size = 1)
		rospy.Subscriber('/four', String, self.callback4, queue_size = 1)
		rospy.Subscriber('/five', String, self.callback5, queue_size = 1)

		# 액션 이름 리스트
		self.action_ls = [['select action'],['Lie down'],['Stand up'],['Crawl'],['Turn left'],['Turn right'],['Squat'],['Turn roll'],['Turn pitch'],['Turn yaw'],['3 axis motion'],['Take a pee'],['Sit down'],['wave hand'],['Give a stretch'],['Wave body'],['Wave side'],['Pray'],['Looking for food'],['Handshake']]
		
		self.zero_sub = ""
		self.one_sub = ""
		self.two_sub = ""
		self.three_sub = ""
		self.four_sub = ""
		self.five_sub = ""

		#self.exit = rospy.Subscriber('/exit', Int8, self.sys, queue_size = 1)
		self.xgo = rospy.Publisher("/xgo", Int8, queue_size = 1) # bitdog 액션 번호 pub
		self.hand_pub = rospy.Publisher("/hand_pub", Image, queue_size = 1) # 손 인식하는 이미지 pub
		self.bridge = CvBridge()
		self.mp_hands = mp.solutions.hands
		self.hands = self.mp_hands.Hands()
		self.mp_draw = mp.solutions.drawing_utils
		self.count = 0

	def check(self, data): #받은 데이터(string)를 action_ls 리스트의 인덱스 번호와 비교 후 int로 리턴
		for i in range(1,20): #
			if data == self.action_ls[i]:
				print("action ======",i)
				return i

	def callback0(self, data):
			self.zero_sub = [data.data]
			self.zero_sub = self.check(self.zero_sub)
			#rospy.loginfo("zero_sub : {}".format(self.zero_sub))
	
	def callback1(self, data):
			self.one_sub = [data.data]
			self.one_sub = self.check(self.one_sub)
			#rospy.loginfo("one_sub : {}".format(self.one_sub))
			

	def callback2(self, data):
			self.two_sub = [data.data]
			self.two_sub = self.check(self.two_sub)
			#rospy.loginfo("two_sub : {}".format(self.two_sub))

	def callback3(self, data):
			self.three_sub = [data.data]
			self.three_sub = self.check(self.three_sub)
			#rospy.loginfo("three_sub : {}".format(self.three_sub))

	def callback4(self, data):
			self.four_sub = [data.data]
			self.four_sub = self.check(self.four_sub)
			#rospy.loginfo("four_sub : {}".format(self.four_sub))

	def callback5(self, data):
			self.five_sub = [data.data]
			self.five_sub = self.check(self.five_sub)
			#rospy.loginfo("five_sub : {}".format(self.five_sub))
		
	def distance(self, x1, x2, y1, y2):
		result = math.sqrt(pow((x1 - x2),2) + pow((y1 - y2),2))
		return result
	
	def number(self, x, y): # 손의 거리 비례 손의 숫자 표현 계산
		
		error = self.distance(x[9], x[0], y[9], y[0]) # 손바닥의 가장 아래 == 0 / 중지의 가장 아래 == 9

		one = math.ceil(self.distance(x[0], x[8], y[0], y[8]) - error) # 검지의 끝 == 8
		two = math.ceil(self.distance(x[0], x[12], y[0], y[12]) - error) # 중지의 끝 == 12
		three = math.ceil(self.distance(x[0], x[16], y[0], y[16]) - error) # 약지의 끝 == 16
		four = math.ceil(self.distance(x[0], x[20], y[0], y[20]) - error) # 소지의 가장 끝 == 20
		five = math.ceil(self.distance(x[17], x[4], y[17], y[4]) - error + 0.05) #엄지의 끝 == 4 / 소지의 가장 아래 == 17

		#print("raw:",one,two,three,four,five)

		number_ls = [[0,0,0,0,0], [1,0,0,0,0], [1,1,0,0,0], [1,1,1,0,0], [1,1,1,1,0], [1,1,1,1,1]] # 0 == 접힌 상태 / 1 == 펴진 상태
		ls = [one,two,three,four,five] # 현재 각 손가락의 접힘과 펴짐 상태를 0 / 1로 표현

		for i in range(6): # 현재 손 상태(ls)와 숫자 표현의 기준(number_ls)을 비교 후 출력 
			if ls == number_ls[i]:
				return i
			else:
				pass

	
	def control(self, image): # image == /cam_pub topic
		self.count += 1
		if self.count >= 3:
			return
		self.count = 0
		
		image = self.bridge.compressed_imgmsg_to_cv2(image,"bgr8")
		
		results = self.hands.process(image)
		
		if results.multi_hand_landmarks:
			id_ls_y = []
			id_ls_x = []
			for hand in results.multi_hand_landmarks:
				self.mp_draw.draw_landmarks(image,hand, self.mp_hands.HAND_CONNECTIONS)
				
			for landmark in hand.landmark: # 손의 관절 포인트의 위치 출력
				id_ls_x.append(round(landmark.x,2))
				id_ls_y.append(round(landmark.y,2))

			num = self.number(id_ls_x, id_ls_y) # def number(self, x, y) 현재 손이 표현하는 숫자를 출력하는 함수
			print("num:",num)

			print("zerosub : ",self.zero_sub)

			if num == 0: # 손이 표현하는 숫자가 0일 경우
				self.xgo.publish(self.zero_sub) #Qt에서 설정한 bitdog 액션 pub
			elif num == 1:
				self.xgo.publish(self.one_sub)
			elif num == 2:
				self.xgo.publish(self.two_sub)
			elif num == 3:
				self.xgo.publish(self.three_sub)
			elif num == 4:
				self.xgo.publish(self.four_sub)
			elif num == 5:
				self.xgo.publish(self.five_sub)
		
		#cv2.putText()
		#cv2.putText(image,'lefteye',(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0),2)
		self.hand_pub.publish(self.bridge.cv2_to_imgmsg(image,"bgr8"))

	def main(self):
		rospy.spin()


if __name__=='__main__':
	rospy.init_node('hand_detect_ros')
	node = HAND()
	node.main()
