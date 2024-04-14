#!/usr/bin/env python3
#-*- cofing:urf-8 -*-

import sys, os, glob, json
import numpy as np
import cv2
import time
from std_msgs.msg import Float64, String, Int32, Int8
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import subprocess
from datetime import datetime
import xml.etree.ElementTree as et
from tqdm import tqdm

class camera:
	def __init__(self):

		self.my_pc_name = "rtree-ai" # 자신의 pc이름으로 변경
		
		self.sub_image = rospy.Subscriber('/cam_pub', CompressedImage, self.camera_capture, queue_size =1) # raw 이미지 sub
		self.now_dir_pub = rospy.Publisher('/now_dir', String, queue_size =1) # 이미지 저장 파일 이름 생성
		self.capture_count_pub = rospy.Publisher('capture_count',Int32, queue_size =1) # Qt의 프로그래스 바에 진행을 위한 토픽
		self.capture = rospy.Subscriber('/capture', Int8, self.capture_signal, queue_size = 1) # Qt에서 캡쳐 버튼을 누를때 이미지 캡쳐를 위한 토픽
		self.capture_count = 0
		self.cvBridge = CvBridge() 
		self.counter = 0
		self.once = True
		self.now_dir = 0
		self.now_time = 0
		self.success = False
		self.signal = 0
		self.sequence = 0 # 순차적으로 프로그램을 실행하기 위한 변수
		self.pub_train_img = rospy.Publisher("/train_cam",Image, queue_size = 1) # 이미지에 바운드 박스를 그린 이미지를 Qt에 보내는 토픽
		
		path = "./" 
		self.annot_path = os.path.join(path,"annotations")
		self.img_path = os.path.join(path,"ImageSets")
		self.label_path = os.path.join(path,"labels")
		
	def capture_signal(self, data):  # Qt에서 캡쳐 버튼을 누를때 신호를 받기 위한 메서드
		self.signal = data.data
		
	def xml_to_yolo_bbox(self,bbox, w, h): # 바운딩 박스 값과 label 데이터를 담은 xml파일을 yolo형식으로 변환하기 위한 메서드 
		# xmin, ymin, xmax, ymax
		x_center = ((bbox[2] + bbox[0]) / 2) / w
		y_center = ((bbox[3] + bbox[1]) / 2) / h
		width = (bbox[2] - bbox[0]) / w
		height = (bbox[3] - bbox[1]) / h
		return [x_center, y_center, width, height]
		
	def xml2txt(self):
		classes = []
		files = glob.glob(os.path.join(self.annot_path, '*.xml'))
		for fil in tqdm(files):
    
			basename = os.path.basename(fil)
			filename = os.path.splitext(basename)[0]
			
			result = []
				
			tree = et.parse(fil)
			root = tree.getroot()
			width = int(root.find("size").find("width").text)
			height = int(root.find("size").find("height").text)
			for obj in root.findall('object'):
				label = obj.find("name").text
				if label not in classes:
					classes.append(label)
				index = classes.index(label)
				pil_bbox = [int(x.text) for x in obj.find("bndbox")]
				yolo_bbox = self.xml_to_yolo_bbox(pil_bbox, width, height)
				bbox_string = " ".join([str(x) for x in yolo_bbox])
				result.append(f"{index} {bbox_string}")
			if result:
				with open(os.path.join(self.label_path, f"{filename}.txt"), "w", encoding="utf-8") as f:
					f.write("\n".join(result))
	
	def remmove_duplicated_line(self,target_file): # 가끔 같은 이름의 이미지가 중복 저장되어 같은 이름의 파일을 제거하는 메서드
		no_dup_lines = set()
		with open(target_file,'r+') as fp:
			file_lines = fp.readlines()
			fp.seek(0)
			for line in file_lines:
				if line not in no_dup_lines:
					fp.write(line)
					no_dup_lines.add(line)
				fp.truncate()
			
	def get_files_count(self,folder_path): #파일 개수 확인
		num_file = os.listdir(folder_path)
		return len(num_file)
		
	def createFolder(self,directory): #yolo형식의 디렉토리 트리를 만들기 위해 폴더를 생성하는 매서드
		os.makedirs(directory)
		
	def camera_capture(self, image): 
		
		if self.sequence == 0: # 첫 번째 순서
			now = time.localtime() 
			self.now_time = datetime.utcnow().strftime('%Y%m%d-%H%M%S%f')  # 현재 시간을 변수에 저장 (캡쳐한 이미지 이름으로 사용) 
			if self.once: # 프로그램 시작 후 한번만 실행
				subprocess.call([("rm -rf /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/data"%self.my_pc_name)],shell =True) #data파일 제거
				subprocess.call([("rm -rf /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/utils"%self.my_pc_name)],shell =True) #utils 파일 제거
				subprocess.call([("rm -rf /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/models"%self.my_pc_name)],shell =True) #models 파일 제거 
				rm_exp = subprocess.call([("rm -rf *")],cwd="/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/runs/train/"%self.my_pc_name,shell =True) # 파일 삭제
				rm_pt = subprocess.call([("rm best.pt")],cwd="/home/%s/catkin_ws/src/bitdog_ros/pytorch_yolov5/"%self.my_pc_name,shell =True) # 기존 pt파일 제거
				self.now_dir = ("%04d%02d%02d-%02d%02d%02d"%(now.tm_year,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec)) # 폴더 이름으로 사용하기 위한 현재 날짜와 시간
				
				self.createFolder('/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s'%(self.my_pc_name,self.now_dir)) 
				self.createFolder('/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/annotations'%(self.my_pc_name,self.now_dir))
				self.createFolder('/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/images'%(self.my_pc_name,self.now_dir))
				self.createFolder('/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/labels'%(self.my_pc_name,self.now_dir))
				
				copy2xml_yolo = subprocess.call([("cp /home/%s/catkin_ws/src/bitdog_ros/src/xml2yolo.py /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/xml2yolo.py"%(self.my_pc_name,self.my_pc_name,self.now_dir))],shell =True)
				
				yaml = open('/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/dataset.yaml'%(self.my_pc_name,self.now_dir), 'w')
				yaml.write("path: /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s\ntrain: /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/images\nval: /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/images\n\nnc: 1\nnames: ['user']"%(self.my_pc_name,self.now_dir,self.my_pc_name,self.now_dir,self.my_pc_name,self.now_dir))
				yaml.close()
					
				self.once = False
			len_file = ("/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/images"%(self.my_pc_name,self.now_dir))
			count_file = self.get_files_count(len_file)
			
			if count_file < 100:
				
				cv_image = self.cvBridge.imgmsg_to_cv2(image, "bgr8")#rgb8
				
				
				if self.success: # Qt에서 캡쳐버튼 누를때 마다 동작하는 조건
					
					self.capture_count += 1
					self.capture_count_pub.publish(self.capture_count)
					
					img_captured = cv2.imwrite("/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/images/%s.jpg"%(self.my_pc_name,self.now_dir,self.now_time), cv_image)
					
					f = open('/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/annotations/%s.xml'%(self.my_pc_name,self.now_dir,self.now_time), 'w')
					
					data = ("<annotation>\n	<filename>%s.jpg</filename>\n	<folder>%s</folder>\n	<source>\n		<database>%s</database>\n		<annotation>custom</annotation>\n		<image>custom</image>\n	</source>\n	<size>\n		<width>640</width>\n		<height>480</height>\n		<depth>3</depth>\n	</size>\n	<segmented>0</segmented>\n	<object>\n		<name>user</name>\n		<pose>unspecified</pose>\n		<truncated>0</truncated>\n		<difficult>0</difficult>\n		<bndbox>\n			<xmin>%s</xmin>\n			<ymin>%s</ymin>\n			<xmax>%s</xmax>\n			<ymax>%s</ymax>\n		</bndbox>\n	</object>\n</annotation>"%(self.now_time, self.now_dir, self.now_dir, 220, 70, 420, 270))
					f.write(data)
					f.close()
					
					self.success = False
				cv2.rectangle(cv_image,(220,70),(420,270), (0,255,0), 1) # 실시간 학습 데이터 촬영 시 고정된 박스
				
				self.pub_train_img.publish(self.cvBridge.cv2_to_imgmsg(cv_image,"bgr8"))

				if self.signal == 1: # signal 변수는 capture_signal 메서드에 위치
					self.success = True
					self.signal = 0
					
			elif count_file >= 100: # 100 == 캡쳐 이미지 개수 제한
				cv2.destroyAllWindows()
				print("next")
				xml2txt = subprocess.call([("python3 xml2yolo.py")],cwd="/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/%s/"%(self.my_pc_name,self.now_dir),shell =True)
				
				if xml2txt == 0: #0은 완료 신호
					print("SUCCESS")
					self.sequence = 1
					
					
				
		elif self.sequence == 1: # 학습 프로세스 시작
			if self.once == False:
				self.train = subprocess.call([("python3 train.py --data %s/dataset.yaml --batch-size 4 --epochs 10"%self.now_dir)],cwd="/home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/"%self.my_pc_name,shell =True)
				
				self.once = True
			elif self.train == 0:
				self.sequence = 2
				print("sequence == 2")
				
		elif self.sequence == 2: # best.pt 파일 다른 디렉토리에 복사
			copy2pt = subprocess.call([("cp /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/runs/train/exp/weights/best.pt /home/%s/catkin_ws/src/bitdog_ros/pytorch_yolov5/best.pt"%(self.my_pc_name,self.my_pc_name))],shell =True)
			if copy2pt == 0:
				#exit()
				self.sequence = 3
				print("sequence == 3")
				
		elif self.sequence == 3: # data,utils, models파일 복사
			subprocess.call([("cp -r /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/data /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/data"%(self.my_pc_name,self.my_pc_name))],shell =True)
			subprocess.call([("cp -r /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/utils /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/utils"%(self.my_pc_name,self.my_pc_name))],shell =True)
			subprocess.call([("cp -r /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/yolov5/models /home/%s/catkin_ws/src/bitdog_ros/src/ros_yolo/models"%(self.my_pc_name,self.my_pc_name))],shell =True)
			self.now_dir_pub.publish(self.now_dir)			
			sys.exit()
			
		
	def main(self):
		rospy.spin()
		
if __name__ == "__main__":
	rospy.init_node('object_train')
	node = camera()
	node.main()
	
