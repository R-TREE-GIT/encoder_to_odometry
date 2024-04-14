from PyQt5 import QtCore, QtGui, QtWidgets
import sys
from time import sleep
from PyQt5.QtWidgets import QComboBox, QApplication, QWidget, QVBoxLayout, QLabel, QHBoxLayout, QMainWindow, QPushButton, QGridLayout, QSizePolicy, QProgressBar, QStackedWidget, QTabWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import subprocess, pathlib

class core(QThread): #roscore
	def __init__(self):
		super(core, self).__init__()
		
	def run(self):
		try:
			self.core = subprocess.Popen([("roscore")], shell = True)
		except:
			rospy.loginfo("roscore is already running")

class hand_thread(QThread):
	def __init__(self):
		super(hand_thread, self).__init__()
		self.cwd = pathlib.Path.home().joinpath('catkin_ws', 'src', 'bitdog_ros', 'src')
		
	def run(self): # hand_thread.start 실행 시 run 메서드 실행
		self.hand = subprocess.Popen([("python3 hand_detect_ros.py")], cwd = self.cwd, shell = True)

	def stop(self): # hand_thread.stop 실행 시 해당 subprocess 강제 종료
		self.hand = subprocess.Popen([("rosnode kill hand_detect_ros")], shell = True)

class line_thread(QThread):
	def __init__(self):
		super(line_thread, self).__init__()
		self.cwd = pathlib.Path.home().joinpath('catkin_ws', 'src', 'bitdog_ros', 'src')

	def run(self):
		self.line = subprocess.Popen([("python3 line_detect_ros.py")], cwd = self.cwd, shell = True)

	def stop(self):
		self.line = subprocess.Popen([("rosnode kill line_detect_ros")], shell = True)

class basic_thread(QThread):
	def __init__(self):
		super(basic_thread, self).__init__()
		self.cwd = pathlib.Path.home().joinpath('catkin_ws', 'src', 'bitdog_ros', 'src')
		
	def run(self):
		self.basic = subprocess.call([("roslaunch yolov5_ros yolo_basic.launch")], cwd = self.cwd, shell = True)

	def stop(self):
		self.basic = subprocess.Popen([("rosnode kill basic_detect")], shell = True)

class object_train_thread(QThread):
	def __init__(self):
		super(object_train_thread, self).__init__()
		self.cwd = pathlib.Path.home().joinpath('catkin_ws', 'src', 'bitdog_ros', 'src')

	def run(self):
		self.object = subprocess.Popen([("python3 object_auto_train.py")], cwd = self.cwd, shell = True)

	def stop(self):
		subprocess.Popen([("rosnode kill object_train")], shell = True)

class object_detect_thread(QThread):
	def __init__(self):
		super(object_detect_thread, self).__init__()
		self.cwd = pathlib.Path.home().joinpath('catkin_ws', 'src', 'bitdog_ros', 'src')
		
	def run(self):
		self.object_detect = subprocess.Popen([("roslaunch yolov5_ros yolov5.launch")], cwd = self.cwd, shell = True)

	def stop(self):
		# launch 종료 시 파일명이 아닌 node이름 작성
		subprocess.Popen([("rosnode kill detect")], shell = True) 
		subprocess.Popen([("rosnode kill object_train")], shell = True)

class Thread(QtCore.QThread):
	change_pixmap_signal1 = QtCore.pyqtSignal(QtGui.QImage)
	change_pixmap_signal2 = QtCore.pyqtSignal(QtGui.QImage)
	change_pixmap_signal3 = QtCore.pyqtSignal(QtGui.QImage)
	change_pixmap_signal4 = QtCore.pyqtSignal(QtGui.QImage)
	change_pixmap_signal5 = QtCore.pyqtSignal(QtGui.QImage)
	scaled_size = QtCore.QSize(640, 480)
	bridge = CvBridge()

	# "/hand_pub" #from hand_detect_ros.py
	def callback1(self,data): 
		cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8") 
		height, width, channels = cv_image.shape
		bytes_per_line = channels * width
		qt_image = QtGui.QImage(cv_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
		qt_img = qt_image.scaled(self.scaled_size, QtCore.Qt.KeepAspectRatio)
		self.change_pixmap_signal1.emit(qt_img)

	# "/line_pub" # from line_detect_ros.py
	def callback2(self,data): 
		cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8") 
		height, width, channels = cv_image.shape
		bytes_per_line = channels * width
		qt_image = QtGui.QImage(cv_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
		qt_img = qt_image.scaled(self.scaled_size, QtCore.Qt.KeepAspectRatio)
		self.change_pixmap_signal2.emit(qt_img)

	# "/yolo_basic" # from /yolov5_ros/launch/yolo_basic.launch
	def callback3(self,data): 
		cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
		height, width, channels = cv_image.shape
		bytes_per_line = channels * width
		qt_image = QtGui.QImage(cv_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
		qt_img = qt_image.scaled(self.scaled_size, QtCore.Qt.KeepAspectRatio)
		self.change_pixmap_signal3.emit(qt_img)

	# "/train_cam" # from object_auto_train.py
	def callback4(self,data): 
		cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8") 
		height, width, channels = cv_image.shape
		bytes_per_line = channels * width
		qt_image = QtGui.QImage(cv_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
		qt_img = qt_image.scaled(self.scaled_size, QtCore.Qt.KeepAspectRatio)
		self.change_pixmap_signal4.emit(qt_img)

	# "/object_train" # from /yolov5_ros/launch/yolov5.launch
	def callback5(self,data): 
		cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8") 
		height, width, channels = cv_image.shape
		bytes_per_line = channels * width
		qt_image = QtGui.QImage(cv_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
		qt_img = qt_image.scaled(self.scaled_size, QtCore.Qt.KeepAspectRatio)
		self.change_pixmap_signal5.emit(qt_img)
	
	def run(self):
		#from hand_detect_ros.py
		img_topic1 = "/hand_pub" 
		self.image_sub1 = rospy.Subscriber(img_topic1, Image, self.callback1, queue_size = 1)
		
		# from line_detect_ros.py
		img_topic2 = "/line_pub" 
		self.image_sub2 = rospy.Subscriber(img_topic2, Image, self.callback2, queue_size = 1)
		
		# from /yolov5_ros/launch/yolo_basic.launch
		img_topic3 = "/yolo_basic" 
		self.image_sub3 = rospy.Subscriber(img_topic3, Image, self.callback3, queue_size = 1)
		
		# from object_auto_train.py
		img_topic4 = "/train_cam" 
		self.image_sub4 = rospy.Subscriber(img_topic4, Image, self.callback4, queue_size = 1)		
		
		# from /yolov5_ros/launch/yolov5.launch
		img_topic5 = "/object_train" 
		self.image_sub5 = rospy.Subscriber(img_topic5, Image, self.callback5, queue_size = 1)
		rospy.spin()

	def scaled(self, scaled_size):
		self.scaled_size = scaled_size

# 첫번째 탭에 들어갈 영상
class PlayStreaming1(QtWidgets.QLabel): 
	reSize = QtCore.pyqtSignal(QtCore.QSize)
	def __init__(self):
		super(PlayStreaming1, self).__init__()
		self.initUI()

	@QtCore.pyqtSlot(QtGui.QImage)
	def setImage1(self, image):
		self.label.setPixmap(QtGui.QPixmap.fromImage(image))

	def initUI(self):
		self.setWindowTitle("Image")
		self.label = QtWidgets.QLabel(self)
		self.th = Thread(self)
		self.th.change_pixmap_signal1.connect(self.setImage1)
		self.reSize.connect(self.th.scaled)
		self.th.start()
		lay = QtWidgets.QVBoxLayout(self)
		lay.addWidget(self.label, alignment=QtCore.Qt.AlignCenter)

	def resizeEvent(self, event):
		self.reSize.emit(self.size())

# 두번째 탭에 들어갈 영상
class PlayStreaming2(QtWidgets.QLabel): 
	reSize = QtCore.pyqtSignal(QtCore.QSize)
	def __init__(self):
		super(PlayStreaming2, self).__init__()
		self.initUI()

	@QtCore.pyqtSlot(QtGui.QImage)
	def setImage2(self, image):
		self.label.setPixmap(QtGui.QPixmap.fromImage(image))

	def initUI(self):
		self.setWindowTitle("Image")
		self.label = QtWidgets.QLabel(self)
		self.th = Thread(self)
		self.th.change_pixmap_signal2.connect(self.setImage2)
		self.reSize.connect(self.th.scaled)
		self.th.start()
		lay = QtWidgets.QVBoxLayout(self)
		lay.addWidget(self.label, alignment=QtCore.Qt.AlignCenter)

	def resizeEvent(self, event):
		self.reSize.emit(self.size())

# 세번째 탭에 들어갈 영상
class PlayStreaming3(QtWidgets.QLabel): 
	reSize = QtCore.pyqtSignal(QtCore.QSize)
	def __init__(self):
		super(PlayStreaming3, self).__init__()
		self.initUI()

	@QtCore.pyqtSlot(QtGui.QImage)
	def setImage3(self, image):
		self.label.setPixmap(QtGui.QPixmap.fromImage(image))

	def initUI(self):
		self.setWindowTitle("Image")
		self.label = QtWidgets.QLabel(self)
		self.th = Thread(self)
		self.th.change_pixmap_signal3.connect(self.setImage3)
		self.reSize.connect(self.th.scaled)
		self.th.start()
		lay = QtWidgets.QVBoxLayout(self)
		lay.addWidget(self.label, alignment=QtCore.Qt.AlignCenter)

	def resizeEvent(self, event):
		self.reSize.emit(self.size())

# 네번째 탭에 들어갈 영상
class PlayStreaming4(QtWidgets.QLabel): 
	reSize = QtCore.pyqtSignal(QtCore.QSize)
	def __init__(self):
		super(PlayStreaming4, self).__init__()
		self.initUI()

	@QtCore.pyqtSlot(QtGui.QImage)
	def setImage4(self, image):
		self.label.setPixmap(QtGui.QPixmap.fromImage(image))

	def initUI(self):
		self.setWindowTitle("Image")
		self.label = QtWidgets.QLabel(self)
		self.th = Thread(self)
		self.th.change_pixmap_signal4.connect(self.setImage4)
		self.reSize.connect(self.th.scaled)
		self.th.start()
		lay = QtWidgets.QVBoxLayout(self)
		lay.addWidget(self.label, alignment=QtCore.Qt.AlignCenter)

	def resizeEvent(self, event):
		self.reSize.emit(self.size())

# 다섯번째 탭에 들어갈 영상
class PlayStreaming5(QtWidgets.QLabel): 
	reSize = QtCore.pyqtSignal(QtCore.QSize)
	def __init__(self):
		super(PlayStreaming5, self).__init__()
		self.initUI()

	@QtCore.pyqtSlot(QtGui.QImage)
	def setImage5(self, image):
		self.label.setPixmap(QtGui.QPixmap.fromImage(image))

	def initUI(self):
		self.setWindowTitle("Image")
		self.label = QtWidgets.QLabel(self)
		self.th = Thread(self)
		self.th.change_pixmap_signal5.connect(self.setImage5)
		self.reSize.connect(self.th.scaled)
		self.th.start()
		lay = QtWidgets.QVBoxLayout(self)
		lay.addWidget(self.label, alignment=QtCore.Qt.AlignCenter)

	def resizeEvent(self, event):
		self.reSize.emit(self.size())

class UIWidget(QtWidgets.QWidget):
	def __init__(self, parent=None):
		super(UIWidget, self).__init__(parent)

		# object_auto_train.py에 capture 신호 pub
		self.capture_count_pub = rospy.Publisher('/capture', Int8, queue_size =1) 
		
		# 사용자가 지정한 xgo action 이름 pub
		self.zero_pub = rospy.Publisher('/zero', String, queue_size =1) 
		self.one_pub = rospy.Publisher('/one', String, queue_size =1)
		self.two_pub = rospy.Publisher('/two', String, queue_size =1)
		self.three_pub = rospy.Publisher('/three', String, queue_size =1)
		self.four_pub = rospy.Publisher('/four', String, queue_size =1)
		self.five_pub = rospy.Publisher('/five', String, queue_size =1)

		self.hand_thread = hand_thread()
		self.line_thread = line_thread()
		self.basic_thread = basic_thread()
		self.object_train_thread = object_train_thread()
		self.object_detect_thread = object_detect_thread()

		# Initialize tab screen
		self.tabs = QtWidgets.QTabWidget()
		self.tab1 = QtWidgets.QWidget()
		self.tab2 = QtWidgets.QWidget()
		self.tab3 = QtWidgets.QWidget()
		self.tab4 = QtWidgets.QWidget()
		self.tab5 = QtWidgets.QWidget()

		self.a = 0

		# Add tabs
		self.tabs.addTab(self.tab1, "HAND")
		self.tabs.addTab(self.tab2, "LINE")
		self.tabs.addTab(self.tab3, "BASIC")
		self.tabs.addTab(self.tab4, "OBJECT_TRAIN")
		self.tabs.addTab(self.tab5, "OBJECT_DETECT")

		# Create first tab
		self.createGridLayout_1()
		self.tab1.layout = QtWidgets.QHBoxLayout()
		self.display = PlayStreaming1()
		self.tab1.layout.addWidget(self.display, stretch=1)
		self.tab1.layout.addWidget(self.horizontalGroupBox)
		self.tab1.setLayout(self.tab1.layout)

		# Create second tab
		self.createGridLayout_2()
		self.tab2.layout = QtWidgets.QHBoxLayout()
		self.display = PlayStreaming2()
		self.tab2.layout.addWidget(self.display, stretch=1)
		self.tab2.layout.addWidget(self.horizontalGroupBox)
		self.tab2.setLayout(self.tab2.layout)

		# Create third tab
		self.createGridLayout_3()
		self.tab3.layout = QtWidgets.QHBoxLayout()
		self.display = PlayStreaming3()
		self.tab3.layout.addWidget(self.display, stretch=1)
		self.tab3.layout.addWidget(self.horizontalGroupBox)
		self.tab3.setLayout(self.tab3.layout)

		# Create fourth tab
		self.createGridLayout_4()
		self.tab4.layout = QtWidgets.QHBoxLayout()
		self.display = PlayStreaming4()
		self.tab4.layout.addWidget(self.display, stretch=1)
		self.tab4.layout.addWidget(self.horizontalGroupBox)
		self.tab4.setLayout(self.tab4.layout)

		# Create fifth tab
		self.createGridLayout_5()
		self.tab5.layout = QtWidgets.QHBoxLayout()
		self.display = PlayStreaming5()
		self.tab5.layout.addWidget(self.display, stretch=1)
		self.tab5.layout.addWidget(self.horizontalGroupBox)
		self.tab5.setLayout(self.tab5.layout)

		# Add tabs to widget
		layout = QtWidgets.QVBoxLayout(self)
		layout.addWidget(self.tabs)

	# 첫번째 탭의 레이아웃
	def createGridLayout_1(self): 
		self.horizontalGroupBox = QtWidgets.QGroupBox("Control")
		self.horizontalGroupBox.setStyleSheet("QGroupBox { background-color: white}");

		# 선택 창 위젯 설정
		self.ls = QComboBox(self) 
		self.ls.addItem('select action')
		self.ls.addItem('Lie down')
		self.ls.addItem('Stand up')
		self.ls.addItem('Crawl')
		self.ls.addItem('Turn left')
		self.ls.addItem('Turn right')
		self.ls.addItem('Squat')
		self.ls.addItem('Turn roll')
		self.ls.addItem('Turn pitch')
		self.ls.addItem('Turn yaw')
		self.ls.addItem('3 axis motion')
		self.ls.addItem('Take a pee')
		self.ls.addItem('Sit down')
		self.ls.addItem('Wave hand')
		self.ls.addItem('Give a stretch')
		self.ls.addItem('Wave body')
		self.ls.addItem('Wave side')
		self.ls.addItem('Pray')
		self.ls.addItem('Looking for food')
		self.ls.addItem('Handshake')
		self.ls.activated[str].connect(self.zero)

		self.ls1 = QComboBox(self)
		self.ls1.addItem('select action')
		self.ls1.addItem('Lie down')
		self.ls1.addItem('Stand up')
		self.ls1.addItem('Crawl')
		self.ls1.addItem('Turn left')
		self.ls1.addItem('Turn right')
		self.ls1.addItem('Squat')
		self.ls1.addItem('Turn roll')
		self.ls1.addItem('Turn pitch')
		self.ls1.addItem('Turn yaw')
		self.ls1.addItem('3 axis motion')
		self.ls1.addItem('Take a pee')
		self.ls1.addItem('Sit down')
		self.ls1.addItem('Wave hand')
		self.ls1.addItem('Give a stretch')
		self.ls1.addItem('Wave body')
		self.ls1.addItem('Wave side')
		self.ls1.addItem('Pray')
		self.ls1.addItem('Looking for food')
		self.ls1.addItem('Handshake')
		self.ls1.activated[str].connect(self.one)

		self.ls2 = QComboBox(self)
		self.ls2.addItem('select action')
		self.ls2.addItem('Lie down')
		self.ls2.addItem('Stand up')
		self.ls2.addItem('Crawl')
		self.ls2.addItem('Turn left')
		self.ls2.addItem('Turn right')
		self.ls2.addItem('Squat')
		self.ls2.addItem('Turn roll')
		self.ls2.addItem('Turn pitch')
		self.ls2.addItem('Turn yaw')
		self.ls2.addItem('3 axis motion')
		self.ls2.addItem('Take a pee')
		self.ls2.addItem('Sit down')
		self.ls2.addItem('Wave hand')
		self.ls2.addItem('Give a stretch')
		self.ls2.addItem('Wave body')
		self.ls2.addItem('Wave side')
		self.ls2.addItem('Pray')
		self.ls2.addItem('Looking for food')
		self.ls2.addItem('Handshake')
		self.ls2.activated[str].connect(self.two)

		self.ls3 = QComboBox(self)
		self.ls3.addItem('select action')
		self.ls3.addItem('Lie down')
		self.ls3.addItem('Stand up')
		self.ls3.addItem('Crawl')
		self.ls3.addItem('Turn left')
		self.ls3.addItem('Turn right')
		self.ls3.addItem('Squat')
		self.ls3.addItem('Turn roll')
		self.ls3.addItem('Turn pitch')
		self.ls3.addItem('Turn yaw')
		self.ls3.addItem('3 axis motion')
		self.ls3.addItem('Take a pee')
		self.ls3.addItem('Sit down')
		self.ls3.addItem('Wave hand')
		self.ls3.addItem('Give a stretch')
		self.ls3.addItem('Wave body')
		self.ls3.addItem('Wave side')
		self.ls3.addItem('Pray')
		self.ls3.addItem('Looking for food')
		self.ls3.addItem('Handshake')
		self.ls3.activated[str].connect(self.three)

		self.ls4 = QComboBox(self)
		self.ls4.addItem('select action')
		self.ls4.addItem('Lie down')
		self.ls4.addItem('Stand up')
		self.ls4.addItem('Crawl')
		self.ls4.addItem('Turn left')
		self.ls4.addItem('Turn right')
		self.ls4.addItem('Squat')
		self.ls4.addItem('Turn roll')
		self.ls4.addItem('Turn pitch')
		self.ls4.addItem('Turn yaw')
		self.ls4.addItem('3 axis motion')
		self.ls4.addItem('Take a pee')
		self.ls4.addItem('Sit down')
		self.ls4.addItem('Wave hand')
		self.ls4.addItem('Give a stretch')
		self.ls4.addItem('Wave body')
		self.ls4.addItem('Wave side')
		self.ls4.addItem('Pray')
		self.ls4.addItem('Looking for food')
		self.ls4.addItem('Handshake')
		self.ls4.activated[str].connect(self.four)

		self.ls5 = QComboBox(self)
		self.ls5.addItem('select action')
		self.ls5.addItem('Lie down')
		self.ls5.addItem('Stand up')
		self.ls5.addItem('Crawl')
		self.ls5.addItem('Turn left')
		self.ls5.addItem('Turn right')
		self.ls5.addItem('Squat')
		self.ls5.addItem('Turn roll')
		self.ls5.addItem('Turn pitch')
		self.ls5.addItem('Turn yaw')
		self.ls5.addItem('3 axis motion')
		self.ls5.addItem('Take a pee')
		self.ls5.addItem('Sit down')
		self.ls5.addItem('Wave hand')
		self.ls5.addItem('Give a stretch')
		self.ls5.addItem('Wave body')
		self.ls5.addItem('Wave side')
		self.ls5.addItem('Pray')
		self.ls5.addItem('Looking for food')
		self.ls5.addItem('Handshake')
		self.ls5.activated[str].connect(self.five)

		self.zero_label = QLabel('ZERO :', self)
		self.one_label = QLabel('ONE :', self)
		self.two_label = QLabel('TWO :', self)
		self.three_label = QLabel('THREE :', self)
		self.four_label = QLabel('FOUR :', self)
		self.five_label = QLabel('FIVE :', self)

		self.start = QPushButton('START', self)
		self.exit = QPushButton('EXIT', self)

		# 각 위젯의 레이아웃 지정
		layout = QGridLayout()
		layout.addWidget(self.zero_label,0,0)
		layout.addWidget(self.ls,0,1)
		layout.addWidget(self.one_label,1,0)
		layout.addWidget(self.ls1,1,1)
		layout.addWidget(self.two_label,2,0)
		layout.addWidget(self.ls2,2,1)
		layout.addWidget(self.three_label,3,0)
		layout.addWidget(self.ls3,3,1)
		layout.addWidget(self.four_label,4,0)
		layout.addWidget(self.ls4,4,1)
		layout.addWidget(self.five_label,5,0)
		layout.addWidget(self.ls5,5,1)

		layout.addWidget(self.start, 6, 0) #hand_detect launch start
		layout.addWidget(self.exit, 6, 1) #hand detct exit

		self.start.clicked.connect(self.hand_start)
		self.exit.clicked.connect(self.hand_exit)

		self.horizontalGroupBox.setLayout(layout)

	# 항목 선택 시 실행 (선택 한 text publish)
	def zero(self,text):
		self.zero_pub.publish(text)
	def one(self,text):
		self.one_pub.publish(text)
	def two(self,text):
		self.two_pub.publish(text)
	def three(self,text):
		self.three_pub.publish(text)
	def four(self,text):
		self.four_pub.publish(text)
	def five(self,text):
		self.five_pub.publish(text)

	# start 버튼 클릭 시 hand_thread의 run 메서드 실행
	def hand_start(self):
		self.hand_thread.start()
		self.start.setDisabled(True) # start 버튼 비활성화
		
	def hand_exit(self):
		self.hand_thread.stop()
		self.start.setDisabled(False) # start 버튼 활성화

	# 두번째 탭의 레이아웃
	def createGridLayout_2(self):
		self.horizontalGroupBox = QtWidgets.QGroupBox("Control")
		self.horizontalGroupBox.setStyleSheet("QGroupBox { background-color: white}");

		self.start = QPushButton('START', self)
		self.exit = QPushButton('EXIT', self)

		layout = QGridLayout()
		layout.addWidget(self.start, 0, 0)
		layout.addWidget(self.exit, 0, 1)

		self.start.clicked.connect(self.line_start)
		self.exit.clicked.connect(self.line_exit)

		self.horizontalGroupBox.setLayout(layout)

	def line_start(self):
		self.line_thread.start()
		self.start.setDisabled(True)
		
	def line_exit(self):
		self.line_thread.stop()
		self.start.setDisabled(False)

	def createGridLayout_3(self):
		self.horizontalGroupBox = QtWidgets.QGroupBox("Control")
		self.horizontalGroupBox.setStyleSheet("QGroupBox { background-color: white}");

		self.start = QPushButton('START', self)
		self.exit = QPushButton('EXIT', self)

		layout = QGridLayout()
		layout.addWidget(self.start, 0, 0)
		layout.addWidget(self.exit, 0, 1) 

		self.start.clicked.connect(self.basic_start)
		self.exit.clicked.connect(self.basic_exit)

		self.horizontalGroupBox.setLayout(layout)

	def basic_start(self):
		self.basic_thread.start()
		self.start.setDisabled(True)
	
	def basic_exit(self):
		self.basic_thread.stop()
		self.start.setDisabled(False)

	def createGridLayout_4(self):
		self.horizontalGroupBox = QtWidgets.QGroupBox("Control")
		self.horizontalGroupBox.setStyleSheet("QGroupBox { background-color: white}");

		self.train = QPushButton('train_tool_start', self)
		self.capture = QPushButton('capture', self)
		self.stop = QPushButton('RE TRAIN', self)
		self.progress = QProgressBar()

		layout = QGridLayout()
		layout.addWidget(self.train, 0, 0)
		layout.addWidget(self.capture, 0, 1)
		layout.addWidget(self.stop, 1, 0)
		layout.addWidget(self.progress, 1, 1)

		self.train.clicked.connect(self.train_start)
		self.capture.clicked.connect(self.train_capture)
		self.stop.clicked.connect(self.train_stop)
		
		self.horizontalGroupBox.setLayout(layout)

	def train_stop(self):
		self.object_train_thread.stop()
		self.train.setDisabled(False)
	
	# capture 버튼 클릭 시 실행 (+progress bar)
	def train_capture(self):
		self.a += 1
		self.progress.setValue(self.a) # progress bar update(++1)
		self.capture_signal = 1
		self.capture_count_pub.publish(self.capture_signal) # capture signal pub 
		self.capture_signal = 0
	
	def train_start(self):
		self.object_train_thread.start()
		self.train.setDisabled(True)

	def createGridLayout_5(self):
		self.horizontalGroupBox = QtWidgets.QGroupBox("Control")
		self.horizontalGroupBox.setStyleSheet("QGroupBox { background-color: white}");

		self.detect_btn = QPushButton('DETECT', self)
		self.stop = QPushButton('STOP', self)

		layout = QGridLayout()
		layout.addWidget(self.detect_btn, 0, 0)
		layout.addWidget(self.stop, 0, 1)

		self.detect_btn.clicked.connect(self.detect_start)
		self.stop.clicked.connect(self.detect_stop)
		
		self.horizontalGroupBox.setLayout(layout)

	def detect_stop(self):
		self.object_detect_thread.stop()
		self.train.setDisabled(False)
		
	def detect_start(self):
		self.object_detect_thread.start()
		self.train.setDisabled(True)

if __name__ == '__main__':
	import sys
	core = core()
	core.start()
	rospy.init_node('bitdog', anonymous=False, disable_signals=True)
	app = QtWidgets.QApplication(sys.argv)
	w = UIWidget()
	w.resize(1000, 800)
	w.show()
	sys.exit(app.exec_())
