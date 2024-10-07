#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from math import cos, sin, pi

# Define variables for encoder ticks and wheelbase
left_ticks = 0
right_ticks = 0
prev_left_ticks = 0
prev_right_ticks = 0

ENCODER_PPR = 16384
WHEEL_RADIUS = 0.085
WHEEL_LENGTH = 0.465

LEFT = 0
RIGHT = 1
WHEEL_NUM = 2

# Define variables for current pose and orientation
current_x = 0.0
current_y = 0.0
current_theta = 0.0

last_time = 0.0
curr_time = 0.0

joint_states_vel = [0.0, 0.0]
joint_states_pos = [0.0, 0.0]
joint_state_header_frame_id = "base_link"

odom = Odometry()
joint_states = JointState()

def left_ticks_callback(msg):
    global left_ticks
    left_ticks = msg.data

def right_ticks_callback(msg):
    global right_ticks
    right_ticks = msg.data

def publish_odometry():
    global prev_left_ticks, prev_right_ticks, current_x, current_y, current_theta, odom, last_time, curr_time, joint_states_pos, joint_states_vel
    curr_time = rospy.Time.now()
    dt = (curr_time - last_time).to_sec();
    curr_left_ticks = left_ticks - prev_left_ticks
    curr_right_ticks = right_ticks - prev_right_ticks
    prev_left_ticks = left_ticks
    prev_right_ticks = right_ticks
    l_revolutions = curr_left_ticks / ENCODER_PPR
    left_distance = 2 * pi * WHEEL_RADIUS * l_revolutions
    r_revolutions = curr_right_ticks / ENCODER_PPR
    right_distance = 2 * pi * WHEEL_RADIUS * r_revolutions
    
    joint_states_vel[LEFT] = (l_revolutions * 2 * pi) / dt
    joint_states_vel[RIGHT] = (r_revolutions * 2 * pi) / dt
    joint_states_pos[LEFT] += (l_revolutions * 2 * pi)
    joint_states_pos[RIGHT] +=  (r_revolutions * 2 * pi)

    # Calculate change in position and orientation
    delta_theta = (right_distance - left_distance) / WHEEL_LENGTH
    delta_x = (left_distance + right_distance) / 2.0 * cos(current_theta)
    delta_y = (left_distance + right_distance) / 2.0 * sin(current_theta)

    # Update current pose and orientation
    current_x += delta_x
    current_y += delta_y
    current_theta += delta_theta

    # Create Odometry message
    odom.pose.pose.position.x = current_x
    odom.pose.pose.position.y = current_y
    odom.pose.pose.orientation.z = sin(current_theta / 2.0)
    odom.pose.pose.orientation.w = cos(current_theta / 2.0)
    odom.twist.twist.linear.x = ((left_distance + right_distance) / 2.0) / dt
    odom.twist.twist.angular.z = delta_theta / dt
    last_time = curr_time
    
def initJointStates():
    global joint_states
    joint_states_name = {"wheel_left_joint", "wheel_right_joint"}
    
    joint_states.header.frame_id = joint_state_header_frame_id
    joint_states.name            = joint_states_name

    
def updateJointStates():
    global joint_states
    joint_states.position = joint_states_pos
    joint_states.velocity = joint_states_vel
    
def updateTF():
    global odom_tf
    odom_tf.header = odom.header
    odom_tf.child_frame_id = odom.child_frame_id
    odom_tf.transform.translation.x = odom.pose.pose.position.x
    odom_tf.transform.translation.y = odom.pose.pose.position.y
    odom_tf.transform.translation.z = odom.pose.pose.position.z
    odom_tf.transform.rotation      = odom.pose.pose.orientation
    
def main():
    global odom, last_time, curr_time

    rospy.init_node('encoder_to_odometry')
    last_time = rospy.Time.now()
    curr_time = last_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"
    
    initJointStates()
    # Subscribe to left and right ticks topics
    rospy.Subscriber('mdh250_l_en', Int32, left_ticks_callback)
    rospy.Subscriber('mdh250_r_en', Int32, right_ticks_callback)

    # Create Odometry publisher
    odom_pub = rospy.Publisher('/md/data/odom', Odometry, queue_size=10)
    joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(30)  # Publish at 30 Hz

    while not rospy.is_shutdown():
        publish_odometry()
        updateJointStates()
        odom.header.stamp = rospy.Time.now()
        odom_pub.publish(odom)
        joint_states.header.stamp = rospy.Time.now()
        joint_states_pub.publish(joint_states)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
