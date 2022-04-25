#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import Quaternion
from math import atan2
from nmap import *
import sys


def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


if __name__ == "__main__" :
	m,am = create_map()
	global cll
	global rr
	start = []
	global goal
	# import pdb; pdb.set_trace()
	start,goal,L,sa,ga=get_input(sys.argv[1:],m,am)
	root = Node(start, sa, 0 , None, 0, start)
	F,C,O,Pxy = DS(root,goal,L,ga,m,am)
	p=reverse_path(F,m,am)
	Vig(O,Pxy,p,m,am)	

	rospy.init_node("speed_controller")
	global x
	global y
	global theta 
	x = 0
	y = 0
	theta = 0
	sub = rospy.Subscriber("/odom", Odometry, newOdom)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	speed = Twist()

	r = rospy.Rate(4)
	
	goal = Point()
	for point in p :
		goal_x = (point.d[0] - 500 )/100
		goal_y = (point.d[1] - 500 )/100

		inc_x,inc_y = 100,100
		while (inc_x > 0.01 and inc_y > 0.01):
			inc_x = (goal_x) - (x) 
			inc_y = (goal_y) - (y) 
			rospy.loginfo(f"X : {goal_x}, Y : {goal_y}")
			rospy.loginfo(f"Odom X : {x}, OdomY : {y}")

			angle_to_goal = atan2(inc_y, inc_x)

			rospy.loginfo(f"T : {angle_to_goal}")

			if (angle_to_goal - theta) > 0.3:
				speed.linear.x = 0.2
				speed.angular.z = 0.5

			elif (angle_to_goal - theta) < -0.3:
				speed.linear.x = 0.2
				speed.angular.z = -0.5
			else:
				speed.linear.x = 0.5
				speed.angular.z = 0.0

			pub.publish(speed)
			r.sleep()
	speed.linear.x = 0
	speed.angular.z = 0
	pub.publish(speed)

