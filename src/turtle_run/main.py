#! /usr/bin/python

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from numpy import arctan2
from math import pi
from math import hypot

class Main:
	def __init__(self):
		self.surv_pose = Pose()
		self.surv_sub = rospy.Subscriber('/michelangelo/pose', Pose, self.set_surv_pose)
		self.runner_pose = Pose()
		self.runner_sub = rospy.Subscriber('/raphael/pose', Pose, self.set_runner_pose)
		self.runner_pub = rospy.Publisher('/raphael/cmd_vel', Twist, queue_size = 1)

	def set_surv_pose(self, pose):
		self.surv_pose = pose

	def set_runner_pose(self, pose):
		self.runner_pose = pose

	def run(self):
		while not rospy.is_shutdown(): 
			if self.get_distance() < 0.3 :
				rospy.logerr('Mike I caught you! Where is all the pizza?')
				msg = Twist()
				msg.linear.x = 0.0
				self.runner_pub.publish(msg)
			else:
				angle_to_surv = self.get_angle_to_surv()
				msg = Twist()
				msg.angular.z = angle_to_surv
				msg.linear.x = self.get_speed(angle_to_surv)
				self.runner_pub.publish(msg)

	def get_speed(self, angle_to_surv):
		max_speed = 1
		return max_speed * (pi - abs(angle_to_surv)) / pi
		
	def get_distance(self):
		return hypot(self.surv_pose.x - self.runner_pose.x, self.surv_pose.y - self.runner_pose.y)

	def get_angle_to_surv(self):
		direction = self.runner_pose.theta
		surv = arctan2(self.surv_pose.y - self.runner_pose.y, self.surv_pose.x - self.runner_pose.x)
		if direction >= 0:
			if surv >= direction:
				return surv - direction
			else:
				if direction - pi <= surv:
					return surv - direction
				else:
					return surv - direction + 2 * pi
		else:
			if surv <= direction:
				return surv - direction
			else:
				if direction + pi >= surv:
					return surv - direction
				else:
					return surv - direction - 2 * pi
rospy.init_node('runner')
rospy.wait_for_service('/spawn')
spawn_func = rospy.ServiceProxy('/spawn', Spawn)
res = spawn_func(2.0, 2.0, 0.0, 'raphael')
m = Main()
m.run()

