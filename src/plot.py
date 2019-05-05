#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy, sys
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

class plot_pub():
	def __init__(self):

		self.goalpos = Pose2D()
		self.currentpos = Pose2D()

		self.x_goal = []
		self.y_goal = []

		self.x_current = []
		self.y_current = []

		rospy.Subscriber('/turtlebot/path', Pose2D, self.plot_pub)
		rospy.Subscriber('/odom', Odometry, self.get_currentpos)

	def get_currentpos(self, Odometry):
		self.currentpos.x = Odometry.pose.pose.position.x
		self.currentpos.y = Odometry.pose.pose.position.y

	def plot_pub(self, Pose2D):
		self.goalpos.x = Pose2D.x
		self.goalpos.y = Pose2D.y

		self.x_goal.append(self.goalpos.x)
		self.y_goal.append(self.goalpos.y)

		self.x_current.append(self.currentpos.x)
		self.y_current.append(self.currentpos.y)

		plt.cla()
		plt.legend(('Desired path', 'Actual path'), loc='upper right')
		plt.xlabel('x coordinate')
		plt.ylabel('y coordinate')
		plt.grid(True)
		plt.title('Tracking Performance')
		# plt.plot(self.x_goal[-2:], self.y_goal[-2:], 'ro')
		plt.plot(self.x_goal, self.y_goal, 'r')
		if len(self.x_current) > 500:
			plt.plot(self.x_current[-500:], self.y_current[-500:], 'b')
		else:
			plt.plot(self.x_current, self.y_current, 'b')
		# plt.plot(self.x_current, self.y_current, 'b')
		plt.draw()
		plt.pause(0.0001)


if __name__ == '__main__':
	rospy.init_node('plot')
	plot_test = plot_pub()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
	    pass
