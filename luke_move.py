#!/usr/bin/env python

import rospy
import roslib
import numpy as np
import sys

from discretePolicyTranslator import discretePolicyTranslator
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import std_msgs.msg
import tf

from pose import Pose

"""
The goalHandler class publishes goal poses to the navigation stack via *robot_name*/move_base_simple/goal.

An instance of goalHandler initilizes a ROS node that published to /move_base_simple/goal, subscribes to
*robot_name*/move_base/status for callbacks, and listens to tf transforms using an instance of the "pose" class.

The goal poses come from discretePolicyTranslator.py, which takes the current xy position and returns the desired
xy position and orientation.

Input
-----------
filename: .txt file
	Text file containing alpha vectors for use in discretePolicyTranslator

Output
-----------
None	

"""

class goalHandler(object):

	def __init__(self, filename):
		rospy.init_node('goal_sender')
		self.pose = Pose('',[0,0,0],'tf',None)
		self.current_status = None
		self.tf_exist = False
		self.current_position = None
		self.goal_point = None
		self.tf_exception_wrapper()
		self.pub = rospy.Publisher('/deckard/move_base_simple/goal',PoseStamped,queue_size=10)
		rospy.Subscriber('/deckard/move_base/status',GoalStatusArray,self.callback)
		self.dpt = discretePolicyTranslator(filename)
		print("initial position: " + str(self.pose._pose))

	def tf_exception_wrapper(self):
		tries = 0
		while not self.tf_exist and tries < 10:
			try: 
				self.pose.tf_update()	
				self.tf_exist = True
				self.current_position = self.pose._pose
				self.goal_point = self.pose._pose
			except tf.LookupException as error:
				tries = tries + 1
				print("\nError!")
				print(error)
				print("Waiting for transforms to become available. Will retry 10 times.") 
				print("Try: " + str(tries) + "  Retrying in 2 seconds.\n")
				rospy.sleep(2)		

	def callback(self,msg):
		self.pose.tf_update()
		self.current_position = self.pose._pose
		self.is_at_goal()		
		print("status: " + str(self.current_status) + "\tcheck: " + str(self.current_status==3) + "\tcurrent position: " + str(self.current_position))
		if self.current_status == 3:
			self.send_goal()
			self.current_status = 1
		rospy.sleep(1)	

	def is_at_goal(self):
		if abs(self.goal_point[0] - self.current_position[0]) < 0.25 and abs(self.goal_point[1] - self.current_position[1]) < 0.25:
			self.current_status = 3

	def get_new_goal(self,current_position):
		point = current_position
		if self.current_status == 3:
			point[0] = round(point[0])
			point[1] = round(point[1])
		print(self.dpt.getNextPose(point))
		return self.dpt.getNextPose(point)

	def send_goal(self):
		self.goal_point = self.get_new_goal(self.current_position)
		print("sent goal: " + str(self.goal_point))

		new_goal = PoseStamped()
		new_goal.pose.position.x = self.goal_point[0]
		new_goal.pose.position.y = self.goal_point[1]
		new_goal.pose.position.z = self.goal_point[2]
		theta = self.goal_point[3]
		'''if self.goal_point[0] - self.current_position[0] > 0.1:
			theta = 0
		elif self.goal_point[0] - self.current_position[0] < -0.1:
			theta = 180
		elif self.goal_point[1] - self.current_position[1] > 0.1:
			theta = 90
		elif self.goal_point[1] - self.current_position[1] < -0.1:
			theta = -90		
		theta = 0	'''
		quat = tf.transformations.quaternion_from_euler(0,0,np.deg2rad(theta))

		new_goal.pose.orientation.x = quat[0]
		new_goal.pose.orientation.y = quat[1]
		new_goal.pose.orientation.z = quat[2]
		new_goal.pose.orientation.w = quat[3] #<>TODO change this to variable orientation so robot doesn't spin as it travels
		#<>NOTE: random spinning that occured in gazebo sim does not occur in when run of physical robot

		new_goal.header.stamp = rospy.Time.now()
		new_goal.header.frame_id = 'map'

		self.pub.publish(new_goal)

if __name__ == "__main__":
	gh = goalHandler(sys.argv[1])
	
	rospy.spin()