#!/usr/bin/env python

import rospy
import roslib
import numpy as np
import sys
import math

from discretePolicyTranslator import discretePolicyTranslator
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import std_msgs.msg
import tf

from pose import Pose

"""
The goalHandler class publishes goal poses to the navigation stack via *robot_name*/move_base_simple/goal.

An instance of goalHandler initilizes a ROS node that published to *robot_name*/move_base_simple/goal, subscribes to
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
		self.stuck_buffer = 10
		self.stuck_count = self.stuck_buffer
		self.dpt = discretePolicyTranslator(filename)
		self.pose = Pose('',[0,0,0],'tf',None)
		self.current_status = 3
		self.tf_exist = False
		self.tf_exception_wrapper()
		self.goal_point = self.get_new_goal(self.pose._pose)
		self.pub = rospy.Publisher('/deckard/move_base_simple/goal',PoseStamped,queue_size=10)
		rospy.sleep(1) #<>TODO: figure out why the hell this works --> solves issue where robot would not move on initialization
		rospy.Subscriber('/deckard/move_base/status',GoalStatusArray,self.callback)
		print("initial position: " + str(self.pose._pose))

	def tf_exception_wrapper(self):
		#waits for transforms to become available and handles interim exceptions
		tries = 0
		while not self.tf_exist and tries < 10:
			try: 
				self.pose.tf_update()	
				self.tf_exist = True
				#self.current_position = self.pose._pose
				#self.goal_point = self.pose._pose
			except tf.LookupException as error:
				tries = tries + 1
				self.tf_exist = False
				print("\nError!")
				print(error)
				print("Waiting for transforms to become available. Will retry 10 times.") 
				print("Try: " + str(tries) + "  Retrying in 2 seconds.\n")
				rospy.sleep(2)		

	def callback(self,msg):
		self.last_position = self.pose._pose
		self.pose.tf_update()
		self.stuck_distance = math.sqrt((self.pose._pose[0] - self.last_position[0]) ** 2 + (self.pose._pose[1] - self.last_position[1]) ** 2)
		self.is_stuck()
		self.is_at_goal()		
		print("status: " + str(self.current_status) + "\tcheck: " + str(self.current_status==3) + "\tcurrent position: " + str(self.pose._pose))
		if self.current_status == 3:
			self.send_goal()
			self.stuck_count = self.stuck_buffer
			self.current_status = 1
		rospy.sleep(1)	

	def is_at_goal(self):
		#checks if robot has arrived at its goal pose
		tol = 0.25
		print("Checking if arrived at goal")
		try:
			#print("X goal diff: " + str(abs(self.goal_point[0] - self.current_position[0])) + "\tY goal diff: " + str(abs(self.goal_point[1] - self.current_position[1])))
			if abs(self.goal_point[0] - self.pose._pose[0]) < tol and abs(self.goal_point[1] - self.pose._pose[1]) < tol:
				self.current_status = 3
		except TypeError:
			print("Goal pose does not yet exist!")
			self.current_status = 3	

	def is_stuck(self):
		#re-sends goal pose if robot is mentally or physically stuck for 10 iterations
		if self.stuck_count > 0: #check buffer
			self.stuck_count += -1
			return False #return not stuck
		self.stuck_count = self.stuck_buffer
		if self.stuck_distance < 0.5:
			print("Robot stuck; resending goal.")
			self.send_goal()
			return True
		else:
			return False	

	def get_new_goal(self,current_position):
		#get new goal pose from module
		point = current_position
		#if self.current_status == 3:
		point[0] = round(point[0])
		point[1] = round(point[1])
		print(self.dpt.getNextPose(point))
		return self.dpt.getNextPose(point)

	def send_goal(self):
		#get and send new goal pose
		self.goal_point = self.get_new_goal(self.pose._pose)
		print("sent goal: " + str(self.goal_point))

		new_goal = PoseStamped()
		new_goal.pose.position.x = self.goal_point[0]
		new_goal.pose.position.y = self.goal_point[1]
		new_goal.pose.position.z = self.goal_point[2]
		theta = self.goal_point[3]

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