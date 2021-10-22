#!/usr/bin/env python
"""
Zhiqing Lu

This node move robot to a desire pose in world frame. It use iiwa_ik_server to solve ik and use /iiwa/iiwa_fk_server to provide current pose.
SUBSCRIBERS:
	- /iiwa/joint_states (JointState)
PUBLISHERS:
	- /iiwa/PositionController/command (Float64MultiArray)
SERVICES:
	- /iiwa/iiwa_ik_server
	- /iiwa/iiwa_fk_server

"""

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from iiwa_tools.srv import GetIK, GetIKRequest, GetFK
from geometry_msgs.msg import Pose
import pandas as pd
import numpy as np


class Move:
	"""
	this class let robot move to a pose in world frame. 
	"""
	
	def __init__(self):
		rospy.loginfo("Robot starts moving")
		self.DT=1./1000.
		
		# setup state variables
		self.rob_state=JointState()
		
		# initial storing
		self.measurement = np.zeros(21)
		
		# setup publishers, subscribers, timers:
		self.move_pub=rospy.Publisher("/iiwa/PositionController/command", Float64MultiArray, queue_size=10)
		self.joint_sub=rospy.Subscriber("/iiwa/joint_states", JointState, self.update_state, queue_size=10)
		self.sim_timer = rospy.Timer(rospy.Duration(self.DT),self.moving)
		
		# setup goal
		self.goal = Float64MultiArray()
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		
		# setup IK client
		self.ik_service = '/iiwa/iiwa_ik_server'
		rospy.wait_for_service(self.ik_service)
		self.get_ik = rospy.ServiceProxy(self.ik_service, GetIK)
		self.ikseed=GetIKRequest()	# data type is GetIKRequest
		
		# setup FK client
		self.fk_service = '/iiwa/iiwa_fk_server'
		rospy.wait_for_service(self.fk_service)
		self.get_fk = rospy.ServiceProxy(self.fk_service, GetFK)
		self.seed = Float64MultiArray()
		self.seed.layout = MultiArrayLayout()
		self.seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
		self.seed.layout.dim[0].size = 1
		self.seed.layout.dim[1].size = 7
		self.seed.data = self.rob_state.position
		
		
	def moving(self,data):
		self.move_pub.publish(self.goal)
		
	def update_state(self,data):
		"""
		update data and storing position, velocity effort, end effector pos & orientation
		"""
		self.rob_state = data
		self.currentposition
		self.measurement = np.vstack((self.measurement, np.concatenate((self.rob_state.position,self.rob_state.velocity, self.rob_state.effort,self.sol_pose), axis=None)))
		
	def updategoal_pose(self, target):
		"""
		upadate the goal
		target: pose
		"""
		self.ikseed.seed_angles = self.ikseed.seed_angles
		self.ikseed.poses = [target]
		self.respIK = self.get_ik(self.ikseed)
		self.goal.data = self.respIK.joints.data
		
	def updategoal_joint(self, target):
		"""
		upadate the goal
		target: joint pos
		"""
		self.goal.data = target
	
	def currentposition(self):
		"""
		print current pose
		"""
		self.seed.data = self.rob_state.position
		self.respFK = self.get_fk(self.seed)
		self.sol_pose = self.respFK.poses[0]
		
		
	def back2zeros(self):
		"""
		move robot to zero position
		"""
		rospy.loginfo("Move to zero pose ")
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		
		
		
		
		

def main():
	"""
	Run the main loop, by instatiating a waver class, and then calling ros.spin
	"""
	rospy.init_node('Traj', log_level=rospy.INFO)

	try:
		test = Move()
		
		# back to zero position
		test.back2zeros()
		rospy.sleep(2.)
		
		# pose1
		rospy.loginfo("Move to pose 1")
		target1 = [0, 0, 0, 0.3, 0, 0, 0]
		test.updategoal_joint(target1)
		rospy.sleep(4.)
		
		# read current position in world frame and print
		print('current pose:')
		print(test.currentposition())
		
		# pose2
		rospy.loginfo("Move to pose 2")
		target2 = [0.5, 0.2, 0.5, 0.3, 0.5, -0.5, 0.5]
		test.updategoal_joint(target2)
		rospy.sleep(4.)
		
		# read current position in world frame and print
		print('current pose:')
		print(test.currentposition())
		
	
		
		"""
		# pose3
		rospy.loginfo("Move to pose 3")
		pose3 = Pose()
		pose3.position.x = -0.44
		pose3.position.y = 0.29
		pose3.position.z = 0.755
		pose3.orientation.x = 0.0
		pose3.orientation.y = 1.0
		pose3.orientation.z = 0.0
		pose3.orientation.w= 0.0
		test.updategoal(pose3) 
		rospy.sleep(4.)
		
		# read current position in world frame
		test.currentposition()
		
		# pose4
		rospy.loginfo("Move to pose 4")
		pose4 = Pose()
		pose4.position.x = 0.62
		pose4.position.y = -0.43
		pose4.position.z = 0.655
		pose4.orientation.x = 0.0
		pose4.orientation.y = 1.0
		pose4.orientation.z = 0.0
		pose4.orientation.w= 0.0
		test.updategoal(pose4) 
		rospy.sleep(4.)
		
		# read current position in world frame
		test.currentposition()
		"""
		# back to zero position
		test.back2zeros()
		rospy.sleep(4.)
		
		# read current position in world frame
		test.currentposition()
		
		# output data to excel
		df = pd.DataFrame(test.measurement)
		path = '/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/jointposition.csv'
		df.to_csv(path, header=False, index=False)
		   
		  
	except rospy.ROSInterruptException: 
		pass
	
	rospy.loginfo("Task complete")
	# rospy.spin()


if __name__=='__main__':
    main()
