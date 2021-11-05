#!/usr/bin/env python
"""
Zhiqing Lu

This node use PD contoller to move robot back to zero position
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
from math import pi


class Move_Torque:
	"""
	this class move robot to a certain joint state with torque input use PD controller
	
	
	"""
	
	def __init__(self, _target):
		rospy.loginfo("Robot starts moving")
		self.DT=1/500							# change frequence to 500hz
		
		# setup state variables
		self.rob_state=JointState()
		
		# initial storing
		#self.measurement = np.zeros(28)
		self.measurement = np.zeros(7)
		
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
		
		# setup publishers, subscribers, timers:
		self.move_pub=rospy.Publisher("/iiwa/TorqueController/command", Float64MultiArray, queue_size=10)
		self.joint_sub=rospy.Subscriber("/iiwa/joint_states", JointState, self.update_state, queue_size=10)
		rospy.wait_for_message("/iiwa/joint_states", JointState)
		self.sim_timer = rospy.Timer(rospy.Duration(self.DT),self.moving)
		
		# setup goal joint state
		self.goal = Float64MultiArray()
		self.goal.data = _target
		
		# setup Torque
		self.Torque = Float64MultiArray()
		self.Torque.data = [0, 0, 0, 0, 0, 0, 0]
		
		# setup constant applied Torque
		self.constantT = [20, 90, 30, 30, 5, 4, 0.5]
		
		# setup shutdown
		rospy.on_shutdown(self.back2zeros)
		
	def moving(self,data):
		"""
		Publish Torque
		"""
		self.CaluTorque()
		self.move_pub.publish(self.Torque)
		
	def update_state(self,data):
		"""
		update data and storing position, velocity ,effort, end effector pos and orientation
		"""
		self.rob_state = data
		self.currentpose()
		#self.measurement = np.vstack((self.measurement, np.concatenate((self.rob_state.position,self.rob_state.velocity,self.rob_state.effort, self.sol_pose.position.x,self.sol_pose.position.y, self.sol_pose.position.z, self.sol_pose.orientation.x, self.sol_pose.orientation.y, self.sol_pose.orientation.z, self.sol_pose.orientation.w), axis=None)))
		#self.measurement = np.vstack((self.measurement, self.Torque.data))
		
	def back2zeros(self):
		"""
		Move robot back to zeros pos and shut down
		"""
		rospy.loginfo("closing node")
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		rospy.sleep(5.)
		
		
	def CaluTorque(self):
		"""
		calculate desire Torque
		"""
		
		# calculate error of pos
		self.error0 = self.goal.data[0] - self.rob_state.position[0]
		self.error1 = self.goal.data[1] - self.rob_state.position[1]
		self.error2 = self.goal.data[2] - self.rob_state.position[2]
		self.error3 = self.goal.data[3] - self.rob_state.position[3]
		self.error4 = self.goal.data[4] - self.rob_state.position[4]
		self.error5 = self.goal.data[5] - self.rob_state.position[5]
		self.error6 = self.goal.data[6] - self.rob_state.position[6]
		
		# calculate force
		Force = [0, 0, 0, 0, 0, 0, 0]
		
		Force[0] = self.sign(self.error0) * self.constantT[0]
		Force[1] = self.sign(self.error1) * self.constantT[1]
		Force[2] = self.sign(self.error2) * self.constantT[2]
		Force[3] = self.sign(self.error3) * self.constantT[3]
		Force[4] = self.sign(self.error4) * self.constantT[4]
		Force[5] = self.sign(self.error5) * self.constantT[5]
		Force[6] = self.sign(self.error6) * self.constantT[6]
		
		self.Torque.data = Force
		
	def update_target(self, _newjointstate):
		"""
		update target
		"""
		self.goal.data = _newjointstate
		
	def currentpose(self):
		"""
		print current pose
		"""
		self.seed.data = self.rob_state.position
		self.respFK = self.get_fk(self.seed)
		self.sol_pose = self.respFK.poses[0]
		
		
	def currentposition(self):
		"""
		print out current position
		"""
		
		print('current joint:')
		print(self.rob_state.position)
		
	def sign(self, value):
		if value > 0.5:
			return 1
		elif value < -0.5:
			return -1
		else:
			return 0
	

def main():
	"""
	Run the main loop, by instatiating a waver class, and then calling ros.spin
	"""
	rospy.init_node('Torq', log_level=rospy.INFO)
	
	
	try:
		jointstate0 = [0, 0, 0, 0, 0, 0, 0]
		test = Move_Torque(jointstate0)
		rospy.sleep(2.)
		
		rospy.loginfo("pose1")
		jointstate1 = [pi/4, pi/4, pi/4, pi/2, pi/4, pi/4, pi/4]
		test.update_target(jointstate1)
		rospy.sleep(5.)
		
		rospy.loginfo("pose1")
		jointstate2 = [0.5, 0.2, 0.5, 0.3, 0.5, -0.5, 0.5]
		test.update_target(jointstate2)
		rospy.sleep(5.)
			
		
		
		"""
		# output data to excel
		df = pd.DataFrame(test.measurement)
		path = '/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/lus-rosscripts/MOVE_T.csv'
		df.to_csv(path, header=False, index=False)
		"""
		  
	except rospy.ROSInterruptException: 
		pass
	
	rospy.loginfo("task complete")

	# rospy.spin()


if __name__=='__main__':
    main()
