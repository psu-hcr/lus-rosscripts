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
from LQR_Tinf import LQR_tinf
from KoopmanOperator import KoopmanOperator


class traj_follower:
	""" 
	This code follow a certain traj
	"""
	
	def __init__(self, _tarj, _control, _K):
		rospy.loginfo("Robot starts moving")
		self.DT=1/500							# change frequence to 500hz
		
		# setup state variables
		self.rob_state=JointState()
		
		# initialize desire joint state
		self.dstate = Float64MultiArray()
		self.dstate.data = [0, 0, 0, 0, 0, 0, 0]
		
		# initialize storing
		self.measurement = np.zeros(28)
		
		# initialize timer and record begin time
		self.t = 0
		self.begintime = rospy.get_rostime()
		
		# initialize traj 
		self.traj = _tarj
		
		# setup controller
		self.control = _control
		
		# setup Koopman Operator
		self.K = _K
		
		# initialize Torque
		self.Torque = Float64MultiArray()
		self.Torque.data = [0, 0, 0, 0, 0, 0, 0]
		
		# Torque saturation 
		self.sat = [10, 10, 3., 2., 2., 2., 1.]			# stiffness 500
		
		# setup shutdown
		rospy.on_shutdown(self.back2zeros)
		
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
		
	def moving(self,data):
		"""
		Publish Torque
		"""
		self.CaluTorque()
		self.move_pub.publish(self.Torque)
		print(self.Torque)
		rospy.sleep(0.1)
		
	def update_state(self,data):
		"""
		update data and storing position, velocity ,effort, end effector pos and orientation
		Note: I still use FK client to find end pose. But I didn't use them here. 
		"""
		self.rob_state = data
		self.currentpose()
		self.measurement = np.vstack((self.measurement, np.concatenate((self.rob_state.position,self.rob_state.velocity, self.rob_state.effort, self.sol_pose.position.x,self.sol_pose.position.y, self.sol_pose.position.z, self.sol_pose.orientation.x, self.sol_pose.orientation.y, self.sol_pose.orientation.z, self.sol_pose.orientation.w), axis=None)))
		
	def back2zeros(self):
		"""
		Move robot back to zeros pos and shut down
		"""
		rospy.loginfo("closing node")
		
		# move robot to zero pos
		
		# rospy.sleep(5.)
		
		
	def CaluTorque(self):
		"""
		This function convert desire traj to desire joint state and calculate the input
		"""
		
		# update time
		self.t = rospy.get_rostime() - self.begintime
		time = self.t.nsecs*1e-9 			# convert duration type to sec (float)
		
		# convert desire traj to desire joint state
		
		self.ikseed.poses = [self.traj(time)]
		self.respIK = self.get_ik(self.ikseed)
		self.dstate.data = self.respIK.joints.data
		
		# convert current state and dstate into measurement
		currentM = np.concatenate((self.rob_state.position,self.rob_state.velocity, self.rob_state.effort, self.sol_pose.position.x,self.sol_pose.position.y, self.sol_pose.position.z, self.sol_pose.orientation.x, self.sol_pose.orientation.y, self.sol_pose.orientation.z, self.sol_pose.orientation.w), axis=None)
		desireM = np.concatenate((self.dstate.data,self.rob_state.velocity, self.rob_state.effort, self.sol_pose.position.x,self.sol_pose.position.y, self.sol_pose.position.z, self.sol_pose.orientation.x, self.sol_pose.orientation.y, self.sol_pose.orientation.z, self.sol_pose.orientation.w), axis=None)
		
		# convert measurement into Koopman Basis function
		cbasis = self.K.basis_func(currentM)
		dbasis = self.K.basis_func(desireM)
		
		# clip Koopman Basis function into Koopman Basis state
		cbasis = cbasis[:self.K.basissize]
		dbasis = dbasis[:self.K.basissize]
		
		
		# calculate output
		output = self.control.output(cbasis, dbasis)
		
		"""
		# saturate input
		output[0] = np.clip(output[0], -self.sat[0], self.sat[0])
		output[1] = np.clip(output[1], -self.sat[1], self.sat[1])
		output[2] = np.clip(output[2], -self.sat[2], self.sat[2])
		output[3] = np.clip(output[3], -self.sat[3], self.sat[3])
		output[4] = np.clip(output[4], -self.sat[4], self.sat[4])
		output[5] = np.clip(output[5], -self.sat[5], self.sat[5])
		output[6] = np.clip(output[6], -self.sat[6], self.sat[6])
		"""
		
		self.Torque.data = output
		
	def currentpose(self):
		"""
		print current pose
		"""
		self.seed.data = self.rob_state.position
		self.respFK = self.get_fk(self.seed)
		self.sol_pose = self.respFK.poses[0]
		
	def currentposition(self):
		"""
		print out current and desire position
		"""
		print('current joint:')
		print(self.rob_state.position)
		print('desire joint:')
		print(self.dstate.data)
		
	
		
def dtraj(t):
	"""
	define the desire traj equation
	"""
	
	#x = 0.5*np.cos(2*t)
	#y = 0.5*np.sin(4*t) + 0.4
	x = 0
	y = 0
	z = 1
	
	
	# convert position to pose type
	pose1 = Pose()
	pose1.position.x = x
	pose1.position.y = y
	pose1.position.z = z
	pose1.orientation.x = 0.0
	pose1.orientation.y = 1.0
	pose1.orientation.z = 0.0
	pose1.orientation.w= 0.0
	
	
	return pose1


def main():
	# Take data from a datasheet and calculate a Koopman Operator
	# time step
	dt = 1./500.
	
	# convert data from datasheets into arrays
	df = pd.read_csv("/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/jointposition_T.csv")
	measuredata = pd.DataFrame.to_numpy(df)
	
	# Get total number of data
	datalength = (len(measuredata[:,0]))
	
	# initial Koopman Operator class
	Koop = KoopmanOperator(measuredata[0,:], dt)
	
	for i in range(1, datalength):
		# update data into Koopman Operator class
		Koop.update_database(measuredata[i,:])
	
	# calculate Koopman Operator
	Koop.compute_operator()
	
	# find A matrix and B matrix
	A = Koop.K_x
	B = Koop.K_u
	
	# define Q, R matrix
	Q = np.diag(np.concatenate((200*np.ones(3), 0.001*np.ones(Koop.basissize-3))))
	R = np.diag(0.001*np.ones(Koop.inputsize))
	
	# solve LQR in infinte time
	LQR = LQR_tinf(A,B,Q,R)
	
	
	#Run the main loop
	try:	
		rospy.init_node('Traj', log_level=rospy.INFO)
		test = traj_follower(dtraj, LQR, Koop)
		rospy.spin()
	
	except rospy.ROSInterruptException: 
		pass
		
	# output data to excel
	df = pd.DataFrame(test.measurement)
	path = '/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/traj_tracking.csv'
	df.to_csv(path, header=True, index=False)
	
	# print out gain
	print(test.control.gain[0])
		
	rospy.loginfo("task complete")

	
	
	
	
	
if __name__=='__main__':
    main()
