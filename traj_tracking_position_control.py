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
	
	def __init__(self, _tarj):
		rospy.loginfo("Robot starts moving")
		self.DT=1./1000.
		
		# setup state variables
		self.rob_state=JointState()
		
		# initial storing
		self.measurement = np.zeros(21)
		
		# initialize traj 
		self.traj = _tarj
		
		# initialize timer and record begin time
		self.t = 0
		self.begintime = rospy.get_rostime()
		
		# setup goal
		self.goal = Float64MultiArray()
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		
		# setup shutdown and a counter
		rospy.on_shutdown(self.back2zeros)
		self.counter = 0
		
		# setup IK client
		self.ik_service = '/iiwa/iiwa_ik_server'
		rospy.wait_for_service(self.ik_service)
		self.get_ik = rospy.ServiceProxy(self.ik_service, GetIK)
		self.ikseed=GetIKRequest()	# data type is GetIKRequest
		seed = Float64MultiArray()
		seed.layout = MultiArrayLayout()
		seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
		seed.layout.dim[0].size = 1
		seed.layout.dim[1].size = 7
		seed.data = [0, np.pi/4, 0, -np.pi/4, 0, 0, 0]
		print(seed)
		self.ikseed.seed_angles = seed
		
		
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
		self.move_pub=rospy.Publisher("/iiwa/PositionController/command", Float64MultiArray, queue_size=10)
		self.joint_sub=rospy.Subscriber("/iiwa/joint_states", JointState, self.update_state, queue_size=10)
		self.sim_timer = rospy.Timer(rospy.Duration(self.DT),self.moving)
		
	def moving(self,data):
	
		# use counter to check if the system should stop
		if self.counter == 0:
			self.updategoal_pose()
		self.move_pub.publish(self.goal)
		rospy.sleep(0.01)
		
	def update_state(self,data):
		"""
		update data and storing position, velocity effort, end effector pos & orientation
		"""
		self.rob_state = data
		
	def updategoal_pose(self):
		"""
		upadate the goal
		"""
		# update time
		self.t = rospy.get_rostime() - self.begintime
		time = self.t.nsecs*1e-9 			# convert duration type to sec (float)
		
		# update goal
		self.ikseed.poses = [self.traj(time)]
		#self.ikseed.seed_angles = 
		self.respIK = self.get_ik(self.ikseed)
		self.goal.data = self.respIK.joints.data
		
	
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
		self.goal.data = [0, np.pi/4, 0, -np.pi/4, 0, 0, 0]
		self.counter = 1
		rospy.sleep(5)
		
def dtraj(t):
	"""
	define the desire traj equation
	"""
	
	x = 0.5
	y = 0.3*np.cos(0.01*t)
	z = 0.3*np.sin(0.04*t) + 0.4
	
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
	"""
	Run the main loop, by instatiating a waver class, and then calling ros.spin
	"""
	rospy.init_node('Move', log_level=rospy.INFO)

	try:
		test = Move(dtraj)
		rospy.spin()   
		  
	except rospy.ROSInterruptException: 
		pass
	
	rospy.loginfo("Task complete")
	
	
	# output data to excel
	df = pd.DataFrame(test.measurement)
	path = '/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/trajPOScontrol.csv'
	df.to_csv(path, header=False, index=False)


if __name__=='__main__':
    main()
