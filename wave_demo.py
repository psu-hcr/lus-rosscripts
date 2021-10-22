#!/usr/bin/env python
"""
Zhiqing Lu

This node follows a desire trajectory.
SUBSCRIBERS:
	- /iiwa/joint_states (JointState)
PUBLISHERS:
	- /iiwa/PositionController/command (Float64MultiArray)
SERVICES:

"""

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState



class Waver:
	
	def __init__(self):
		rospy.loginfo("Creating Waver class")
		DT=1./100.
		# setup state variables
		self.rob_state=JointState()
		
		# setup publishers, subscribers, timers:
		self.wave_pub=rospy.Publisher("/iiwa/PositionController/command", Float64MultiArray, queue_size=10)
		self.joint_sub=rospy.Subscriber("/iiwa/joint_states", JointState, self.update_state, queue_size=10)
		self.sim_timer = rospy.Timer(rospy.Duration(DT),self.timercb)
		
		# setup a counter
		self.counter = 0
		
		# setup two joint angle
		self.msg1 = Float64MultiArray()
		self.msg2 = Float64MultiArray()
		self.msg1.data = [0, 0, 0, 1, 0, 0, 0]
		self.msg2.data = [0, 0, 0, -1, 0, 0, 0]

		
		
	def timercb(self,data):
		
		# waving
		if self.counter == 0:
			self.wave_pub.publish(self.msg1)
			if self.rob_state.position[3] > 1:
				self.counter = 1
		else:
			self.wave_pub.publish(self.msg2)
			if self.rob_state.position[3] < -1:
				self.counter = 0
	
		
	def update_state(self,data):
		self.rob_state = data
		
		

def main():
    """
    Run the main loop, by instatiating a waver class, and then calling ros.spin
    """
    rospy.init_node('wave', log_level=rospy.INFO)

    try:
        test = Waver()
    except rospy.ROSInterruptException: 
    	pass

    rospy.spin()


if __name__=='__main__':
    main()
