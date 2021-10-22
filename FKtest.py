#!/usr/bin/env python

"""
Zhiqing Lu

This is example code for using fk_server
SUBSCRIBERS:
    
PUBLISHERS:
    
SERVICES:
	/iiwa/iiwa_fk_server
"""

from iiwa_tools.srv import GetFK
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

fk_service = '/iiwa/iiwa_fk_server'
rospy.wait_for_service(fk_service)
get_fk = rospy.ServiceProxy(fk_service, GetFK)
seed = Float64MultiArray()
seed.layout = MultiArrayLayout()
seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
seed.layout.dim[0].size = 1
seed.layout.dim[1].size = 7
seed.data = [0., 0., 0., 0., 0., 0., 0.]
resp = get_fk(joints=seed)
sol_pose = resp.poses[0]
print('sol:', sol_pose)



