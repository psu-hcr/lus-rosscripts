#!/usr/bin/env python

"""
Zhiqing Lu

This is example code for using ik_server
SUBSCRIBERS:
    
PUBLISHERS:
    
SERVICES:
	/iiwa/iiwa_ik_server
"""

from iiwa_tools.srv import GetIK, GetIKRequest
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension, Float32, Int32, Float64
from geometry_msgs.msg import Pose


ik_service = '/iiwa/iiwa_ik_server'
rospy.wait_for_service(ik_service)
get_ik = rospy.ServiceProxy(ik_service, GetIK)
ikseed=GetIKRequest()
pose1 = Pose()
pose1.position.x = 0.0
pose1.position.y = 0.0
pose1.position.z = 1.0
pose1.orientation.x = 0.0
pose1.orientation.y = 1.0
pose1.orientation.z = 0.0
pose1.orientation.w= 0.0
ikseed.poses=[pose1]

seed = Float64MultiArray()
seed.layout = MultiArrayLayout()
seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
seed.layout.dim[0].size = 1
seed.layout.dim[1].size = 7
seed.data = [0., 0., 0., 0., 0., 0., 0.]
ikseed.seed_angles = seed

resp = get_ik(ikseed)
sol_joint = resp.joints
print('sol:', sol_joint)



