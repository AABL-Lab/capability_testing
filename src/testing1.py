#!/usr/bin/env python3

from gen3_testing.gen3_movement_utils import Arm
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Pose
from kortex_driver.srv import *
from kortex_driver.msg import *
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import os
import sys
from capability_gathering import CapabilityGather, UserProfile
# print current python path
print(sys.path)
arm = Arm()




# # w1_pos = [0.16619999401711588, 0.7785654241466864, -2.656147389032405, -1.6220813525387703, -0.428920203052674, -0.8029643739476109, 0.9538544207795929]
# w1_pos = [0.3065738502051339, 1.4442917808003342, -2.9135245978414783, -1.4683647596838325, 0.3277827324943069, 1.4257284827380456, 1.2834905136555133]
# # w2_pos = [0.10105382233300322, 0.9539111461108116, -2.6693241774738996, -1.2647927259581762, -0.4682215365234086, -0.8400520879202729, 0.8754516240779328]
# pan_pos = [-0.6753696629608248, 0.7808406958239956, -2.7498163572087813, -1.4012104896363953, -0.2698799511786687, -0.6530007077008131, 0.8639198702417805]
# init_position = [-0.08981404249848701, 0.8337751503890102, 3.136384043129816, -2.097153072018081, 0.017837751032215257, 1.4101099750511126, 1.5872177775504885]

# capabilitygather = CapabilityGather(user_weight = 0, weight_history = [], weight_list = [1,5], initial_pose=init_position, pan_pose=pan_pos)
# capabilitygather.add_weight(1)

# arm.open_gripper()
# arm.goto_joint_pose(init_position)
# arm.goto_joint_pose(w1_pos)
# arm.close_gripper()
# # arm.goto_joint_pose(w2_pos)
# arm.goto_joint_pose(pan_pos)
# arm.open_gripper()