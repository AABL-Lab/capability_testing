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
# print current python path
print(sys.path)


class UserProfile:
    def __init__(self, comfort_history = [], comfort_level = 5):
        self.comfort_history = comfort_history
        self.comfort_level = comfort_level  # last comfort level

    def store_comfort(self):
        print("describe your comfort level(0-5):")
        curr_comfort = input()
        self.comfort_level = curr_comfort
        self.comfort_history.append(curr_comfort)
    
    def print_comfort_history(self):
        print(self.comfort_history)

class CapabilityGather:
    def __init__(self, user_weight = 0, weight_history = [], weight_list = [1,5], initial_pose=None, pan_pose=None):
        try:
            rospy.init_node('arm_movement')
        except:
            pass

        self.user_weight = user_weight
        self.weight_history = weight_history
        self.weight_list = weight_list
        # dictionary of weights and their poses
       
        w1_pos = [0.3065738502051339, 1.4442917808003342, -2.9135245978414783, -1.4683647596838325, 0.3277827324943069, 1.4257284827380456, 1.2834905136555133]
        self.weight_loc = {1: w1_pos}

        self.initial_pose = initial_pose
        self.pan_pose = pan_pose

        self.arm = Arm()
        self.user_profile = UserProfile()
    
    def get_weight(self, weight):
        # get weight pose from dictionary
        weight_loc = self.weight_loc[weight]
        # go to weight pose
        self.arm.goto_joint_pose(weight_loc)
        # close gripper
        self.arm.close_gripper()
        # return to initial pose
        self.arm.goto_joint_pose(self.initial_pose)

    
    def add_weight(self, weight):
        self.user_weight += weight
        self.weight_history.append(weight)
        self.get_weight(weight)
        # goto pan pose
        self.arm.goto_joint_pose(self.pan_pose)
        # open gripper
        self.arm.open_gripper()
        # return to initial pose
        self.arm.goto_joint_pose(self.initial_pose)

        self.user_profile.store_comfort()

# # define main function
# def main():
    



    







        
