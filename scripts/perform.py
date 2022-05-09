#!/usr/bin/env python3

import rospy
import numpy as np
import os
import copy

from numpy import random
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveObjectToTag
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
import cv2, cv_bridge
import moveit_commander
import math

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class Perform(object):
    def __init__(self):
        # once everything is setup initialized will be set to true
        self.initialized = False

        # Initialize this node
        rospy.init_node("perform")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-8 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        self.colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": self.colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        
        # read in our trained q_matrix
        self.q_matrix = np.loadtxt(path_prefix + "converged_q_matrix.csv", dtype=int)

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()


        self.current_state = 0

        

        # publishers and subscribers
        self.action_publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        self.action_subscriber = rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_callback)
        # subscribe to the robot's RGB camera data stream
        self.image_subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # create a default twist msg with values of all zeros
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)

        # Alex








        # Kendrick









        rospy.sleep(2)

    def run(self):
        #select_action

    # find object and move to it
    def find_object(self): #Alex
    
    # find tag and move to it
    def find_tag(self): #Kendrick

    def pick_up(self): #Alex

    def put_down(self): #Kendrick

    def select_action(self): #Alex
        # publish action
        pass
    
    # callback function for when we publish an action
    def action_callback(self, msg):
        
    def image_callback(self, msg):
        pass