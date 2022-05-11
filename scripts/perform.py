#!/usr/bin/env python3

import rospy
import numpy as np
import os
import copy

from numpy import random
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveObjectToTag
from sensor_msgs.msg import Image, LaserScan
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

        # current state 
        self.current_state = 0

        # current action from action subscriber -> RobotMoveObjectToTag object
        # string robot_object
        # int16 tag_id
        self.current_action = None

        # publishers and subscribers
        self.action_publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        self.action_subscriber = rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_callback)
        # subscribe to the robot's RGB camera data stream
        self.image_subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.velo_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


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
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.target_color = None
        self.target_tag = None
        self.ang_complete = False
        self.lin_complete = False
        self.stop_threshold = 0.25






        # Kendrick









        rospy.sleep(2)

    def run(self):
        #select_action
        # pass
        self.select_action()
        rospy.sleep(1)
        self.find_object()
        # if ang_complete == True and lin_complete == True
        # pick_up()
        # turn around 

    # find object and move to it
    def find_object(self): #Alex
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # 300, 40, 60
        lower_pink = np.array([150, 100, 70])
        # 340, 60, 80 
        upper_pink = np.array([170, 155, 210])
        # 60, 45, 45
        lower_green = np.array([30, 85, 80])
        # 85, 80, 95
        upper_green = np.array([45, 150, 200])
        # 180, 30, 50
        lower_blue = np.array([90, 85, 100])
        # 210, 75, 85
        upper_blue = np.array([105, 140, 240])
        
        mask = None
        if self.current_action.robot_object == "pink":
            mask = cv2.inRange(hsv, lower_pink, upper_pink)
        elif self.current_action.robot_object == "green":
            mask = cv2.inRange(hsv, lower_green, upper_green)
        elif self.current_action.robot_object == "blue":
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        else:
            print("Error with current_action robot_object")
            return
        
        h, w, c = self.image.shape   # height, width, channel

        # https://www.pythonpool.com/opencv-moments/
        # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        # if there are any yellow pixels found
        if M['m00'] > 0:
            # center of the yellow pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)

            kp_ang = 0.05
            ang_err = (cx - w/2)
            # adjusting robo ang
            while abs(ang_err) > 10:
                velo = Twist(
                    linear = Vector3(0,0,0),
                    angular = Vector3(0,0,kp_ang*ang_err)
                )
                self.velo_publisher.publish(velo)
            self.ang_complete = True
            # adjusting robo dist
            curr_dist = self.get_smoothed_dist(2)
            kp_lin = 0.25
            while curr_dist > self.stop_threshold:
                curr_dist = self.get_smoothed_dist(2)
                lin_err = self.stop_threshold - curr_dist
                velo = Twist(
                    linear = Vector3(kp_lin*lin_err,0,0),
                    angular = Vector3(0,0,0)
                )
                self.velo_publisher.publish(velo)
            self.lin_complete = True

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        cv2.imshow("window", self.image)
        cv2.waitKey(3)


    
    # find tag and move to it
    def find_tag(self): #Kendrick
        pass

    def pick_up(self): #Alex
        pass

    def put_down(self): #Kendrick
        pass

    def select_action(self): #Alex
        # publish action
        # pass
        max_q_val_for_state = max(self.q_matrix[self.current_state])
        actions = [i for i, q_val in enumerate(self.q_matrix[self.current_state]) if q_val == max_q_val_for_state]
        selected_action_idx = random.choice(actions)
        selected_action = self.actions[selected_action_idx]
        self.perform_action(selected_action)
        for i, action_taken in enumerate(self.action_matrix[self.current_state]):
            if action_taken == selected_action_idx:
                self.current_state = i
                break

    # perform an action by publishing to "/q_learning/robot_action"
    def perform_action(self, selected_action):
        color, tag = self.get_action_details(selected_action)
        message = RobotMoveObjectToTag(
            robot_object = color, 
            tag_id = tag
        )
        self.action_publisher.publish(message)
        return
    
    # callback function for when we publish an action
    def action_callback(self, msg):
        # pass
        self.current_action = msg
        print("received instructions:", self.current_action)
    
    def image_callback(self, msg):
        # pass
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def scan_callback(self, msg):
        # pass
        self.scan_ranges = msg

    def get_smoothed_dist(self, smoothing_factor):
        smoothed_dist = 0
        for i in range(-smoothing_factor, smoothing_factor + 1):
            smoothed_dist.append(self.scan_ranges[i])
        smoothed_dist /= (2*smoothing_factor + 1)
        return smoothed_dist

    def turnaround(self):
        velo = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,0.785398) #45 deg in rad
        )
        self.velo_publisher.publish(velo)
        rospy.sleep(4)
