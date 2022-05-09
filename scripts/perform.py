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
from sensor_msgs.msg import LaserScan

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
        # TODO: read in csv with proper types
        # self.q_matrix = np.loadtxt(path_prefix + "converged_q_matrix.csv", dtype=int)

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

        # # the interface to the group of joints making up the turtlebot3
        # # openmanipulator arm
        # self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # # the interface to the group of joints making up the turtlebot3
        # # openmanipulator gripper
        # self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        # self.move_group_arm.go([0,0,0,0], wait=True)

        # Alex








        # Kendrick
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.search_for_tag = False
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.img_center_x = 0
        self.grayscale_img = None
        self.closest_range_in_front = 0
        self.closest_distance_allowed = 0.6
        self.goal_id = 0
        self.ar_tag_found = False






        rospy.sleep(2)

    def run(self):
        #select_action

        # when looking for tag set goal_id and set search_for_tag to True
        self.goal_id = 2
        self.search_for_tag = True

        # Keep the program alive.
        # node.stop()
        rospy.spin()

    # find object and move to it
    def find_object(self): #Alex
        pass
    
    # find tag and move to it
    def find_tag(self): #Kendrick
        corners, ids, rejected_points = cv2.aruco.detectMarkers(self.grayscale_img, self.aruco_dict)
        curr_center_x = 0
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            print("found tag")
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                if not markerID == self.goal_id:
                    continue
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                width = int(bottomRight[0]) - int(bottomLeft[0])
                curr_center_x = int(bottomLeft[0]) + (width / 2)
        print(curr_center_x)
        if curr_center_x == 0 and not self.ar_tag_found:
            #turn more until another tag
            # TODO: uncomment this later
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0.0
            print("no tag found")
            print("closest range:", self.closest_range_in_front)
        elif curr_center_x == 0 and self.ar_tag_found:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            # self.put_down
            if self.closest_distance_allowed < self.closest_range_in_front:
                print("moving forward")
                self.twist.linear.x = 0.1
            else:
                self.search_for_tag = False
                self.ar_tag_found = False
        else:
            k = 0.01
            e = self.img_center_x - curr_center_x
            self.twist.angular.z = k * e
            self.ar_tag_found = True
            if self.closest_distance_allowed < self.closest_range_in_front:
                print("moving forward")
                self.twist.linear.x = 0.1
            else:
                print("too close")
                print("closest range:", self.closest_range_in_front)
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                # self.put_down
                self.search_for_tag = False
                

        # Publish the Twist message
        self.twist_publisher.publish(self.twist)



                
    def pick_up(self): #Alex
        pass
    
    def put_down(self): #Kendrick
        pass

    def select_action(self): #Alex
        # publish action
        pass
    
    # callback function for when we publish an action
    def action_callback(self, msg):
        pass

    def image_callback(self, msg):
        # print("doing image callback")
        if self.search_for_tag:
            # converts the incoming ROS message to OpenCV format and grayscale
            self.grayscale_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')

            # find the x coordinate of the center of the image
            h, w = self.grayscale_img.shape
            self.img_center_x = w / 2
            print("searching for tag")
            self.find_tag()

    def lidar_callback(self, msg):
        # print("doing lidar callback")
        closest_range = 100
        angle = 0
        for angle_range in msg.ranges:
            if angle < 45 or angle > 315:
                if angle_range < closest_range and not angle_range == 0:
                    closest_range = angle_range
            angle = angle + 1
        self.closest_range_in_front = closest_range

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist)

if __name__ == '__main__':
    # declare the ROS node and run it
    node = Perform()
    print("running")
    node.run()