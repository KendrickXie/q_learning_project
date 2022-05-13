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
        self.q_matrix = np.loadtxt(path_prefix + "converged_q_matrix.csv", dtype=float, delimiter=",")

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # set current state
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

        
        
        # Alex
        # self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.target_color = None
        self.target_tag = None
        self.stop_threshold = 0.23
        self.image = None
        self.search_for_object = False
        self.object_found = False
        self.image = None





        # Kendrick
        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.search_for_tag = False
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.grayscale_img = None
        self.closest_range_in_front = 0
        self.closest_distance_allowed = 0.4
        self.ar_tag_found = False

        self.arm_up = [0,math.radians(-50),0,0]
        self.arm_down = [
            0.0,
            math.radians(5.0),
            math.radians(20.0),
            math.radians(-20.0)
        ]

        self.open_grip = [0.015,0.015]
        self.close_grip = [0.0001,0.0001]

        # set arm to upward position
        self.move_group_arm.go(self.arm_up, wait=True)

        # set gripper to open position
        self.move_group_gripper.go(self.open_grip, wait=True)



        rospy.sleep(5)

    def run(self):
        while True:
            #select_action
            self.select_action()
            rospy.sleep(1)
            self.search_for_object = True
            self.find_object()
            rospy.sleep(1)
            self.pick_up()
            # self.ang_complete = False
            # self.lin_complete = False
            rospy.sleep(1)
            self.turnaround()
            rospy.sleep(1)
            self.search_for_tag = True
            self.find_tag()
            rospy.sleep(1)
            self.put_down()
            rospy.sleep(1)
            self.turnaround()
            rospy.sleep(1)               



        # # when looking for tag set goal_id and set search_for_tag to True
        # self.goal_id = 2
        # self.search_for_tag = False
        # self.pick_up()
        # print("pick_up executed")
        # rospy.sleep(3)
        # self.put_down()
        # print("put_down executed")


        # # Keep the program alive.
        # rospy.spin()

    # find object and move to it
    def find_object(self): #Alex
        while self.search_for_object:
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
                print("pink mask")
                mask = cv2.inRange(hsv, lower_pink, upper_pink)
            elif self.current_action.robot_object == "green":
                print("green mask")
                mask = cv2.inRange(hsv, lower_green, upper_green)
            elif self.current_action.robot_object == "blue":
                print("blue mask")
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
            else:
                print("Error with current_action robot_object")

            h, w, c = self.image.shape   # height, width, channel

            # https://www.pythonpool.com/opencv-moments/
            # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
            # using moments() function, the center of the yellow pixels is determined
            M = cv2.moments(mask)
            # if there are any yellow pixels found
            if M['m00'] > 0 and not self.closest_range_in_front == 0:
                # center of the yellow pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)

                # turn robot towards tag and move forward
                kp_ang = 0.003
                ang_err = w/2 - cx
                
                self.twist.angular.z = kp_ang * ang_err
                if self.stop_threshold < self.closest_range_in_front:
                    print("moving forward")
                    self.twist.linear.x = 0.1
                else:
                    print("reached object")
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.search_for_object = False
                        
            else:
                # turn until a tag is found
                self.twist.angular.z = 0.5
                self.twist.linear.x = 0.0

            # Publish the Twist message
            self.velo_publisher.publish(self.twist)

    def stop(self):
        velo = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,0)
        )
        self.velo_publisher.publish(velo)


    
    # find tag and move to it
    def find_tag(self): #Kendrick
        # find the x coordinate of the center of the image
        while self.search_for_tag:
            h, w = self.grayscale_img.shape
            img_center_x = w / 2
            print("searching for tag")
            
            # set goal id
            goal_id = self.current_action.tag_id
            # extract tag parameters
            corners, ids, rejected_points = cv2.aruco.detectMarkers(self.grayscale_img, self.aruco_dict)
            curr_center_x = 0
            # check that a tag if found
            if len(corners) > 0:
                print("found tag")
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over detected tag corners
                for (markerCorner, markerID) in zip(corners, ids):
                    # skip if we are not looking for this tag
                    if not markerID == goal_id:
                        continue
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    # topRight = (int(topRight[0]), int(topRight[1]))
                    # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    # topLeft = (int(topLeft[0]), int(topLeft[1]))
                    # find the x coordinate of the center of the tag
                    width = int(bottomRight[0]) - int(bottomLeft[0])
                    curr_center_x = int(bottomLeft[0]) + (width / 2)

            if curr_center_x == 0 and not self.ar_tag_found:
                # turn until a tag is found
                self.twist.angular.z = 0.5
                self.twist.linear.x = 0.0
            elif curr_center_x == 0 and self.ar_tag_found:
                # stop once an tag was found and the tag is to close for the camera to detect
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                
                # if still far from the closest object in the front, keep moving forward
                if self.closest_distance_allowed < self.closest_range_in_front:
                    print("moving forward")
                    self.twist.linear.x = 0.1
                else:
                    # self.put_down
                    self.search_for_tag = False
                    self.ar_tag_found = False
            else:
                # turn robot towards tag and move forward
                k = 0.005
                e = img_center_x - curr_center_x
                self.twist.angular.z = k * e
                self.ar_tag_found = True
                if self.closest_distance_allowed < self.closest_range_in_front:
                    print("moving forward")
                    self.twist.linear.x = 0.1
                # else:
                #     print("too close")
                #     print("closest range:", self.closest_range_in_front)
                #     self.twist.linear.x = 0.0
                #     self.twist.angular.z = 0.0
                #     # self.put_down
                #     self.search_for_tag = False
                    

            # Publish the Twist message
            self.velo_publisher.publish(self.twist)



                
    def pick_up(self): #Alex
        # Move the arm down
        self.move_group_arm.go(self.arm_down, wait=True)
        rospy.sleep(3)
        # set gripper to close position
        self.move_group_gripper.go(self.close_grip, wait=True)
        rospy.sleep(1)
        # Move the arm up
        self.move_group_arm.go(self.arm_up, wait=True)
        rospy.sleep(3)

    
    def put_down(self): #Kendrick
        # Move the arm down
        self.move_group_arm.go(self.arm_down, wait=True)
        rospy.sleep(3)
        # set gripper to open position
        self.move_group_gripper.go(self.open_grip, wait=True)
        rospy.sleep(1)
        # Move the arm up
        self.move_group_arm.go(self.arm_up, wait=True)
        rospy.sleep(3)

    def select_action(self): #Alex
        # publish action
        print("Selecting action...")
        self.ang_complete = False
        self.lin_complete = False
        max_q_val_for_state = max(self.q_matrix[self.current_state])
        actions = [i for i, q_val in enumerate(self.q_matrix[self.current_state]) if q_val == max_q_val_for_state]
        selected_action_idx = random.choice(actions)
        selected_action = self.actions[selected_action_idx]
        self.perform_action(selected_action)
        # print("selected action:", selected_action)
        for i, action_taken in enumerate(self.action_matrix[self.current_state]):
            if action_taken == selected_action_idx:
                self.current_state = i
                break
    
    # perform an action by publishing to "/q_learning/robot_action"
    def perform_action(self, selected_action):
        print("performing action...", selected_action)
        color, tag = selected_action["object"], selected_action["tag"]
        message = RobotMoveObjectToTag(
            robot_object = color, 
            tag_id = int(tag)
        )
        self.action_publisher.publish(message)
        return

    # # get the color and tag associated with an action
    # def get_action_details(self, selected_action):
    #     action_deets = self.actions[selected_action["action_idx"]]
    #     color = action_deets["object"]
    #     tag = action_deets["tag"]
    #     return color, tag


    # callback function for when we publish an action
    def action_callback(self, msg):
        self.current_action = msg
        print("received instructions:", self.current_action)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # converts the incoming ROS message to OpenCV format and grayscale
        self.grayscale_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')

    def lidar_callback(self, msg):
        self.scan_ranges = msg.ranges
        # find range of the closest object within the front 90 degrees of the robot
        closest_range = 100
        angle = 0
        for angle_range in msg.ranges:
            if angle < 45 or angle > 315:
                if angle_range < closest_range and not angle_range == 0:
                    closest_range = angle_range
            angle = angle + 1
        self.closest_range_in_front = closest_range

    def get_smoothed_dist(self, smoothing_factor):
        smoothed_dist = 0
        for i in range(-smoothing_factor, smoothing_factor + 1):
            smoothed_dist += (self.scan_ranges[i])
        smoothed_dist /= (2*smoothing_factor + 1)
        return smoothed_dist
        # min_dist = 100
        # for i in range(-smoothing_factor, smoothing_factor + 1):
        #     if self.scan_ranges[i] < min_dist:
        #         min_dist = self.scan_ranges[i]
        # print("min_dist:", min_dist)
        # return min_dist

    def turnaround(self):
        print("turning around...")
        velo_b = Twist(
            linear = Vector3(-0.3,0,0),
            angular = Vector3(0,0,0)
        )
        self.velo_publisher.publish(velo_b)
        rospy.sleep(1.5)
        self.stop()
        velo_t = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,0.785398) #45 deg in rad
        )
        self.velo_publisher.publish(velo_t)
        rospy.sleep(4)


if __name__ == '__main__':
    # declare the ROS node and run it
    node = Perform()
    print("running")
    node.run()




'''
roscore
rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw
ssh turtlebot
inside ssh: bringup, 
ssh another window
inside ssh: bringup_cam
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
'''