#!/usr/bin/env python3

import rospy
import numpy as np
import os
import copy

from numpy import random
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveObjectToTag


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # once everything is setup initialized will be set to true
        self.initialized = False

        # Initialize this node
        rospy.init_node("q_learning")

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

        # Hyperparameters and Macros
        self.lr = 1.0
        self.dr = 0.8
        self.epochs = 5000
        self.converged = False
        self.iterations = 0
        self.num_states = len(self.states)
        self.num_actions = len(self.actions)
        self.curr_state = 0
        self.curr_reward = 0
        self.converge_threshold = 0.1
        self.num_of_continuous_below_thresh = 0


        # Q Matrix
        self.q_matrix = np.zeros((self.num_states, self.num_actions))

        # Publishers and Subscribers
        self.action_publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        self.reward_subscriber = rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)
        self.q_matrix_publisher = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)


        rospy.sleep(2)

        self.initialized = True




    def train(self):
        if not self.initialized:
            return 

        print("initialized")
        while not self.converged and self.iterations < self.epochs:
            # select list of valid actions for current state
            valid_actions = self.select_valid_actions()
            # no valid actions, so reset positions and increase iterations
            if len(valid_actions) == 0:
                self.reset_positions()
                self.iterations += 1
                continue
            # received candidate actions, choose one random candidate
            selected_action = random.choice(valid_actions, size=1)[0]  #next state index, action of to get there index
            # perform action
            self.perform_action(selected_action)

            # give time for the subscriber to receive the reward
            rospy.sleep(0.25)

            # get reward
            r_t = self.curr_reward

            # update Q value
            next_state = selected_action["next_state"]
            max_a_Q = max(self.q_matrix[next_state])
            curr_q = copy.deepcopy(self.q_matrix[self.curr_state][selected_action["action_idx"]])
            q_update = self.lr * (r_t + self.dr * max_a_Q - curr_q)
            self.q_matrix[self.curr_state][selected_action["action_idx"]] += q_update
            if self.check_converged(curr_q, selected_action):
                self.converged = True
                print("CONVERGED at iteration", self.iterations)
                self.save_q_matrix()
            self.iterations += 1
            self.curr_state = next_state
        if not self.converged:
            print("NOT CONVERGED")


    # reset positions to the state where all objects are at the origin
    def reset_positions(self):
        self.curr_state = 0
        return 


    # get a list of valid actions from current state's row in the action matrix
    def select_valid_actions(self):
        row_num = self.curr_state
        valid_actions = [] 
        for i, action in enumerate(self.action_matrix[row_num]):
            if action != -1:
                valid_actions.append({"next_state": i, "action_idx": int(action)})
        return valid_actions
        

    # check Q-matrix for convergence
    def check_converged(self, curr_q, selected_action):
        delta_q = abs(curr_q - self.q_matrix[self.curr_state][selected_action["action_idx"]])
        if delta_q < self.converge_threshold:
            self.num_of_continuous_below_thresh += 1
        else:
            self.num_of_continuous_below_thresh = 0
        if self.num_of_continuous_below_thresh >= 100:
            return True
        return False
        

    # perform an action by publishing to "/q_learning/robot_action"
    def perform_action(self, selected_action):
        color, tag = self.get_action_details(selected_action)
        message = RobotMoveObjectToTag(
            robot_object = color, 
            tag_id = tag
        )
        self.action_publisher.publish(message)
        return


    # get the color and tag associated with an action
    def get_action_details(self, selected_action):
        action_deets = self.actions[selected_action["action_idx"]]
        color = action_deets["object"]
        tag = action_deets["tag"]
        return color, tag


    # callback function to retreive the reward published by "/q_learning/reward"
    def get_reward(self, data):     # data: QLearningReward
        # pass
        self.curr_reward = data.reward
        return 

    
    # save the Q-matrix
    def save_q_matrix(self):
        print("saving q_matrix")
        np.savetxt(path_prefix + "converged_q_matrix.csv", self.q_matrix, delimiter = ",")
        return

if __name__ == "__main__":
    node = QLearning()
    node.train()
