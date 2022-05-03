# q_learning_project
## Team members
Kendrick Xie, Alex Wuqi Zhang
## Implementation plan
### Q-learning algorithm
#### Executing the Q-learning algorithm
Implement the algorithm that we covered in class today i.e. create Q-matrix, assign positive weight to successful block placement and negative weights to incorrect block placements. Test it by verifying that the values in our Q matrix are progressing in the correct general direction as we iterate.
#### Determining when the Q-matrix has converged
Once the Q-value stops changing after a certain number of trajectories we can determine that the Q-matrix has converged. We can experiment with the number of trajectories with a constant Q-value that need to occur before convergence. We can test this by observing the number of times the Q-value is the same before the Q-matrix has converged.
#### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
We will argmax it and go with the action with the maximum reward based on our Q matrix (greedy algo). We will visually inspect the robot’s actions and given the lack of ambiguity in the optimal actions, it should be easy to determine.
### Robot perception
#### Determining the identities and locations of the three colored objects
We’ll figure out the location and color of the objects by leveraging the camera and the relevant OpenCV libraries (similar to the line follower lab we did earlier). More specifically, we’ll scan for the HSV pixel values for a particular color in each frame. If the said color is not in the frame, we’ll just have the robot pivot in place until it is. We’re assuming that the environment is constrained enough such that the colored objects and the AR tags are simultaneously within the robot’s circumference of sight.
#### Determining the identities and locations of the three AR tags
Utilize the robot’s camera and the aruco module in OpenCV and follow internet resources such as the following:
-   [https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/](https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/)

To test this, we will make sure to use a real TurtleBot, because differences in camera angle and lighting may affect whether the ArUco marker is detected.

### Robot manipulation & movement
#### Picking up and putting down the colored objects with the OpenMANIPULATOR arm
We’ll identify the location of the target object following the procedure described in “Determining the identities and locations of the three colored objects”. We’ll leverage the open_manipulator_controller package created by Robotis to control the joint movements. ([https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#launch-controller](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#launch-controller)). Test by visually evaluating the arm performance and optimize by trial and error.
#### Navigating to the appropriate locations to pick up and put down the colored objects
We can adjust the robot’s orientation, until either the AR tags or the colored objects are within a range of the center pixel of the camera. Based on whether the arm is holding an object, we can determine whether the robot should be looking for the AR tags or the colored objects. Visually inspecting that the robot is placing the objects down at the correct AR tag.
### Timeline:
We plan on meeting up over weekends to test, while spending weekdays implementing functions.

4/27: Divide up functions

4/29: Meet up to test

5/3: Intermediate deliverable due

5/10: Be ready for final polish

5/12: Everything due

## Writeup
### Objectives Description:
The goal of this project was to use Q-learning to give a robot the ability to place objects in the correct position (positions that yield the highest reward). The project also involves implementing detection of objects to pick up and AR tags to place objects in front of. The last part of the project involves manipulating a robot’s arm to pick up the objects, navigating to the location where they are to be dropped off, and dropping the object from the robot’s arm.
### High-Level Description:
Given a current state, the robot will select a random possible next action. After the robot has performed the selected action and received its reward, the current (state+action)’s Q value will be updated to account for its current reward and max possible reward of the next state. For our specific task of determining where colored objects belong, this means for all 64 possible states, there will be rewards for the nine possible actions that can be performed.
### Q-learning Algorithm Description:
#### Selecting and executing actions for the robot (or phantom robot) to take
We selected a series of valid actions by referencing the action matrix row corresponding to the current state (initialized to 0). We picked out the ones that were not -1 and saved the resulting state and action index information in a dictionary obj. From this list of valid actions, we selected one at random using np.random.choice. I do want to note that when no valid states are available at the current state, we will reset the current state to 0. 
#### Updating the Q-matrix
Retrieved the reward after sleeping for 0.25 after performing the action. We then implemented the algorithm provided in the project spec - using a LR of 1.0, DR of 0.8, selecting the max Q value for the next state and a negative current state Q value to create the update factor that was then added to the current state Q value. 
#### Determining when to stop iterating: 
We stop iterating once we observe 100 consecutive iterations in which the Q value updates are less than a threshold value of 0.1 (which could technically be 0 in this case) but we’re using a threshold value for the sake of generalizability (in future work). We erred on the safe side by waiting to declare a convergence after 100 consecutive iterations. 
