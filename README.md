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
