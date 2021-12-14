# Pick-Pack: UR5 Kitchen Robot
* From burger flipping to cooking robots, many food industries are beginning to discover the benefits of automating their tedious, repetitive tasks with robots.
* The major challenge of any restaurant industry is finding, training, and retaining staff due to an acute shortage in the labor force. Leveraging these robots to automate repetitive tasks with utmost precision which can work 24/7 with minimum errors will reap profits for the restaurant owners. We can use these robots for packaging, cooking, integrating with the delivery robots and other processes, making fewer mistakes, reducing food wastage, and working all day long without a pay rise.
* We propose Pick Pack which is a 6 axis UR5 robot which can assist restaurants by packaging their food
products or helping out in the cooking process.

## Installation

Clone this repository into your workspace:

```bash
git clone https://github.com/sumedhreddy90/KitchenRobot
catkin_make
source devel/setup.bash
```

## Usage
### Inverse Kinematics Validation 
we have implement a C++ program that allows us to move the tcp in Cartesian space. We have created a ROS node that uses KDL for our kinematic calculations and that interfaces with our simulated robot in Gazebo. 
- Launch UR5 onto empty gazebo world 
```python
roslaunch kitchen_bot iksolver.launch
```
- Run the script to plan and execute the custom trajectory of the tool center point/ gripper position 
```python
rosrun kitchen_bot IKSolver
```
### Pick and Place Food objects
- Launch UR5 Robot onto kitchen gazebo world 
```python
roslaunch kitchen_planning demo_gazebo.launch
```
- Run the below script for planning the robot to pick food objects and place them in a plate
```python
rosrun kitchen_bot KitchenPlan
```
### Plotting UR5 joint angles with RQT

```python
roslaunch kitchen_bot jointRQT.launch
```
## Simulation Videos

Presentation: https://youtu.be/dOgkYswC5M4
Simulation Demo: https://youtu.be/OZsbqnFeqTQ
IK Validation: https://youtu.be/VXxi4hUbNsw
Workspace Analysis: https://youtu.be/Q8qiyR_ZVNs
RViz Forward Kinematics: https://youtu.be/3U3yQqhbA_E

## Contributors

- Rahul Karanam - 118172507
- Sumedh Reddy Koppula - 117386066

## License
[MIT](https://choosealicense.com/licenses/mit/)
