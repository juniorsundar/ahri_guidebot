# ahri_guidebot
This nodes in this project were written for a simulated environment. Therefore, you will need to install the necessary files to run the Sawyer Gazebo simulator in your workspace. 
Clone the repository into your workspace.
```console
git clone https://github.com/juniorsundar/ahri_guidebot.git
```
Use the following instructions to run the project

## Instruction
First start the Sawyer Gazebo simulation
```console
roslaunch sawyer_gazebo sawyer_world.launch
```
Next start the MotionPlanner rosnode
```console
rosrun ahri_guidebot MotionPlanner.py
```
Wait for the console to indicate the RRT is complete and the `Node is now accepting coordinates`.
Depending on the type of guidance you want to provide (On-demand or Continuous), start the corresponding WayPoints nodes.
for On-Demand:
```console
rosrun ahri_guidebot WayPoints.py
```
for Continuous:
```console
rosrun ahri_guidebot WayPointsCont.py
```
Finally, to initiate the guidance, publish to the `camera_input` rostopic, the position of the hand.
For On-Demand guidance, a trajectory will only be provided iff the hand position is within obstacle boundaries.
For Continuous guidance, regardless of the hand position, a trajectory will be provided.

You can change the obstacle position by modifying the hardcoded information in the Python files.
You can change the shoulder position, human hand end position, human hand start position etc. by changing the hardcoded information in the Python files.

## Simulation using PyBullet

Clone https://github.com/cairo-robotics/cairo_simulator repo and follow instructions in the ReadMe.

copy `src/collision_detection.py` from this repo into `cairo_simulator/src/` directory

run the code with this command 

```
DISPLAY=:0 python collision_detection.py
```