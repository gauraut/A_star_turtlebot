# Project 3 Phase 2 Part 2

## Steps to run the package
1. Download the package and place it in your 'catkin_ws/src' folder.
2. Open terminal and type,
```
cd ~/catkin_ws
catkin_make clean && catkin_make
source devel/setup.bash
```
3. After the catkin workspace is build and set, we will run the code. The code takes five input arguments which are the starting 2D coordinates, starting orientation and the goal nodes. These values are already defaulted to the extreme coordinates so the launch file can run without this input.
4. The coordinates input is given from -5 to 5 range and the angle takes a range of 0 to 360. Below are the arguments:
- x_pos: Initial x-coordinate
- y_pos: Initial y-coordinate
- theta: Initial orientation
- gx: Goal's x-coordinate
- gy: Goal's y-coordinate
5. To run the code, type in the terminal,
```
roslaunch project_3 a_star.launch # x_pos:=-3.5 y_pos:=-3.5 gx:=4.5
```
You can uncomment and edit the input arguments as shown above.
