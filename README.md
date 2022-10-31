# FEUP - Intelligent Robotics

## Group Information

- Lisa Sonck (up202202272@up.pt)
- João Pires (up201806079@up.pt)
- Sergio Murino (up201806554@up.pt)

## Directory Structure

- robot_gazebo
    - launch
        - robot.launch (Launch file for this project)
    - models/model_wall
        - model.config (Model Configuration file)
        - model.sdf (Model Definition file)
    - scripts
        - move.py (Python script with the robot code - movement and stopping criteria)
    - worlds
        - robot.world (World Definition file)
    - CMakeLists.txt and package.xml (Compiling and configuration files)

## Requirements

1. Have ROS and Gazebo installed.

2. Setup a Catkin workspace.

3. Clone and build the necessary GitHub repositories.

**Note:** To achive the above steps, please consult the following link [ROS Installing](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros). Perform all the steps until *Testing Gazebo with ROS integration*. 

## How to compile

1. Copy the folder ```robot_gazebo``` to the folder ```catkin_ws/src``` and run the following command in the folder ```catkin_ws```

```
catkin_make
```

## How to run


1. Open a terminal and run the following 3 commands sequentially:

```
source /opt/ros/noetic/setup.bash
```

```
source ~/catkin_ws/devel/setup.bash
```

```
roscore &
```

2. Then, in a new terminal, run:

```
roslaunch robot_gazebo robot.launch
```

3. Inside a new terminal, go to the folder ```catkin_ws/src/robot_gazebo``` and run the following 2 commands to run the robot: 

```
chmod +x scripts/move.py
```

```
python scripts/move.py
```


