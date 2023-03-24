# Project : A* algorithm for path planning of Turtlebot in 2D environment and Gazebo

The project is to implement the A* algorithm for path planning of Turtlebot in 2D environment and Gazebo.
The project is a part of the course ENPM661 - Planning for Autonomous Robots at the University of Maryland, College Park.
The vizualization of the project is done using pygame.
The project is implemented in Python 3.8.10

The git repository is the ROS Package `project_3_phase2_661` and contains the 2D implementation of the project in the folder `/Astar_turtlebot_2d`

## Contributor

- Sanchit Kedia  UID:119159620
- Tanmay Haldankar UID:119175460

## Dependencies

The ROS dependencies required to run the project are:

1. [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. [Gazebo 11.12.0](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
3. [Ubuntu 20.04.2 LTS](http://releases.ubuntu.com/20.04/)
4. [Turtlebot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

The python packages required to run the project are:

1. numpy v1.23.5
2. pygame v2.2.0
3. vidmaker v2.3.0
4. heapq
5. time
6. argparse
7. sys
8. math

## Build Package

```sh
source /opt/ros/noetic/setup.bash
cd <Your ROS workspace src folder>
git clone https://github.com/Sanchitkedia/Astar_Turtlebot.git project_3_phase2_661 
cd ..
rosdep install -i --from-path src --rosdistro noetic -y #Check for missing dependencies
catkin_make
```

## Usage

### The project 2d implementation can be run using the following commands

```sh
cd <Your ROS workspace/src/project_3_phase2_661/Astar_turtlebot_2d/>
python3 a_star_sanchit_tanmay.py -h # Use this command to get help for the command line arguments
python3 a_star_sanchit_tanmay.py # Use this command to run the project with vizualization in pygame wihout saving the video
python3 a_star_sanchit_tanmay.py --save_video # Use this command to run the project with vizualization in pygame and save the video
```

### The project gazebo implementation can be run using the following commands

```sh
export TURTLEBOT3_MODEL=burger
source /opt/ros/noetic/setup.bash
cd <Your ROS workspace/src/project_3_phase2_661/>
source devel/setup.bash
roslaunch project_3_phase2_661 astar.launch
```

## Output

- Test case 1
  - Clearance: 2
  - START NODE: (19,19,0)
  - GOAL NODE: (70,200)
  - RPM1: 5
  - RPM2: 10

- Test case 2
  - Clearance: 2
  - START NODE: (50,125,0)
  - GOAL NODE: (440,125)
  - RPM1: 5
  - RPM2: 10
