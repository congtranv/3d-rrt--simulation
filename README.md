# 3d-rrt--simulation

## Required
- ROS Melodic
- `mkdir ~/3d-rrt`
- `cd ~/3d-rrt`
- `catkin_init_workspace`
- `mkdir ~/3d-rrt/path_planning`
- `cd ~/3d-rrt/path_planning`
- `catkin_create_pkg path_planning roscpp std_msgs visualization_msgs`

## Source
- `git clone https://github.com/congtranv/3d-rrt--simulation.git` *(separate directory with ~/3d-rrt/path_planning/path_planning)*
- `cd 3d-rrt--simulation`
- copy `environment.cpp`, `rrt.cpp` and `rrt_node.cpp` to `~/3d-rrt/path_planning/path_planning/src`
- copy `obstacles.h`, and `rrt.h` to `~/3d-rrt/path_planning/path_planning/include/path_planning`
- copy `CMakeLists.txt` and replace to `~/3d-rrt/path_planning/path_planning/CMakeLists.txt`

## Build
- `cd ~/3d-rrt/path_planning/path_planning`
- `cmake make CMakeLists.txt`
- `sudo make install`
- `source devel/setup.bash`

## Usage
*terminal 1*
- `source devel/setup.bash`
- `roscore`

*terminal 2*
- `source devel/setup.bash`
- `rosrun rviz rviz`  
- *in rviz window* enter "path_planner" to Fixed Frame; select Add --> Marker, douple click to expand Marker, then enter 'path_planner_rrt' to Marker Topic

*terminal 3*
- `source devel/setup.bash`
- `rosrun path_planning env_node`

*terminal 4*
- `source devel/setup.bash`
- `rosrun path_planning rrt_node`
