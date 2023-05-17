# Multi_Robot_ws
Workspace for exploration with multiple robots. Occupancy Grid Mapping, Move base, Multi Map Merge and Exploration node in Stage world.

The key file for multirobot exploration is multi_explore.py on first_robot_navigation/scripts/multi_explore.py.

Used packages:
  - p3dx_description;
  - rc_simul_worlds;
  - robot_laser_grid_mapping; 

How to run:
  - source devel/setup.bash
  - roslaunch first_robot_navigation main.launch
