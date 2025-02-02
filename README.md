Unitree GO1 Simulation with ROS 2

This repository contains a simulation environment for the Unitree GO1 robot in Gazebo 11 and ROS 2, along with an interface with the ROS 2 navigation stack(work in progress). The functionality has been tested with ROS Humble on Ubuntu 22.04.
Dependencies:

    lcm (Needs to be built from source, instructions here)
    navigation2
    ros2_control
    ros2_controllers
    gazebo_plugins

Structure

The repository contains the following ROS 2 packages:

    go1_sim:
        go1_description: Contains required xacro and config files to load the robot in simulation. Modified from here.
        go1_gazebo: Contains the gazebo world and required launch files to initialize the simulation.
        go1_navigation: Contains the scripts, launch and configuration files for using the navigation stack with the robot.

    ros2_unitree_legged_controller: Contains the implementation of unitree's control plugin in ROS 2. This has been ported from the ROS 1 plugin available here.

    ros2_unitree_legged_msgs: Contains unitree custom messages for interfacing, source here.

    unitree_guide2: Provides an interface between the navigation and control framework along with a state machine for different control modes. This has been ported to ROS 2 from here.

Testing

For nav stack functionality (work in progress)

    Replace the paths on lines 6 and 7 in this file with your workspace paths.

After placing all the packages in a ROS 2 workspace and building it successfully, to verify the functionality, execute the following in order (in separate terminal panes):

    (window 1) ros2 launch go1_gazebo spawn_go1.launch.py: This will load the simulation and initialize controllers.

    (window 2) ros2 run unitree_guide2 junior_ctrl: This activates the interface and state machine. Run this once the last controller plugin (RL_calf_controller) has loaded successfully

    (window 3) ros2 launch go1_navigation navigation.launch.py: This will launch the navigation stack. (work in progress)

    Once the costmaps are visible in rviz, switch the mode in window 2 by pressing '2'. The robot will stand up. Switch to nav mode by pressing '5'.

    The robot will now navigate as per commands received on the interface topic /cmd_vel.

Assumptions and Remarks

    For the sake of demonstration, a known map is used and SLAM does not run. A ground truth plugin is used for odometry, and the map to odom frame transform is assumed to be static and zero.
    The navigation stack (work in progress) parameters are not tuned to a high degree. Hence for some goals there may be unexpected performance which can be improved upon further tuning. The goal tolerance parameters are set to xy: 0.2 and yaw: 0.3

