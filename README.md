uav_landing
=========

Software packages for automatic landing of an unmanned aerial vehicle, along with the software necessary for the successful commissioning of the algorithm.

## Contents of individual packages and files
#### Software package uav_landing
   * action/Landing.action: An action used by the algorithm to set an action goal, notify about the success of the action goal, and send feedback.

         # Goal - Desired final height in meters
         float64 target_height
         ---
         # Result - Landing state: 1 - landed successfully, 2 - landing canceled
         int8 status_code
         ---
         # Feedback - Current drone height in meters
         float64 current_height

   * calibrationdata_pinhole/ost.yaml: Calibrated camera parameters
   * start/start.sh: With the command `./start.sh` in the terminal can be used to start tmux
   * start/uav_landing.yml: Lists the commands that will be opened within tmux

#### Software package uav_landing_cpp
   * src/landing_action_server.cpp: Action server with the implementation of regulation and setting the speed,, ramp during startup, critical states of the algorithm etc.
   * src/landing_action_client.cpp: Action client with implementation of optical marker detection, server connection or disconnection detection, sending action goal, receiving feedback, etc.
   
   More information is provided in the comments of the codes
#### Software package aruco_ros
   * Aruco marker detection package downloaded from https://github.com/pal-robotics/aruco_ros
   * Detection is performed using the detectors `single.launch.py`, `single_1.launch.py` ​​and `single_2.launch.py`, in which it is possible to set the topic from which the camera parameters are subscribed, the output image from the camera, the size of the marker and its ID

#### Config file pylon_camera_wrapper_config.zip
   * Configuration file for the software package from https://github.com/basler/pylon-ros-camera/tree/humble, specifically for `pylon_ros2_camera_wrapper`, which allows setting fps and a relative path to the file with calibrated camera parameters

## How to run
Starting detectors, individual echoes for detector poses, and the algorithm consisting of an action server and an action client by moving to file `uav_landing/start` and the following command `./start.sh`

Stopping the algorithm and all other sessions is possible with `tmux kill-server` and stopping only a specific session with `tmux kill-session`

For the packages to work properly, it is necessary to use `colcon build` for the workspace and `source` packages in file `~/.bashrc` or in every terminal. In case of problems with the `colcon build` command of the basler package, it is advisable to study document `Interfacing Basler Cameras with ROS 2`
