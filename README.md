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
   * src/landing_action_client.cpp: Action client with implementation of optical marker detection, server connection or disconnection detection, sending action goals, receiving feedback, etc.
   
   More information is provided in the comments of the codes
#### Software package aruco_ros
   * 
   * 
   * 

#### pylon_camera_wrapper_config.zip
   * 
   * 
   * 

Starting detectors, individual echoes for detector poses, and the algorithm consisting of an action server and an action client by moving to file `uav_landing/start` and the following command `./start.sh`

Stopping the algorithm and all other sessions is possible with `tmux kill-server` and stopping only a specific session with `tmux kill-session`
