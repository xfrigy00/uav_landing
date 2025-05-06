import launch
from launch.actions import ExecuteProcess
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Open tmux in a new terminal and split the window into 4 panes
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--',  # Open a new GNOME terminal
                'bash', '-c',  # Use bash to execute the tmux commands
                # Start tmux session and run ROS2 nodes in different panes
                'tmux new-session -d -s my_session2_1; '
                'tmux split-window -h; '  # Split the window horizontally
                'tmux split-window -v; '  # Split one of the panes vertically
                'tmux select-pane -t 0; '  # Select pane 0 (image_subscriber)
                'tmux split-window -v; '  # Split the remaining pane vertically
                'tmux select-pane -t 0; '  # Select pane 0 (image_subscriber)
                'tmux send-keys -t my_session2: "ros2 launch aruco_ros single_1_1.launch.py" C-m; '
                'tmux select-pane -t 1; '
                'tmux send-keys -t my_session2: "ros2 topic echo /aruco_single/pose_1" C-m; '
                'tmux select-pane -t 2; '
                'tmux send-keys -t my_session2: "ros2 run aruco_ros camera_info_publisher" C-m; '
                'tmux select-pane -t 3; '
                'tmux send-keys -t my_session2: "ros2 service call /x500_1/aircraft/take_off uas_std/srv/Float64 data:\ 3.0\" ; '
                'tmux attach-session -t my_session2;'  # Attach to the tmux session
            ],
            output='screen'  # Output the process logs to the screen
        ),
    ])
