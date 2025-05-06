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
                'tmux new-session -d -s my_session2; '
                'tmux split-window -h -p 50; '  # Split the window horizontally
                'tmux split-window -v -p 50; '  # Split one of the panes vertically
                'tmux select-pane -t 0; '  # Select pane 0 (image_subscriber)
                'tmux split-window -v -p 50; '  # Split the remaining pane vertically
                'tmux split-window -h -p 100; '  # Split the window horizontally
                'tmux select-pane -t 0; '  # Select pane 0 (image_subscriber)
                'tmux send-keys -t my_session2: "ros2 launch aruco_ros double.launch.py" C-m; '
                'tmux select-pane -t 1; '
                'tmux send-keys -t my_session2: "ros2 topic echo /aruco_double/pose" C-m; '
                'tmux select-pane -t 2; '
                'tmux send-keys -t my_session2: "ros2 topic echo /aruco_double/pose2" C-m; '
                'tmux select-pane -t 3; '
                'tmux send-keys -t my_session2: "ros2 run aruco_ros camera_info_publisher" C-m; '
                'tmux select-pane -t 4; '
                'tmux send-keys -t my_session2: "ros2 service call /x500_1/aircraft/take_off uas_std/srv/Float64 data:\ 3.0\" ; '
                'tmux attach-session -t my_session2;'  # Attach to the tmux session
            ],
            output='screen'  # Output the process logs to the screen
        ),
    ])
