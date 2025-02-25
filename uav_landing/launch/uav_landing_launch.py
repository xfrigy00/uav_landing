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
                'tmux send-keys -t my_session2: "ros2 run uav_landing image_subscriber" C-m; '
                'tmux split-window -h; '  # Split the window horizontally
                'tmux split-window -v; '  # Split one of the panes vertically
                'tmux send-keys -t my_session2: "ros2 run uav_landing aruco_detector" C-m; '  # Start the `uav_landing aruco_detector` in pane 3
                'tmux select-pane -t 0; '  # Select pane 0 (image_subscriber)
                'tmux split-window -v; '  # Split the remaining pane vertically
                'tmux send-keys -t my_session2: "ros2 run image_tools cam2image" C-m; '  # Start the `image_tools cam2image` node in pane 1
                'tmux attach-session -t my_session2;'  # Attach to the tmux session
            ],
            output='screen'  # Output the process logs to the screen
        ),
    ])

