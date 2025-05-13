#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>

#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "uav_landing/action/landing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "uav_landing_cpp/visibility_control.h"

// Text
#define Green_b "\033[1;32m"    // Green bold
#define Red_b "\033[1;31m"      // Red bold
#define Black_b "\033[1;30m"    // Black bold
#define Cyan_b "\033[1;36m"     // Cyan bold
#define Blue_b "\033[1;34m"     // Blue bold
#define White_b "\033[1;37m"    // White bold
#define Yellow_b_i "\033[1;93m" // Yellow bold, hgh intensity

// Background
#define Black_back "\033[40m"   // Black
#define White_back "\033[47m"   // White
#define Blue_back "\033[44m"    // Blue
#define Cyan_back "\033[46m"    // Cyan
#define Green_back "\033[42m"   // Green
#define Red_back  "\033[41m"    // Red

// Choosing a mode
#define MODE 1 // 0 - 2 in 1 marker (nested), 1 - Cascade of 3 Markers

// Formatting off
#define Reset "\033[0m"    

//                          //
//      Check TO_EDIT       //
//                          //

using namespace std::chrono_literals; // For timer, ms

namespace uav_landing_cpp
{
class LandingActionServer : public rclcpp::Node 
{
    public:
        using Landing = uav_landing::action::Landing;
        using GoalHandleLanding = rclcpp_action::ServerGoalHandle<Landing>;
        
        // Create Action server
        UAV_LANDING_CPP_PUBLIC
        explicit LandingActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("landing_action_server", options)
        {
            using namespace std::placeholders;

            // Initialization of a new action server
            this->action_server_ = rclcpp_action::create_server<Landing>(
                this,
                "landing",
                std::bind(&LandingActionServer::handle_goal, this, _1, _2),
                std::bind(&LandingActionServer::handle_cancel, this, _1),
                std::bind(&LandingActionServer::handle_accepted, this, _1));

            // Create subscribers of pose 
            pose_subscriber_0 = this->create_subscription<geometry_msgs::msg::PoseStamped>( // Subsrciber of pose from detector for cascade small marker
                "/aruco_single/pose", 10,
                std::bind(&LandingActionServer::poseCallback, this, std::placeholders::_1)
            );

            pose_subscriber_1 = this->create_subscription<geometry_msgs::msg::PoseStamped>( // Subsrciber of pose from detector for cascade medium or nested small marker
                "/aruco_single_1/pose_1", 10,
                std::bind(&LandingActionServer::poseCallback, this, std::placeholders::_1)
            );

            pose_subscriber_2 = this->create_subscription<geometry_msgs::msg::PoseStamped>( // Subsrciber of pose from detector for cascade big or nested big marker  
                "/aruco_single_2/pose_2", 10,
                std::bind(&LandingActionServer::poseCallback, this, std::placeholders::_1)
            );

            pose_subscriber_3 = this->create_subscription<std_msgs::msg::Int8>( // Smallest marker detection
                "/x500_1/aircraft/abort_smallest", 10,
                std::bind(&LandingActionServer::poseCallback_s, this, std::placeholders::_1)
            );

            pose_subscriber_4 = this->create_subscription<std_msgs::msg::Int8>( // Middle-sized marker detection
                "/x500_1/aircraft/abort_middle_sized", 10,
                std::bind(&LandingActionServer::poseCallback_m, this, std::placeholders::_1)
            );

            pose_subscriber_5 = this->create_subscription<std_msgs::msg::Int8>( // Biggest marker detection
                "/x500_1/aircraft/abort_biggest", 10,
                std::bind(&LandingActionServer::poseCallback_b, this, std::placeholders::_1)
            );

            // Create a publisher
            twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>( // Velocity publisher
                "/x500_1/aircraft/cmd_vel", 10
            );

            // Create publishers
            int_publisher_alive = this->create_publisher<std_msgs::msg::Int8>(  // Publisher of alive message
                "/x500_1/aircraft/server_alive", 10
            );

            // Create a timer
            timer_alive = this->create_wall_timer( // Timer for publishing alive messages
                100ms, std::bind(&LandingActionServer::timer_callback, this));  // If changing this, change also variable timer_period in the code

            // Create a timer 
            timer_ramp_x = this->create_wall_timer(
                3ms, std::bind(&LandingActionServer::timer_callback_ramp_x, this));
            timer_ramp_x->cancel(); // Cancel the timer at the beginning

            // Create a timer 
            timer_ramp_y = this->create_wall_timer( // Timer for ramp in y axe
                3ms, std::bind(&LandingActionServer::timer_callback_ramp_y, this));
            timer_ramp_y->cancel(); // Cancel the timer at the beginning
            
            // Create a timer 
            timer_land = this->create_wall_timer( // Timer for waiting after reaching the target height and before calling the action land
                50ms, std::bind(&LandingActionServer::timer_callback_land, this));
            timer_land->cancel(); // Cancel the timer at the beginning
            timer_land_allowed = 0;

            land_client_ = this->create_client<std_srvs::srv::Trigger>("/x500_1/aircraft/land"); // Client for calling the action land
            wait_for_act_land = 3.0;
            wait_for_act_land_temp = wait_for_act_land;                    // Waiting before calling the action land [s]
            counter_period = 0.05;                      // Period of timer for landing [s]

            speed_inc_dec = 0.00025;                    // speed adding by speed_inc_dec m / s

            // PID regulator initialization
            Kp = 0.35; 
            Ki = 0.003; 
            Kd = 0.0;
            I_active_zone = 0.3;            // Active zone for I [m], anti windup
            last_time = this->get_clock()->now();

            error_x_old = 0;        // Old error in x axe
            error_y_old = 0;        // Old error in y axe 
            integral_x = 0;         // Integral error in x axe
            integral_y = 0;         // Integral error in y axe
            //derivative_x = 0;    // Derivative error in x axe
            //derivative_y = 0;    // Derivative error in y axe

            // Set initialization for variables used for slow speeding up
            indicator_vel_x = 0;    // indicator_velocity_x == 0 - slow speeding up is needed || 1 - slow speeding up is in progress || 2 - slow speeding up is done
            indicator_vel_y = 0;    // indicator_velocity_y == 0 - slow speeding up is needed || 1 - slow speeding up is in progress || 2 - slow speeding up is done
            desired_vel_x = 0;      // Desired velocity
            desired_vel_y = 0;
            actual_vel_x = 0;       // Actual velocity in x axe
            actual_vel_y = 0;       // Actual velocity in y axe
            velocity_x_timer = 0;   // Actual velocity for timer
            velocity_y_timer = 0;   // Actual velocity for timer
            z = 0;                  // Height of a drone
            x_orient = 0;           // Orientation in x axe
            y_orient = 0;           // Orientation in y axe
            z_orient = 0;           // Orientation in z axe
            w_orient = 0;           // Orientation in w axe
            target_height = 0;      // Target height for setttin the action goal
            goal_got = 0;               // Variable for detecting if an action goal was received
            goal_abort = 0;             // Variable for detecting if an action goal was aborted
            goal_abort_detector = 0;    // Detecting if an action goal was aborted because of non-detection of the markers
            goal_abort_difference = 0;  // Detecting if an action goal was aborted because of inconsistent detection
            diff_x_old = 0;             // Old difference in x axe
            diff_y_old = 0;             // Old difference in y axe
            diff_z_old = 0;             // Old difference in z axe
            diff_x = 0;                 // Difference in x axe
            diff_y = 0;                 // Difference in y axe
            diff_z = 0;                 // Difference in z axe
            diff_allow = 0;             // Allowing difference in x axe
            poseCallback_s_var = 0;         // Variable for detecting if the smallest marker was detected
            poseCallback_m_var = 0;         // Variable for detecting if the middle-sized marker was detected
            poseCallback_b_var = 0;         // Variable for detecting if the biggest marker was detected

            // Create a Twist message and initialize it
            twist_msg.linear.x = 0;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = 0;
            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;
            twist_msg.angular.z = 0;

            Int8_msg_alive.data = 5;    // Server alive message, 5 because action client "previous value" is starting from 1

            // Setting variable for indicating end of landing
            landing_ind = 2;    /*  2 - landig was started but position of the drone is less than landing_high_limit, 
                                    0 - FALSE,landing was started but position of the drone is more than landing_high_limit, 
                                    1 - TRUE, landing was started previous state was 0 and now position of the drone is less than landing_high_limit*/

            RCLCPP_INFO(this->get_logger(), Cyan_b "[INFO] Marker detector action server has started." Reset); // Informing about starting a node
        }

        // Destructor
        /*~LandingActionServer()
        {
            twist_msg.linear.x = 2.0;       
            twist_msg.linear.y = 3.0;
            twist_msg.linear.z = 4.0;
            
            if (rclcpp::ok()) {
                for (int i = 0; i < 10; i++) {
                    twist_publisher_->publish(twist_msg);
                    rclcpp::sleep_for(std::chrono::milliseconds(100));   // Allow time for publishing
                    RCLCPP_INFO(this->get_logger(), Red_b "Publishing..." Reset);
                }
            }
            else
                RCLCPP_INFO(this->get_logger(), Red_b "rclcpp not here." Reset);

            RCLCPP_INFO(this->get_logger(), Red_b "Shutting down action server... Sending 0 velocities." Reset);
        }*/

    private:
        rclcpp_action::Server<Landing>::SharedPtr action_server_;

        // Timer method for publishing alive messages
        void timer_callback()
        {
            int_publisher_alive->publish(Int8_msg_alive);
            Int8_msg_alive.data++;
        }

        // Startup acceleration ramp, x
        void timer_callback_ramp_x()
        {
            //RCLCPP_INFO(this->get_logger(), "timer_callback_ramp_x");
            if(indicator_vel_x == 1)                    // indicator_vel_x == 1 - slow speeding up is in progress
            {
                // Increasing or decreasing actual velocity based on desired velocity
                if(desired_vel_x != actual_vel_x)
                {
                    actual_vel_x = (desired_vel_x > actual_vel_x) ? actual_vel_x + speed_inc_dec : actual_vel_x - speed_inc_dec;
                    velocity_x_timer = actual_vel_x;
                }

                // Checking if desired velocity was reached
                if((desired_vel_x >= 0 && velocity_x_timer >= desired_vel_x) || (desired_vel_x <= 0 && velocity_x_timer <= desired_vel_x))
                {
                    indicator_vel_x = 2;                // indicator_vel_x == 2 - slow speeding up is done

                    for(int i = 0; i < 5; i++)          // Informing when ramp in x axe is done
                        RCLCPP_INFO(this->get_logger(), Black_back White_b "[INFO] Ramp y DONE." Reset);
                        
                    timer_ramp_x->cancel();                // Cancel the timer
                }
            }
        }

        // Startup acceleration ramp, y
        void timer_callback_ramp_y()
        {
            //RCLCPP_INFO(this->get_logger(), "timer_callback_ramp_x");
            if(indicator_vel_y == 1)                    // indicator_vel_y == 1 - slow speeding up is in progress
            {
                // Increasing or decreasing actual velocity based on desired velocity
                if(desired_vel_y != actual_vel_y)
                {
                    actual_vel_y = (desired_vel_y > actual_vel_y) ? actual_vel_y + speed_inc_dec : actual_vel_y - speed_inc_dec;
                    velocity_y_timer = actual_vel_y;
                }

                // Checking if desired velocity was reached
                if((desired_vel_y >= 0 && velocity_y_timer >= desired_vel_y) || (desired_vel_y <= 0 && velocity_y_timer <= desired_vel_y))
                {
                    indicator_vel_y = 2;                // indicator_vel_y == 2 - slow speeding up is done

                    for(int i = 0; i < 5; i++)         // Informing when ramp in y axe is done
                        RCLCPP_INFO(this->get_logger(), White_back Black_b "[INFO] Ramp y DONE." Reset);
                    
                    timer_ramp_y->cancel();                // Cancel the timer
                }
            }
        }

        // Timer for landing
        void timer_callback_land()
        {
            if(wait_for_act_land_temp >= 0)
                RCLCPP_INFO(this->get_logger(), Green_b "Action land will be triggered in %.2f secs." Reset, wait_for_act_land_temp);

            if(wait_for_act_land_temp <= 0)
            {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto result_future = land_client_->async_send_request(request);

                timer_land->cancel();       // Cancel the timer
                RCLCPP_INFO(this->get_logger(), Green_b "Action land has been triggered, disconnecting the action server." Reset);
                rclcpp::shutdown();         // Turn off the action server
            }
            
            if(wait_for_act_land_temp >= 0)
                wait_for_act_land_temp = wait_for_act_land_temp - counter_period; // Decreasing countdown for landing
        }

        // Processing the abort messages
        void poseCallback_s(const std_msgs::msg::Int8::SharedPtr msg_s) 
        {
            if(msg_s->data == 1) // Marker was not detected
                poseCallback_s_var = 1;
            else
                poseCallback_s_var = 0;

            //RCLCPP_INFO(this->get_logger(), "poseCallback_s_var = %d", poseCallback_s_var);
            //RCLCPP_INFO(this->get_logger(), "poseCallback_s msg_s->data = %d", msg_s->data);
        }

        // Processing the abort messages
        void poseCallback_m(const std_msgs::msg::Int8::SharedPtr msg_m) 
        {
            if(msg_m->data == 1) // Marker was not detected
                poseCallback_m_var = 1;
            else
                poseCallback_m_var = 0;

            //RCLCPP_INFO(this->get_logger(), "poseCallback_m_var = %d", poseCallback_m_var);
            //RCLCPP_INFO(this->get_logger(), "poseCallback_m msg_m->data = %d", msg_m->data);
        }

        // Processing the abort messages
        void poseCallback_b(const std_msgs::msg::Int8::SharedPtr msg_b) 
        {
            if(msg_b->data == 1) // Marker was not detected
                poseCallback_b_var = 1;
            else
                poseCallback_b_var = 0;
                
            //RCLCPP_INFO(this->get_logger(), "poseCallback_b_var = %d", poseCallback_b_var);
            //RCLCPP_INFO(this->get_logger(), "poseCallback_b msg_b->data = %d", msg_b->data);
            }

        // Processing the pose messages
        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
        {   
            /*
            Procedure:
            Height loading
            According to height - read x and y data from the message according to msg->header.frame_id
            Horizontal position deviation control
            */
           
           // Load height
           z = msg->pose.position.z;

           // Goal was aborted because of non-detection of the markers
           if(goal_abort_detector == 1)
           {
               twist_msg.linear.x = 0.0;       
               twist_msg.linear.y = 0.0;
               twist_msg.linear.z = 0.0;
               
               RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "Velocity: linear.x = %.2f, linear.y = %.2f, linear.z = %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z);
               RCLCPP_INFO(this->get_logger(), Red_b "Goal was aborted --- DETECTOR ABSENCE ---" Reset);

               twist_publisher_->publish(twist_msg);
           }

           // Goal was aborted because of inconsistent detection
           if(goal_abort_difference == 1)
           {
               twist_msg.linear.x = 0.0;       
               twist_msg.linear.y = 0.0;
               twist_msg.linear.z = 0.0;
               
               RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "Velocity: linear.x = %.2f, linear.y = %.2f, linear.z = %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z);
               RCLCPP_INFO(this->get_logger(), Red_b "Goal was aborted --- INCONSISTENT DETECTION ---" Reset);

               twist_publisher_->publish(twist_msg); // Publish the message
           }

            if(goal_got == 1 && goal_abort_detector == 0 && goal_abort_difference == 0) // Goal has been received
            {
                // Initialize x and y
                float x = 0;
                float y = 0;
                
                #if MODE
                    // Marker sizes [m]
                    float big_marker_s = 0.428;
                    float middle_marker_s = 0.212;
                    float small_marker_s = 0.129;

                    // White gap [m]
                    float white_bm_ms = 0.039; // White space between big and middle marker and at the same time between medium and small marker

                    // Marker positions [m]
                    float big_marker_pos_x = big_marker_s / 2.0;
                    //float big_marker_pos_y = big_marker_s / 2; // Unused
                    float middle_marker_pos_x = big_marker_s + white_bm_ms + middle_marker_s / 2.0; // / 2.0 for splitting the marker in half
                    float middle_marker_pos_y = big_marker_s - middle_marker_s / 2.0;
                    float small_marker_pos_x = big_marker_s + white_bm_ms + small_marker_s / 2.0;
                    float small_marker_pos_y = big_marker_s - middle_marker_s - white_bm_ms - small_marker_s / 2.0;

                    // From sizes of landing markers, the landing point is (big_marker_s + middle_marker_s) / 2 [m] for x axe and big_marker_s / 2 [m] for y axe
                    float middle_x = (big_marker_s + white_bm_ms + middle_marker_s) / 2.0;
                    float middle_y = big_marker_s / 2.0;
                #else
                    // Marker sizes [m]
                    float big_marker_s = 0.428;
                    //float small_marker_s = 0.1;    // Unused

                    // Marker positions [m]
                    // float big_marker_pos_x = big_marker_s / 2;  // Unused
                    //float big_marker_pos_y = big_marker_s / 2;   // Unused
                    float small_marker_pos_x = 0.134;
                    float gap_bs = 0.083;                           // Gap between the biggest marker and the smallest marker
                    float small_marker_pos_y = big_marker_s - gap_bs - small_marker_pos_x / 2.0; // Small marker is exactly in the middle of the biggest marker'r white space

                    // From sizes of landing markers, the landing point position is in the middle of the big marker
                    float middle_x = big_marker_s / 2.0;
                    float middle_y = big_marker_s / 2.0;
                #endif

                /*
                Marker layout - all markers are squares
                An image ↓↓↓ is for reference only

                XXXXXXXXXXXXXXXXXXXX
                X000000000000XXXXXXX
                X000000000000X222XXX
                X000000000000X222XXX
                X000000000000XXXXXXX
                X00000000X000X11111X
                X000000000000X11111X
                X000000000000X11111X
                X000000000000X11111X
                XXXXXXXXXXXXXXXXXXXX

                X - Landing point - middle of the object of markers
                0 - Biggest marker
                1 - Middle-sized marker
                2 - Smallest marker
                X - White gap
                */

                #if MODE
                    float horizontal_dev_x = 0;            // horizontal_deviation [m] - Horizontal deviation from center of marker in axe x
                    float horizontal_dev_y = 0;            // horizontal_deviation [m] - Horizontal deviation from center of marker in axe y
                    float level_2 = 2.25;                  // 2.25 m height for landing
                    float level_1 = 0.8;                   // 1 m height for landing
                #else
                    float horizontal_dev_x = 0;            // horizontal_deviation [m] - Horizontal deviation from center of marker in axe x
                    float horizontal_dev_y = 0;            // horizontal_deviation [m] - Horizontal deviation from center of marker in axe y
                    float level_2 = 0.8;                   // height for landing
                    float level_1 = 0;                     // Unused but for consistency
                #endif

                // Determining desired distance from marker in axes x and y
                // Because of velocity_x = velocity_x * (-1); below also mutiplying "* (-1)" needs to be here
                #if MODE
                    if(z > level_2)                         // Detecting the biggest marker but descending to middle
                    {
                        horizontal_dev_x = (big_marker_pos_x - middle_x) * (-1);
                        horizontal_dev_y = 0;
                    }
                    else if (z <= level_2 && z >= level_1)  // Detecting middle-sized marker
                    {
                        horizontal_dev_x = (middle_marker_pos_x - middle_x) * (-1);
                        horizontal_dev_y = (middle_marker_pos_y - middle_y) * (-1);
                    }
                    else                                    // Detecting smallest marker
                    {
                        horizontal_dev_x = (small_marker_pos_x - middle_x) * (-1);
                        horizontal_dev_y = (small_marker_pos_y - middle_y) * (-1);
                    }
                #else
                    if(z > level_2)                         // Detecting the biggest marker but descending to middle
                    {
                        horizontal_dev_x = 0;
                        horizontal_dev_y = 0;
                    }
                    else                                    // Detecting smallest marker
                    {
                        horizontal_dev_x = (small_marker_pos_x - middle_x) * (-1);
                        horizontal_dev_y = (small_marker_pos_y - middle_y) * (-1);
                    }
                #endif

                float no_inp_bord = 0.15;          // Border for interval without input [m]
                // Cancelling the goal because of non - detection of the markers
                if(z > level_2 - no_inp_bord && poseCallback_b_var == 1) // Marker was not detected
                {
                    goal_abort_detector = 1;
                    //RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Goal aborted because of non-detection of the biggest marker." Reset);
                }
                else if(z <= level_2 - no_inp_bord && z >= level_1 && poseCallback_m_var == 1) // Marker was not detected
                {
                    goal_abort_detector = 1;
                    //RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Goal aborted because of non-detection of the middle - sized marker." Reset);
                }
                else if(z < level_1 - no_inp_bord && poseCallback_s_var == 1) // Marker was not detected
                {
                    goal_abort_detector = 1;
                    //RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Goal aborted because of non-detection of the smallest marker." Reset);
                }
                
                // Decide which state is active
                // Camera coordinates: x: →, y: ↓
                // If more than level_2 m, between level_2 and level_1 m or below level_1 m
                // Load x and y positions from the PoseStamped message
                if((z > level_2 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_2") || (z <= level_2 && z >= level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_1") || (z < level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_0"))
                {
                    // Rotation consideration
                    x_orient = msg->pose.orientation.x;
                    y_orient = msg->pose.orientation.y;
                    z_orient = msg->pose.orientation.z;
                    w_orient = msg->pose.orientation.w;

                    float s = 2 * (w_orient * z_orient + x_orient * y_orient);
                    float c = w_orient * w_orient + x_orient * x_orient - y_orient * y_orient - z_orient * z_orient;
                    float yaw_rad = std::atan2(s, c); // yaw (z-axis rotation)
                    float yaw_deg = yaw_rad * 180.0 / M_PI;
                    float angle_rot = yaw_deg + 360; // Angle of rotation in z axe
                    //RCLCPP_INFO(this->get_logger(), "Yaw (deg), angle_rot = %.2f", angle_rot);

                    float temp_hor_x = horizontal_dev_x; // Temporary variable for rotation
                    float temp_hor_y = horizontal_dev_y; // Temporary variable for rotation
                    horizontal_dev_x = temp_hor_x * cos(angle_rot / 180.0 * M_PI) - temp_hor_y * sin(angle_rot / 180.0 * M_PI); // New x coordinate for deviation in x axe .. Rotation matrix
                    horizontal_dev_y = temp_hor_x * sin(angle_rot / 180.0 * M_PI) + temp_hor_y * cos(angle_rot / 180.0 * M_PI); // New y coordinate for deviation in y axe .. Rotation matrix

                    //RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "horizontal_dev_x = %.2f, horizontal_dev_y = %.2f", horizontal_dev_x, horizontal_dev_y);

                    x = msg->pose.position.x + horizontal_dev_x;
                    y = msg->pose.position.y + horizontal_dev_y;
                
                    // Counting difference
                    float diff_threshold = 0.5;      // TO_EDIT Threshold for difference
                    float level_HIGH = 6.0;          // If the drone is high, also higher noise is acceptable 
                    if(z > level_HIGH) 
                        diff_threshold = 1.0;       // TO_EDIT Threshold for difference

                    // Ignore first difference
                    if(diff_allow == 0)
                        diff_allow++;
                    else
                    {
                        diff_x = x - diff_x_old;    // Difference in x axe
                        diff_y = y - diff_y_old;    // Difference in y axe
                        diff_z = z - diff_z_old;    // Difference in z axe
                    }
                    
                    //RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "diff_x = %.2f, diff_y = %.2f, diff_x_old = %.2f, diff_y_old = %.2f, x = %.2f, y = %.2f,", diff_x, diff_y, diff_x_old, diff_y_old, x ,y);
                    
                    // Abort the goal
                    // diff_threshold and (- diff_threshold) are because in diff_(x, y, z) = (x, y, z) - diff_(x, y, z)_old; above is not absolute value
                    if(diff_x > diff_threshold || diff_x < -diff_threshold || diff_y > diff_threshold || diff_y < -diff_threshold || diff_z > diff_threshold || diff_z < -diff_threshold)
                        goal_abort_difference = 1;

                    // Saving actual deviaton values for the next iteration
                    diff_x_old = x;
                    diff_y_old = y;
                    diff_z_old = z;

                    // PID regulator
                    rclcpp::Time now = this->get_clock()->now();
                    rclcpp::Duration duration = now - last_time;
                    double dt = duration.seconds();  // dt
                    //double freq = 1.0 / dt; // Frequency of the PID controller
                    //RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "freq = %.2f", freq);
                    last_time = now;

                    //derivative_x = (x - error_x_old) / dt;    // Derivative error in x axe
                    //derivative_y = (y - error_y_old) / dt;    // Derivative error in y axe
                    //RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "derivative_x = %.2f, derivative_y = %.2f", derivative_x, derivative_y);
                    integral_x += x * dt;                  // Integral error in x axe
                    integral_y += y * dt;                  // Integral error in y axe

                    if(abs(x) > I_active_zone)
                        integral_x = 0;                    // Anti - windup for integral error in x axe
                    
                    if(abs(y) > I_active_zone)
                        integral_y = 0;                    // Anti - windup for integral error in y axe
                    //RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "integral_x = %.2f, integral_y = %.2f", integral_x, integral_y);

                    // World coordinates: x: ↑, y: ←
                    float velocity_x = Kp * y + Ki * integral_y; //+ Kd * derivative_y;  Velocity in world x coordinate
                    float velocity_y = Kp * x + Ki * integral_x; //+ Kd * derivative_x;  Velocity in world y coordinate

                    error_x_old = x;    // Old error in x axe
                    error_y_old = y;    // Old error in y axe

                    velocity_x = velocity_x * (-1);             // X of a world coordinate system and y of an image coordinate system are oppositely oriented
                    velocity_y = velocity_y * (-1);             // Y of a world coordinate system and x of an image coordinate system are oppositely oriented

                    // Velocity saturation limit
                    float vel_saturation = 0.25;                // Velocity saturation [m]

                    // Slow speeding up after start of the regulator_node_server
                    if(indicator_vel_x == 0 && velocity_x != 0) // indicator_vel_x == 0 - speeding up slowly is needed
                    {
                        indicator_vel_x = 1;                    // indicator_vel_x == 1 - slow speeding up is in progress

                        // Setting desired velocity from saturation velocity level, velocity can be also negative and so saturation level need to be also negative
                        if(velocity_x < 0 && velocity_x < - vel_saturation)    
                            desired_vel_x = - vel_saturation;                   
                        else if (velocity_x > 0 && velocity_x > vel_saturation)
                            desired_vel_x = vel_saturation;
                        else
                            desired_vel_x = velocity_x;

                        timer_ramp_x->reset(); // Reset the timer for ramping
                        RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Ramp x STARTED." Reset);
                    }
                    
                    //  Taking velocity from timer
                    if(indicator_vel_x == 1)  
                        velocity_x = velocity_x_timer; // Actual velocity for timer

                    if(indicator_vel_y == 0 && velocity_y != 0) // indicator_vel_y == 0 - speeding up slowly is needed
                    {
                        indicator_vel_y = 1;                    // indicator_vel_y == 1 - slow speeding up is in progress
                        
                        // Setting desired velocity from saturation velocity level, velocity can be also negative and so saturation level need to be also negative
                        if(velocity_y < 0 && velocity_y < - vel_saturation)
                            desired_vel_y = - vel_saturation;
                        else if (velocity_y > 0 && velocity_y > vel_saturation)
                            desired_vel_y = vel_saturation;
                        else
                            desired_vel_y = velocity_y;

                        timer_ramp_y->reset(); // Reset the timer for ramping
                        RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Ramp y STARTED." Reset);
                    }
                    
                    //  Taking velocity from timer
                    if(indicator_vel_y == 1)  
                        velocity_y = velocity_y_timer; // Actual velocity from timer

                    // Apply saturation for velocity_x
                    velocity_x = (velocity_x > vel_saturation) ? vel_saturation : velocity_x;
                    velocity_x  = (velocity_x < -vel_saturation) ? -vel_saturation : velocity_x;

                    // Apply saturation for velocity_y
                    velocity_y = (velocity_y > vel_saturation) ? vel_saturation : velocity_y;
                    velocity_y = (velocity_y < -vel_saturation) ? -vel_saturation : velocity_y;

                    // Fill and publish a Twist message
                    twist_msg.linear.x = velocity_x; 
                    twist_msg.linear.y = velocity_y;

                    // Landing speed
                    float z_landing_vel = 0;
                    #if MODE
                        if (z > level_2)
                            z_landing_vel = -0.2;	
                        else if ((z <= level_2) && (z >= level_1))
                            z_landing_vel = -0.15;	
                        else
                            z_landing_vel = -0.1;
                    #else

                        float safety_level = 1.0; // For lowering the speed of landing
                        if (z > level_2 + safety_level)
                            z_landing_vel = -0.2;
                        else if (z <= level_2 + safety_level && z > level_2)
                            z_landing_vel = -0.15;
                        else
                            z_landing_vel = -0.1;
                    #endif	

                    float z_landing_vel_stop = 0.0; 	// Landing speed
                    float landing_high_limit = target_height;	    // Stop decreasing when this level is reached [m]

                    // Horizontal threshold
                    float horizontal_thrshld = 0;       // TO_EDIT horizontal_threshold [m], maximum desired regulation deviation while reaching desired point
                    if (z > level_2)
                        horizontal_thrshld = 0.15;	
                    else if ((z <= level_2) && (z >= level_1))
                        horizontal_thrshld = 0.1;	
                    else
                        horizontal_thrshld = 0.03;	

                    float vertical_error = 0.05;       // Vertical error of detector for z axe [m]

                    // If height in z is more than z_landing_vel_stop height &&
                    // If x or y are in interval (horizontal_thrshld; - horizontal_thrshld), then vertical movement is allowed

                    if (((goal_abort == 0) && (z < level_2 + no_inp_bord && z > level_2 - no_inp_bord)) || ((goal_abort == 0) && (z < level_1 + no_inp_bord && z > level_1 - no_inp_bord)))   // Avoiding two inputs and just descending without input
                        twist_msg.linear.z = z_landing_vel;
                    else
                        twist_msg.linear.z = (goal_abort == 0 && msg->pose.position.z > landing_high_limit && x < horizontal_thrshld && x > - horizontal_thrshld && y < horizontal_thrshld && y > - horizontal_thrshld) ? z_landing_vel : z_landing_vel_stop;
                    
                    // If goal was aborted, stop the drone
                    if(goal_abort == 1 || goal_abort_difference == 1 || goal_abort_detector == 1)
                    {
                        twist_msg.linear.x = 0.0;       
                        twist_msg.linear.y = 0.0;
                        twist_msg.linear.z = 0.0;
                    }

                    if(msg->pose.position.z >= landing_high_limit + vertical_error) 
                        landing_ind = 0;                                                                                    // Position of the drone is more than landing_high_limit, changing state of landing
                    else if(landing_ind == 0 && msg->pose.position.z <= landing_high_limit && timer_land_allowed == 0)      // If position of the drone is in axe z less of same as Landing and value of landing_ind is 0, action land can be called
                    {
                        landing_ind = 1;
                        for(int i = 0; i < 100; i++)
                            RCLCPP_INFO(this->get_logger(), Yellow_b_i "[INFO] Automatic landing DONE, regulation to minimum regulation deviation in axes x and y is still working." Reset);
                            
                        timer_land_allowed = 1;             // For height verification
                        timer_land->reset();                // Allow to call the action land
                    }

                    // No angular movement
                    twist_msg.angular.x = 0.0;		
                    twist_msg.angular.y = 0.0;
                    twist_msg.angular.z = 0.0;

                    // Publish only data from used frame
                    if((z > level_2 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_2") || (z <= level_2 && z >= level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_1") || (z < level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_0"))
                    {
                        twist_publisher_->publish(twist_msg);

                        // Log what was published
                        RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "Input Pose: x = %.2f, y = %.2f -> Velocity: linear.x = %.2f, linear.y = %.2f, linear.z = %.2f",
                                    x, y, twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z);

                        //RCLCPP_INFO(this->get_logger(), White_b "[DATA] " Reset "z = %.2f, target_height = %.2f, timer_land_allowed = %.2f", z, target_height, timer_land_allowed);
                        if(z > target_height && timer_land_allowed == 1)
                            {
                                wait_for_act_land_temp = wait_for_act_land;
                                RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Resetting the timer for calling action Land." Reset);
                            }
                    }
                }
            }
        }

        // Handling new goals
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Landing::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with height %.2f m", goal->target_height);

            goal_got = 1;
            goal_abort = 0;
            diff_x = 0;
            diff_y = 0;
            diff_z = 0;
            diff_x_old = 0;
            diff_y_old = 0;
            diff_z_old = 0;
            diff_allow = 0;
            poseCallback_s_var = 0;
            poseCallback_m_var = 0;
            poseCallback_b_var = 0;
            goal_abort_difference = 0;
            goal_abort_detector = 0;
            indicator_vel_x = 0;    // indicator_velocity_x == 0 - slow speeding up is needed || 1 - slow speeding up is in progress || 2 - slow speeding up is done
            indicator_vel_y = 0;    // indicator_velocity_y == 0 - slow speeding up is needed || 1 - slow speeding up is in progress || 2 - slow speeding up is done

            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Dealing with cancellation
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLanding> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Accepts a new goal and starts processing it:
        void handle_accepted(const std::shared_ptr<GoalHandleLanding> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread

            std::thread{std::bind(&LandingActionServer::execute, this, _1), goal_handle}.detach();
        }

        // All further processing and updates are done in the execute method in the new thread
        void execute(const std::shared_ptr<GoalHandleLanding> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), Green_b "Executing goal" Reset);
            rclcpp::Rate loop_rate(1);
            target_height = goal_handle->get_goal()->target_height;
            auto feedback = std::make_shared<Landing::Feedback>();
            auto result = std::make_shared<Landing::Result>();

            while (target_height < z && rclcpp::ok() && goal_abort_difference != 1 && goal_abort_detector != 1) 
            {
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) 
                {
                    result->status_code = 2;    // Goal canceled
                    RCLCPP_INFO(this->get_logger(), Red_b "Goal canceled" Reset);
                    goal_abort = 1;
                    goal_got = 0;
                    goal_handle->canceled(result);

                    return;
                }

                feedback->set__current_height(z);
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), Blue_b "Publish feedback" Reset);
                
                loop_rate.sleep();
            }
            
            // Cancelling because of inconsistent detection or absence of the detector
            if(goal_abort_difference == 1 || goal_abort_detector == 1)
            {
                result->status_code = 2;    // Goal aborted

                if(goal_abort_difference == 1)
                    RCLCPP_INFO(this->get_logger(), Red_back Black_b "Goal was aborted --- INCONSISTENT DETECTION ---" Reset);
                else if(goal_abort_detector == 1)
                    RCLCPP_INFO(this->get_logger(), Red_back Black_b "Goal was aborted --- DETECTOR ABSENCE ---" Reset);

                goal_handle->abort(result);

                return;
            }

            // Check if goal is done
            if (rclcpp::ok()) 
            {
                result->status_code = 1; // Goal succeeded

                //RCLCPP_INFO(this->get_logger(), Blue_b "HERE" Reset);

                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), Green_b "Goal succeeded" Reset);
            }
        }

        // Member variables for proportional gain, slow speeding up, subscribers publisher and difference
        int goal_got;
        int goal_abort;
        int goal_abort_detector;
        int goal_abort_difference;
        int diff_allow;
        int poseCallback_s_var;
        int poseCallback_m_var;
        int poseCallback_b_var;
        float Kp;
        float Ki;
        float Kd;
        float I_active_zone;
        float error_x_old;
        float error_y_old;
        float integral_x;               // Integral error in x axe
        float integral_y;               // Integral error in y axe
        //float derivative_x;             // Derivative error in x axe
        //float derivative_y;             // Derivative error in y axe
        float velocity_x_timer;
        float velocity_y_timer;
        float diff_x_old;
        float diff_y_old;
        float diff_z_old;
        float diff_x;
        float diff_y;
        float diff_z;
        float speed_inc_dec;
        float indicator_vel_x;
        float indicator_vel_y;
        float actual_vel_x;
        float actual_vel_y;
        float desired_vel_x;
        float desired_vel_y;
        int landing_ind;
        float z;
        float x_orient;
        float y_orient;
        float z_orient;
        float w_orient;
        float target_height;
        float counter_period;
        float wait_for_act_land_temp;
        float wait_for_act_land;
        float timer_land_allowed;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_0;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_1;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_2;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr pose_subscriber_3;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr pose_subscriber_4;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr pose_subscriber_5;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_alive;
        geometry_msgs::msg::Twist twist_msg;
        std_msgs::msg::Int8 Int8_msg_alive;
        rclcpp::TimerBase::SharedPtr timer_alive;
        rclcpp::TimerBase::SharedPtr timer_ramp_x;
        rclcpp::TimerBase::SharedPtr timer_ramp_y;
        rclcpp::TimerBase::SharedPtr timer_land;
        rclcpp::Time last_time;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(uav_landing_cpp::LandingActionServer)