#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>

#include "uav_landing/action/landing.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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
#define Green_back "\033[42m"    // Green
#define Red_back  "\033[41m"    // Red

// Choosing a mode
#define MODE 1 // 0 - 2 in 1 marker, 1 - Cascade of 3 Markers

// Formatting off
#define Reset "\033[0m"         

using namespace std::chrono_literals; // For timer, ms

namespace uav_landing_cpp
{
    // Creating a node
    class LandingActionClient : public rclcpp::Node 
    {
        public:
            using Landing = uav_landing::action::Landing;
            using GoalHandleLanding = rclcpp_action::ClientGoalHandle<Landing>;

            // Constructor for the LandingActionClient class initializes the node name as landing_action_client
            explicit LandingActionClient(const rclcpp::NodeOptions & options)
            : Node("landing_action_client", options)
            {
                // The constructor also instantiates a new action client
                this->client_ptr_ = rclcpp_action::create_client<Landing>(
                this,
                "landing");

                // Create ubscribers
                pose_sub_0 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10,
                    std::bind(&LandingActionClient::poseCallback, this, std::placeholders::_1)
                );

                pose_sub_1 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single_1/pose_1", 10,
                    std::bind(&LandingActionClient::poseCallback, this, std::placeholders::_1)
                );

                pose_sub_2 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single_2/pose_2", 10,
                    std::bind(&LandingActionClient::poseCallback, this, std::placeholders::_1)
                );

                // Create publishers
                int_publisher_0 = this->create_publisher<std_msgs::msg::Int8>(
                    "/x500_1/aircraft/marker_detection", 10
                );

                int_publisher_1 = this->create_publisher<std_msgs::msg::Int8>(
                    "/x500_1/aircraft/marker_detection_1", 10
                );

                int_publisher_2 = this->create_publisher<std_msgs::msg::Int8>(
                    "/x500_1/aircraft/marker_detection_2", 10
                );

                int_publisher_3 = this->create_publisher<std_msgs::msg::Int8>(
                    "/x500_1/aircraft/abort_smallest", 10
                );

                int_publisher_4 = this->create_publisher<std_msgs::msg::Int8>(
                    "/x500_1/aircraft/abort_middle_sized", 10
                );

                int_publisher_5 = this->create_publisher<std_msgs::msg::Int8>(
                    "/x500_1/aircraft/abort_biggest", 10
                );

                alive_subscriber = this->create_subscription<std_msgs::msg::Int8>(
                    "/x500_1/aircraft/server_alive", 10,
                    std::bind(&LandingActionClient::alive_callback, this, std::placeholders::_1)
                );

                // Create a timer
                timer_ = this->create_wall_timer(
                    500ms, std::bind(&LandingActionClient::timer_callback, this));  // If changing this, change also variable timer_period in the code

                // Create a publisher
                twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                    "/x500_1/aircraft/cmd_vel", 10
                );

                timer_period = 0.5;          // Timer period in seconds

                msg_time_prev_0 = -1;        // Variable with previous time stamp
                msg_time_actual_0 = -1;      // Variable with actual time stamp
                marker_detected_0 = 0;       // Output_0 from detector

                msg_time_prev_1 = -1;        // Variable with previous time stamp
                msg_time_actual_1 = -1;      // Variable with actual time stamp
                marker_detected_1 = 0;       // Output_1 from detector

                msg_time_prev_2 = -1;        // Variable with previous time stamp
                msg_time_actual_2 = -1;      // Variable with actual time stamp
                marker_detected_2 = 0;       // Output_2 from detector

                goal_sent = 0;               // Variable for detection of sending the goal

                counter_ND_s = 0;            // Counter of non-detection of the smallest marker
                counter_ND_m = 0;            // Counter of non-detection of the middle-sized marker
                counter_ND_b = 0;            // Counter of non-detection of the biggest marker
                    
                mark_t_abort = 4;              // Time in seconds after which the goal is abortled
                mark_t_abort = mark_t_abort / timer_period;             // How many times timer must be called to reach mark_t_abort seconds

                server_t_abort = 3;            // Time in seconds after which the action server is not ready
                server_t_abort = server_t_abort / timer_period;         // How many times timer must be called to reach server_t_abort seconds

                smallest_abort = 0;         // Cancelled goal becasue of non - detection of the smallest marker
                middle_abort = 0;           // Cancelled goal becasue of non - detection of the middle-sized marker
                biggest_abort = 0;          // Cancelled goal becasue of non - detection of the biggest marker

                allow_client = 1;           // Variable for allowing the client to function, 1 - allowed, 0 - not allowed, if 0, Action server not available after waiting
                client_nr = 0;              // Number of client tries

                alive_check_new = 1;            // New alive check message
                alive_check_prev = 1;           // Previous alive check message

                // Create a Twist message
                twist_msg.linear.x = 0;
                twist_msg.linear.y = 0;
                twist_msg.linear.z = 0;
                twist_msg.angular.x = 0;
                twist_msg.angular.y = 0;
                twist_msg.angular.z = 0;

                RCLCPP_INFO(this->get_logger(), Cyan_b "[INFO] Marker detector action client has started." Reset); // Informing about starting a node
            }

            void send_goal()
            {
                using namespace std::placeholders;

                //RCLCPP_ERROR(this->get_logger(), "HERE_1");

                int secs_w_act_s = 1;
                if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(secs_w_act_s))) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
                    allow_client = 0;

                    rclcpp::shutdown();
                    return;
                }

                auto goal_msg = Landing::Goal();
                goal_msg.target_height = 0.6; //TO_EDIT

                RCLCPP_INFO(this->get_logger(), "Sending goal: %.2f", goal_msg.target_height);

                auto send_goal_options = rclcpp_action::Client<Landing>::SendGoalOptions();
                send_goal_options.goal_response_callback =
                std::bind(&LandingActionClient::goal_response_callback, this, _1);
                send_goal_options.feedback_callback =
                std::bind(&LandingActionClient::feedback_callback, this, _1, _2);
                send_goal_options.result_callback =
                std::bind(&LandingActionClient::result_callback, this, _1);
                this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            }

        private:
            rclcpp_action::Client<Landing>::SharedPtr client_ptr_;
            
            // When the client receives a message from the server, it will call the alive_callback
            void alive_callback(const std_msgs::msg::Int8::SharedPtr msg_alive)
            {
                alive_check_new = msg_alive->data;
            }

            // When the server receives and accepts the goal, it will send a response to the client
            void goal_response_callback(const GoalHandleLanding::SharedPtr & goal_handle)
            {

                if (!goal_handle) 
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                else 
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }

            // Assuming the goal was accepted by the server, it will start processing. Any feedback to the client will be handled by the feedback_callback
            void feedback_callback(
                GoalHandleLanding::SharedPtr,
                const std::shared_ptr<const Landing::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Feedback: %.2f m", feedback->current_height);
            }

            // When the server is finished processing, it will return a result to the client. The result is handled by the result_callback
            void result_callback(const GoalHandleLanding::WrappedResult & result)
            {
                switch (result.code) 
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
                }
                rclcpp::shutdown();
            }

            // Processing data after every time period of timer
            void timer_callback()
            {   
                // If new alive check message is different from previous alive check message, action server is ready
                if(alive_check_new != alive_check_prev) 
                {
                    RCLCPP_INFO(this->get_logger(), Green_back Black_b "[INFO] Action server is connected." Reset);
                    alive_check_prev = alive_check_new;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), White_back Black_b "[INFO] Action server is not ready for %.2f secs." Reset, client_nr * timer_period);
                    
                    if(client_nr >= server_t_abort)
                    {
                        twist_msg.linear.x = 0.0;       
                        twist_msg.linear.y = 0.0;
                        twist_msg.linear.z = 0.0;
                        
                        RCLCPP_INFO(this->get_logger(), Green_b "[DATA] " Reset "Velocity: linear.x = %.2f, linear.y = %.2f, linear.z = %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z);
                        RCLCPP_INFO(this->get_logger(), Red_back Black_b "[INFO] Goal was canceled --- ACTION SERVER WAS DISCONNECTED ---" Reset);
                        
                        twist_publisher_->publish(twist_msg);

                        rclcpp::shutdown();
                        return;
                    }
                    client_nr++;
                }
                
                #if MODE
                auto Int8_msg_3 = std_msgs::msg::Int8();    // Creating an Int8 message
                Int8_msg_3.data = smallest_abort;   
                if(msg_time_actual_0 != msg_time_prev_0)    // If actual time stamp value is different from previous time stamp value, marker is detected
                {
                    counter_ND_s = 0;
                    smallest_abort = 0;
                    Int8_msg_3.data = smallest_abort; 

                    marker_detected_0 = 1;
                    msg_time_prev_0 = msg_time_actual_0;
                    RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Smallest marker detected." Reset);

                    if (goal_sent == 0 && allow_client == 1)
                    {
                        this->send_goal();
                        //RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Smallest marker detected." Reset);
                        goal_sent = 1;
                    }
                }
                else
                {
                    marker_detected_0 = 0;                  // If actual time stamp value is same as previous time stamp value, marker is not detected

                    //RCLCPP_INFO(this->get_logger(), Red_b "[INFO] counter_ND_s = %d, mark_t_abort = %f", counter_ND_s, mark_t_abort);
                    if(counter_ND_s >= mark_t_abort)     // If time of counter of non-detection of the smallest marker equals or is greater than time mark_t_abort seconds
                        smallest_abort = 1;                // Cancel the goal
                    
                    Int8_msg_3.data = smallest_abort; 
                        
                    RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Smallest marker was NOT detected for %.2f secs." Reset, counter_ND_s * timer_period);
                    counter_ND_s++;                         // Incrementing counter of non-detection of the smallest marker
                }
                #endif

                auto Int8_msg_4 = std_msgs::msg::Int8();
                Int8_msg_4.data = middle_abort;
                if(msg_time_actual_1 != msg_time_prev_1)    // If actual time stamp value is different from previous time stamp value, marker is detected
                {   
                    counter_ND_m = 0;
                    middle_abort = 0;
                    Int8_msg_4.data = middle_abort;

                    marker_detected_1 = 1;
                    msg_time_prev_1 = msg_time_actual_1;
                    RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Middle - sized marker detected." Reset);

                    if (goal_sent == 0 && allow_client == 1)
                    {
                        this->send_goal();
                        //RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Middle - sized marker detected." Reset);
                        goal_sent = 1;
                    }
                }
                else
                {
                    marker_detected_1 = 0;                  // If actual time stamp value is same as previous time stamp value, marker is not detected
                    
                    //RCLCPP_INFO(this->get_logger(), Red_b "[INFO] counter_ND_m = %d, mark_t_abort = %f", counter_ND_m, mark_t_abort);
                    if(counter_ND_m >= mark_t_abort)     // If time of counter of non-detection of the middle - sized marker equals or is greater than time mark_t_abort seconds
                        middle_abort = 1;                // Cancel the goal
                
                    Int8_msg_4.data = middle_abort; 
                        
                    RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Middle - sized marker was NOT detected for %.2f secs." Reset, counter_ND_m * timer_period);
                    counter_ND_m++;                         // Incrementing counter of non-detection of the middle - sized marker
                }

                auto Int8_msg_5 = std_msgs::msg::Int8();
                Int8_msg_5.data = biggest_abort;
                if(msg_time_actual_2 != msg_time_prev_2)    // If actual time stamp value is different from previous time stamp value, marker is detected
                {
                    counter_ND_b = 0;
                    biggest_abort = 0;
                    Int8_msg_5.data = biggest_abort;

                    marker_detected_2 = 1;
                    msg_time_prev_2 = msg_time_actual_2;
                    RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Biggest marker detected." Reset);

                    if (goal_sent == 0 && allow_client == 1)
                    {
                        this->send_goal();
                        //RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Biggest marker detected." Reset);
                        goal_sent = 1;
                    }
                }
                else
                {
                    marker_detected_2 = 0;                  // If actual time stamp value is same as previous time stamp value, marker is not detected

                    //RCLCPP_INFO(this->get_logger(), Red_b "[INFO] counter_ND_b = %d, mark_t_abort = %f", counter_ND_b, mark_t_abort);
                    if(counter_ND_b >= mark_t_abort)     // If time of counter of non-detection of the biggest marker equals or is greater than time mark_t_abort seconds
                        biggest_abort = 1;                // Cancel the goal
                
                    Int8_msg_5.data = biggest_abort; 
                        
                    RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Biggest marker was NOT detected for %.2f secs." Reset, counter_ND_b * timer_period);
                    counter_ND_b++;                         // Incrementing counter of non-detection of the biggest marker
                }

                #if MODE
                    if(counter_ND_b > mark_t_abort && counter_ND_m > mark_t_abort && counter_ND_s > mark_t_abort)
                    {
                        RCLCPP_INFO(this->get_logger(), Red_back Black_b "[INFO] --- Cancelling the goal because of non - detection of the markers ---" Reset);
                        rclcpp::shutdown();
                        return;
                    }
                #else
                    if(counter_ND_b > mark_t_abort && counter_ND_m > mark_t_abort)
                    {
                        RCLCPP_INFO(this->get_logger(), Red_back Black_b "[INFO] --- Cancelling the goal because of non - detection of the markers ---" Reset);
                        rclcpp::shutdown();
                        return;
                    }
                #endif

                // Create an Int8 message
                auto Int8_msg_0 = std_msgs::msg::Int8();
                auto Int8_msg_1 = std_msgs::msg::Int8();
                auto Int8_msg_2 = std_msgs::msg::Int8();
                
                // Fill and publish an Int8 message
                Int8_msg_0.data = marker_detected_0; 
                int_publisher_0->publish(Int8_msg_0);

                // Fill and publish an Int8 message
                Int8_msg_1.data = marker_detected_1; 
                int_publisher_1->publish(Int8_msg_1);

                // Fill and publish an Int8 message
                Int8_msg_2.data = marker_detected_2; 
                int_publisher_2->publish(Int8_msg_2);
                
                // Publish an Int8 message of aborling the goal because of non - detection of the markers
                #if MODE
                    int_publisher_3->publish(Int8_msg_3);
                #endif
                int_publisher_4->publish(Int8_msg_4);
                int_publisher_5->publish(Int8_msg_5);
            }

            // Receiving new data
            void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                if(msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_0")         // If smallest marker was detected
                {
                    msg_time_actual_0 = msg->header.stamp.nanosec;                              // Save actual time
                }
                else if(msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_1")    // If middle - sized marker was detected
                {
                    msg_time_actual_1 = msg->header.stamp.nanosec;                              // Save actual time
                }
                else if(msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_2")    // If biggest marker was detected
                {
                    msg_time_actual_2 = msg->header.stamp.nanosec;                              // Save actual time
                }
            }

            float msg_time_prev_0;      // Variable with previous time stamp
            float msg_time_actual_0;    // Variable with actual time stamp
            float marker_detected_0;    // Output from detector

            float msg_time_prev_1;      // Variable with previous time stamp
            float msg_time_actual_1;    // Variable with actual time stampabort
            float marker_detected_1;    // Output from detector

            float msg_time_prev_2;      // Variable with previous time stamp
            float msg_time_actual_2;    // Variable with actual time stamp
            float marker_detected_2;    // Output from detector

            // Member variables for times, subscribers and publishers
            int goal_sent;
            float mark_t_abort;
            float server_t_abort;      // Threshold time in seconds after which the action server is not ready (converted to timer repeats)
            int counter_ND_s;           // Counter of non - detection of the smallest marker
            int counter_ND_m;           // Counter of non - detection of the middle-sized marker
            int counter_ND_b;           // Counter of non - detection of the biggest marker
            int smallest_abort;        // Cancelled goal becasue of non - detection of the smallest marker
            int middle_abort;          // Cancelled goal becasue of non - detection of the middle-sized marker
            int biggest_abort;         // Cancelled goal becasue of non - detection of the biggest marker
            int allow_client;          // Variable for allowing the client to function
            int client_nr;             // Number of client tries
            float timer_period;       // Timer period in seconds
            int alive_check_new;          // New alive check message
            int alive_check_prev;         // Previous alive check message
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_0;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_1;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_2;
            rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr alive_subscriber;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_0;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_1;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_2;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_3;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_4;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_5;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
            geometry_msgs::msg::Twist twist_msg;
    };
}   // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(uav_landing_cpp::LandingActionClient)