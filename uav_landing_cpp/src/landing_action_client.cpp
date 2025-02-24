#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

                // Create a timer
                timer_ = this->create_wall_timer(
                    500ms, std::bind(&LandingActionClient::timer_callback, this));

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

                RCLCPP_INFO(this->get_logger(), Cyan_b "[INFO] Marker detector action client has started." Reset); // Informing about starting a node
            }

            void send_goal()
            {
                using namespace std::placeholders;

                if (!this->client_ptr_->wait_for_action_server()) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    rclcpp::shutdown();
                }

                auto goal_msg = Landing::Goal();
                goal_msg.target_height = 1.4;

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
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
                }
                rclcpp::shutdown();
            }

            // Processing data after every time period of timer
            void timer_callback()
            {
                if(msg_time_actual_0 != msg_time_prev_0)    // If actual time stamp value is different from previous time stamp value, marker is detected
                {
                    marker_detected_0 = 1;
                    msg_time_prev_0 = msg_time_actual_0;
                    RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Smallest marker detected." Reset);

                    if (goal_sent == 0)
                    {
                        this->send_goal();
                        RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Smallest marker detected, SENDING GOAL." Reset);
                        goal_sent = 1;
                    }
                }
                else
                {
                    marker_detected_0 = 0;                  // If actual time stamp value is same as previous time stamp value, marker is not detected
                    RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Smallest marker was NOT detected." Reset);
                }

                if(msg_time_actual_1 != msg_time_prev_1)    // If actual time stamp value is different from previous time stamp value, marker is detected
                {
                    marker_detected_1 = 1;
                    msg_time_prev_1 = msg_time_actual_1;
                    RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Middle - sized marker detected." Reset);

                    if (goal_sent == 0)
                    {
                        this->send_goal();
                        RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Middle - sized marker detected, SENDING GOAL." Reset);
                        goal_sent = 1;
                    }
                }
                else
                {
                    marker_detected_1 = 0;                  // If actual time stamp value is same as previous time stamp value, marker is not detected
                    RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Middle - sized marker was NOT detected." Reset);
                }

                if(msg_time_actual_2 != msg_time_prev_2)    // If actual time stamp value is different from previous time stamp value, marker is detected
                {
                    marker_detected_2 = 1;
                    msg_time_prev_2 = msg_time_actual_2;
                    RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Biggest marker detected." Reset);

                    if (goal_sent == 0)
                    {
                        this->send_goal();
                        RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Biggest marker detected, SENDING GOAL." Reset);
                        goal_sent = 1;
                    }
                }
                else
                {
                    marker_detected_2 = 0;                  // If actual time stamp value is same as previous time stamp value, marker is not detected
                    RCLCPP_INFO(this->get_logger(), Red_b "[INFO] Biggest marker was NOT detected." Reset);
                }

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

            double msg_time_prev_0;      // Variable with previous time stamp
            double msg_time_actual_0;    // Variable with actual time stamp
            double marker_detected_0;    // Output from detector

            double msg_time_prev_1;      // Variable with previous time stamp
            double msg_time_actual_1;    // Variable with actual time stamp
            double marker_detected_1;    // Output from detector

            double msg_time_prev_2;      // Variable with previous time stamp
            double msg_time_actual_2;    // Variable with actual time stamp
            double marker_detected_2;    // Output from detector

            // Member variables for times, subscribers and publishers
            int goal_sent;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_0;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_1;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_2;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_0;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_1;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_2;
    };
}   // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(uav_landing_cpp::LandingActionClient)