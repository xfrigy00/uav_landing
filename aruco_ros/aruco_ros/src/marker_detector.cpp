#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>

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

// Creating a node
class marker_detector_node : public rclcpp::Node 
{
    public:
        marker_detector_node() : Node("marker_detector_node") 
        {
            // Create ubscribers
            pose_sub_0 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10,
                std::bind(&marker_detector_node::poseCallback, this, std::placeholders::_1)
            );

            pose_sub_1 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single_1/pose_1", 10,
                std::bind(&marker_detector_node::poseCallback, this, std::placeholders::_1)
            );

            pose_sub_2 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single_2/pose_2", 10,
                std::bind(&marker_detector_node::poseCallback, this, std::placeholders::_1)
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
                250ms, std::bind(&marker_detector_node::timer_callback, this));

            msg_time_prev_0 = -1;        // Variable with previous time stamp
            msg_time_actual_0 = -1;      // Variable with actual time stamp
            marker_detected_0 = 0;       // Output_0 from detector

            msg_time_prev_1 = -1;        // Variable with previous time stamp
            msg_time_actual_1 = -1;      // Variable with actual time stamp
            marker_detected_1 = 0;       // Output_1 from detector

            msg_time_prev_2 = -1;        // Variable with previous time stamp
            msg_time_actual_2 = -1;      // Variable with actual time stamp
            marker_detected_2 = 0;       // Output_2 from detector

            RCLCPP_INFO(this->get_logger(), Cyan_b "[INFO] Marker_detector_node node has started." Reset); // Informing about starting a node
        }

    private:
        // Processing data after every time period of timer
        void timer_callback()
        {
            if(msg_time_actual_0 != msg_time_prev_0)    // If actual time stamp value is different from previous time stamp value, marker is detected
            {
                marker_detected_0 = 1;
                msg_time_prev_0 = msg_time_actual_0;
                RCLCPP_INFO(this->get_logger(), Green_b "[INFO] Smallest marker detected." Reset);
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
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_0;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_1;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_2;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_0;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_1;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher_2;
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<marker_detector_node>();   // For better reading
    rclcpp::spin(node);                                     // Better reading of spin(node) than spin(std::make_shared<regulator_node>())
    rclcpp::shutdown();
    return 0;
}