#include <rclcpp/rclcpp.hpp>
#include <aruco_msgs/msg/marker_array.hpp>
#include <aruco_msgs/msg/marker.hpp>

class MarkerSubscriber : public rclcpp::Node
{
public:
    MarkerSubscriber()
        : Node("marker_subscriber")
    {
        // Create a subscription to the /marker_publisher/markers topic
        subscription_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/marker_publisher/markers", 10, std::bind(&MarkerSubscriber::markerCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MarkerSubscriber node started, listening to /marker_publisher/markers");
    }

private:
    // Callback function to process incoming MarkerArray messages
    void markerCallback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(), 
            "Received %zu markers", msg->markers.size());

        for (const auto &marker : msg->markers)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Marker ID: %d\n"
                "  Position: [%.2f, %.2f, %.2f]\n"
                "  Orientation: [%.2f, %.2f, %.2f, %.2f]\n"
                "  Confidence: %.2f",
                marker.id,
                marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z,
                marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w,
                marker.confidence);
        }
    }

    // ROS 2 subscription object
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create and spin the MarkerSubscriber node
    rclcpp::spin(std::make_shared<MarkerSubscriber>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}

