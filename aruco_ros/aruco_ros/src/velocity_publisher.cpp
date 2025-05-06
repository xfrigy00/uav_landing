#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher() : Node("velocity_publisher")
    {
        // Declare and get parameters
        this->declare_parameter("linear_x", 0.0);
        this->declare_parameter("linear_y", 0.0);
        this->declare_parameter("linear_z", 0.0);

        double linear_x, linear_y, linear_z;
        this->get_parameter("linear_x", linear_x);
        this->get_parameter("linear_y", linear_y);
        this->get_parameter("linear_z", linear_z);

        // Create a publisher for the Twist message
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/x500_1/aircraft/cmd_vel", 10);

        // Initialize the Twist message
        twist_msg_.linear.x = linear_x;
        twist_msg_.linear.y = linear_y;
        twist_msg_.linear.z = linear_z;
        twist_msg_.angular.x = 0.0;
        twist_msg_.angular.y = 0.0;
        twist_msg_.angular.z = 0.0;

        // Set a timer to publish the message
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/5), 
            std::bind(&VelocityPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing Twist: [%.2f, %.2f, %.2f]",
                    twist_msg_.linear.x, twist_msg_.linear.y, twist_msg_.linear.z);
        publisher_->publish(twist_msg_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist twist_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}

