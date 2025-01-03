/*
&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
Note: Uprava kodu bude sucastou diplomovej prace
&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class regulator_node : public rclcpp::Node 
{
    public:
        regulator_node() : Node("regulator_node") 
        {
            // Create a subscribeer
            pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single_1/pose_1", 10,
                std::bind(&regulator_node::poseCallback, this, std::placeholders::_1)
            );

            // Create a publisher
            twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/x500_1/aircraft/cmd_vel", 10
            );

            // Set a proportional gain
            Kp = 0.5;

            RCLCPP_INFO(this->get_logger(), "Regulator node has started.");
        }

    private:
        // Processing the pose messages
        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
        {
            // Load x and y positions from the PoseStamped message
            double x = msg->pose.position.x;
            double y = msg->pose.position.y;

            // P regulator
            double velocity_x = -Kp * y;	// TO EDIT ... Uprava kodu bude sucastou diplomovej prace
            double velocity_y = -Kp * x;

            // Create a Twist message
            auto twist_msg = geometry_msgs::msg::Twist();

            // Velocity saturation limit
            double vel_saturation = 2.0;
            double vel_default = 0.05;  // Velocity default

            // Apply saturation for velocity_x
            velocity_x = (velocity_x > vel_saturation) ? vel_default : velocity_x;
            velocity_x  = (velocity_x < -vel_saturation) ? -vel_default : velocity_x;

            // Apply saturation for velocity_y
            velocity_y = (velocity_y > vel_saturation) ? vel_default : velocity_y;
            velocity_y = (velocity_y < -vel_saturation) ? -vel_default : velocity_y;

            // Fill and publish a Twist message
            twist_msg.linear.x = velocity_x; 
            twist_msg.linear.y = velocity_y;
            double z_landing_vel = -0.15;		// Landing speed
            double z_landing_vel_stop = 0.0; 	// Landing speed
            double landing_high_limit = 1.0;	// Stop decreasing when this level is reached
            twist_msg.linear.z = (msg->pose.position.z > landing_high_limit) ? z_landing_vel : z_landing_vel_stop;	// No vertical movement
            
            // No angular movement
            twist_msg.angular.x = 0.0;		
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;

            twist_publisher_->publish(twist_msg);

            // Log what was published
            RCLCPP_INFO(this->get_logger(), "Input Pose: x=%.2f, y=%.2f -> Velocity: linear.x=%.2f, linear.y=%.2f, linear.z=%.2f",
                        x, y, velocity_x, velocity_y, twist_msg.linear.z);
        }

        // Member variables for proportional gain, subscriber and publisher 
        double Kp;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<regulator_node>();     // Better reading
    rclcpp::spin(node);                                 // Better reading, spin(node);  
    rclcpp::shutdown();
    return 0;
}