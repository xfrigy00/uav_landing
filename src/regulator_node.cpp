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
            // Create a subscriber
            pose_subscriber_0 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10,
                std::bind(&regulator_node::poseCallback, this, std::placeholders::_1)
            );

            pose_subscriber_1 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single_1/pose_1", 10,
                std::bind(&regulator_node::poseCallback, this, std::placeholders::_1)
            );

            pose_subscriber_2 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single_2/pose_2", 10,
                std::bind(&regulator_node::poseCallback, this, std::placeholders::_1)
            );

            // Create a publisher
            twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/x500_1/aircraft/cmd_vel", 10
            );

            // Set a proportional gain
            Kp = 0.5;

            RCLCPP_INFO(this->get_logger(), "Regulator node has started."); // Informing about starting a node
        }

    private:
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
            double z = msg->pose.position.z;

            // Initialize x and y
            double x = 0;
            double y = 0;

            // Marker sizes [m]
            double big_marker_s = 0.78;
            double middle_marker_s = 0.58;
            double small_marker_s = 0.15;

            /*
            Marker layout - all markers are squares
            An image ↓↓↓ is for reference only

            ############
            ############&&&
            ############&&&
            ############@@@@@@
            ############@@@@@@
            ############@@@@@@
            ############@@@@@@

            # - Biggest marker
            @ - Middle-sized marker
            & - Smallest marker
            */

            double horizontal_dev_x = 0;        // horizontal_deviation [m] - Horizontal deviation from center of marker in axe x
            double horizontal_dev_y = 0;        // horizontal_deviation [m] - Horizontal deviation from center of marker in axe y
            int level_2 = 3;        // 3 m height 
            double level_1 = 1.5;   // 1.5 m height

            // Determining desired distance from marker in axes x and y
            if(z > level_2)                         // Detecting the biggest marker but descending to middle-sized marker
            {
                horizontal_dev_x = big_marker_s / 2 + middle_marker_s / 2;
                horizontal_dev_y = big_marker_s / 2 - middle_marker_s / 2;
            }
            else if (z <= level_2 && z >= level_1)  // Detecting middle-sized marker but descending smallest
            {
                horizontal_dev_x = - middle_marker_s / 2 + small_marker_s / 2;
                horizontal_dev_y = - middle_marker_s / 2 - small_marker_s / 2;
            }
            else                                    // Detecting smallest marker and descending to smallest marker
            {
                horizontal_dev_x = 0;
                horizontal_dev_y = 0;
            }

            // Decide which state is active
            // Camera coordinates: x: →, y: ↓
            // If more than 3 m, between 3 and 1.5 m or below 1.5 m
            // Load x and y positions from the PoseStamped message
            if((z > level_2 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_2") || (z <= level_2 && z >= level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_1") || (z < level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_0"))
            {
                x = msg->pose.position.x + horizontal_dev_x;
                y = msg->pose.position.y + horizontal_dev_y;
            }

            // P regulator
            // World coordinates: x: ↑, y: ←
            double velocity_x = Kp * y;     // Velocity in world x coordinate
            double velocity_y = Kp * x;     // Velocity in world y coordinate
            velocity_x = velocity_x * (-1); // X of a world coordinate system and y of an image coordinate system are oppositely oriented
            velocity_y = velocity_y * (-1); // Y of a world coordinate system and x of an image coordinate system are oppositely oriented

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
            double landing_high_limit = 0.3;	// Stop decreasing when this level is reached [m]
            double horizontal_thrshld = 0.03;   // horizontal_threshold [m], maximum desired regulation deviation while reaching desired point

            // If height in z is more than z_landing_vel_stop height &&
            // If x or y are in interval (horizontal_thrshld; - horizontal_thrshld), then vertical movement is allowed
            double no_inp_bord = 0.05;          // Border for interval without input [m]
            if ((z < level_2 + no_inp_bord && z > level_2 - no_inp_bord) || (z < level_1 + no_inp_bord && z > level_1 - no_inp_bord))   // Avoiding two inputs and just descending without input
                twist_msg.linear.z = z_landing_vel;
            else
                twist_msg.linear.z = (msg->pose.position.z > landing_high_limit && x < horizontal_thrshld && x > - horizontal_thrshld && y < horizontal_thrshld && y > - horizontal_thrshld) ? z_landing_vel : z_landing_vel_stop;
            
            // No angular movement
            twist_msg.angular.x = 0.0;		
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;

            // Publish only data from used frame
            if((z > level_2 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_2") || (z <= level_2 && z >= level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_1") || (z < level_1 && msg->header.frame_id == "stereo_gazebo_left_camera_optical_frame_0"))
            {
                twist_publisher_->publish(twist_msg);

                // Log what was published
                RCLCPP_INFO(this->get_logger(), "Input Pose: x=%.2f, y=%.2f -> Velocity: linear.x=%.2f, linear.y=%.2f, linear.z=%.2f",
                            x, y, velocity_x, velocity_y, twist_msg.linear.z);
            }
        }

        // Member variables for proportional gain, subscriber and publisher 
        double Kp;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_0;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_1;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_2;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<regulator_node>();     // For better reading
    rclcpp::spin(node);                                 // Better reading of spin(node) than spin(std::make_shared<regulator_node>())
    rclcpp::shutdown();
    return 0;
}