#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Text
#define Green_b "\033[1;32m"     // Green bold
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

class regulator_node : public rclcpp::Node 
{
    public:
        regulator_node() : Node("regulator_node") 
        {
            // Create subscribers
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

            // Set initialization for variables used for slow speeding up
            indicator_vel_x = 0;    // indicator_velocity_x == 0 - slow speeding up is needed || 1 - slow speeding up is in progress || 2 - slow speeding up is done
            indicator_vel_y = 0;
            desired_vel_x = 0;      // Desired velocity
            desired_vel_y = 0;
            actual_vel_x = 0;       // Actual velocity  
            actual_vel_y = 0;

            // Setting variable for indicating end of landing
            landing_ind = 2;    /*  2 - landig was started but position of the drone is less than landing_high_limit, 
                                    0 - FALSE,landing was started but position of the drone is more than landing_high_limit, 
                                    1 - TRUE, landing was started previous state was 0 and now position of the drone is more than landing_high_limit*/

            RCLCPP_INFO(this->get_logger(), Cyan_b "[INFO] Regulator node has started." Reset); // Informing about starting a node
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

            // Marker positions [m]
            double big_marker_pos_x = big_marker_s / 2;
            double big_marker_pos_y = big_marker_s / 2;
            double middle_marker_pos_x = big_marker_s + middle_marker_s / 2;
            double middle_marker_pos_y = big_marker_s - middle_marker_s / 2;
            double small_marker_pos_x = big_marker_s + small_marker_s / 2;
            double small_marker_pos_y = big_marker_s - middle_marker_s - small_marker_s / 2;

            // From sizes of landing markers, the landing point is (big_marker_s + middle_marker_s) / 2 [m] for x axe and big_marker_s / 2 [m] for y axe
            double middle_x = (big_marker_s + middle_marker_s) / 2;
            double middle_y = big_marker_s / 2;

            /*
            Marker layout - all markers are squares
            An image ↓↓↓ is for reference only

            000000000000
            000000000000222
            000000000000222
            00000000X00011111
            00000000000011111
            00000000000011111
            00000000000011111

            X - Landing point - middle of the object of markers
            0 - Biggest marker
            1 - Middle-sized marker
            2 - Smallest marker
            */

            double horizontal_dev_x = 0;            // horizontal_deviation [m] - Horizontal deviation from center of marker in axe x
            double horizontal_dev_y = 0;            // horizontal_deviation [m] - Horizontal deviation from center of marker in axe y
            double level_2 = 3;                     // 3 m height for landing
            double level_1 = 1.5;                   // 1.5 m height for landing

            // Determining desired distance from marker in axes x and y
            // Because of velocity_x = velocity_x * (-1); below also mutiplying "* (-1)" needs to be here
            if(z > level_2)                         // Detecting the biggest marker but descending to middle-sized marker
            {
                horizontal_dev_x = (big_marker_pos_x - middle_x) * (-1);
                horizontal_dev_y = (big_marker_pos_y - middle_y) * (-1);
            }
            else if (z <= level_2 && z >= level_1)  // Detecting middle-sized marker but descending smallest
            {
                horizontal_dev_x = (middle_marker_pos_x - middle_x) * (-1);
                horizontal_dev_y = (middle_marker_pos_y - middle_y) * (-1);
            }
            else                                    // Detecting smallest marker and descending to smallest marker
            {
                horizontal_dev_x = (small_marker_pos_x - middle_x) * (-1);
                horizontal_dev_y = (small_marker_pos_y - middle_y) * (-1);
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
            double velocity_x = Kp * y;                 // Velocity in world x coordinate
            double velocity_y = Kp * x;                 // Velocity in world y coordinate
            velocity_x = velocity_x * (-1);             // X of a world coordinate system and y of an image coordinate system are oppositely oriented
            velocity_y = velocity_y * (-1);             // Y of a world coordinate system and x of an image coordinate system are oppositely oriented

            // Velocity saturation limit
            double vel_saturation = 1.0;                // Velocity saturation [m]

            // Slow speeding up after start of the regulator_node
            double speed_inc_dec = 0.002;               // speed adding by speed_inc_dec m / s
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
            }
            
            if(indicator_vel_x == 1)                    // indicator_vel_x == 1 - slow speeding up is in progress
            {
                // Increasing or decreasing actual velocity based on desired velocity
                if(desired_vel_x != actual_vel_x)
                {
                    actual_vel_x = (desired_vel_x > actual_vel_x) ? actual_vel_x + speed_inc_dec : actual_vel_x - speed_inc_dec;
                    velocity_x = actual_vel_x;
                }

                // Checking if desired velocity was reached
                if((desired_vel_x >= 0 && velocity_x >= desired_vel_x) || (desired_vel_x <= 0 && velocity_x <= desired_vel_x))
                {
                    indicator_vel_x = 2;                // indicator_vel_x == 2 - slow speeding up is done

                    for(int i = 0; i < 5; i++)          // Informing when ramp in x axe is done
                        RCLCPP_INFO(this->get_logger(), Black_back White_b "[INFO] Ramp y DONE." Reset);
                }
            }

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
            }
            
            if(indicator_vel_y == 1)                    // indicator_vel_y == 1 - slow speeding up is in progress
            {
                // Increasing or decreasing actual velocity based on desired velocity
                if(desired_vel_y != actual_vel_y)
                {
                    actual_vel_y = (desired_vel_y > actual_vel_y) ? actual_vel_y + speed_inc_dec : actual_vel_y - speed_inc_dec;
                    velocity_y = actual_vel_y;
                }

                // Checking if desired velocity was reached
                if((desired_vel_y >= 0 && velocity_y >= desired_vel_y) || (desired_vel_y <= 0 && velocity_y <= desired_vel_y))
                {
                    indicator_vel_y = 2;                // indicator_vel_y == 2 - slow speeding up is done

                    for(int i = 0; i < 5; i++)         // Informing when ramp in y axe is done
                        RCLCPP_INFO(this->get_logger(), White_back Black_b "[INFO] Ramp y DONE." Reset);
                }
            }

            // Create a Twist message
            auto twist_msg = geometry_msgs::msg::Twist();

            // Apply saturation for velocity_x
            velocity_x = (velocity_x > vel_saturation) ? vel_saturation : velocity_x;
            velocity_x  = (velocity_x < -vel_saturation) ? -vel_saturation : velocity_x;

            // Apply saturation for velocity_y
            velocity_y = (velocity_y > vel_saturation) ? vel_saturation : velocity_y;
            velocity_y = (velocity_y < -vel_saturation) ? -vel_saturation : velocity_y;

            // Fill and publish a Twist message
            twist_msg.linear.x = velocity_x; 
            twist_msg.linear.y = velocity_y;
            double z_landing_vel = -0.15;		// Landing speed
            double z_landing_vel_stop = 0.0; 	// Landing speed
            double landing_high_limit = 1;	    // Stop decreasing when this level is reached [m]
            double horizontal_thrshld = 0.03;   // horizontal_threshold [m], maximum desired regulation deviation while reaching desired point
            double vertical_error = 0.05;       // vertical error of detector for z axe [m]

            // If height in z is more than z_landing_vel_stop height &&
            // If x or y are in interval (horizontal_thrshld; - horizontal_thrshld), then vertical movement is allowed
            double no_inp_bord = 0.05;          // Border for interval without input [m]
            if ((z < level_2 + no_inp_bord && z > level_2 - no_inp_bord) || (z < level_1 + no_inp_bord && z > level_1 - no_inp_bord))   // Avoiding two inputs and just descending without input
                twist_msg.linear.z = z_landing_vel;
            else
                twist_msg.linear.z = (msg->pose.position.z > landing_high_limit && x < horizontal_thrshld && x > - horizontal_thrshld && y < horizontal_thrshld && y > - horizontal_thrshld) ? z_landing_vel : z_landing_vel_stop;
            
            if(msg->pose.position.z >= landing_high_limit + vertical_error) 
                landing_ind = 0;                                                        // Position of the drone is more than landing_high_limit, changing state of landing
            else if(landing_ind == 0 && msg->pose.position.z <= landing_high_limit)     // If position of the drone is in axe z less of same as Landing and value of landing_ind is 0, automatic landing is done
                landing_ind = 1;

            if(landing_ind == 1)
                RCLCPP_INFO(this->get_logger(), Yellow_b_i "[INFO] Automatic landing DONE, regulation to minimum regulation deviation in axes x and y is still working." Reset);

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
                            x, y, velocity_x, velocity_y, twist_msg.linear.z);
            }
        }

        // Member variables for proportional gain, slow speeding up, subscribers and publisher 
        double Kp;
        double indicator_vel_x;
        double indicator_vel_y;
        double actual_vel_x;
        double actual_vel_y;
        double desired_vel_x;
        double desired_vel_y;
        int landing_ind;
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
