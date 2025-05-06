#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoPublisher : public rclcpp::Node {
public:
    CameraInfoPublisher() : Node("camera_info_publisher") 
    {
        // Publisher
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/cameras/left_hand_camera/camera_info", 10);

        // Set camera parameters
        camera_info_msg_.header.frame_id = "general_camera";
        //camera_info_msg_.width = 752;
        //camera_info_msg_.height = 480;
        camera_info_msg_.width = 720;
        camera_info_msg_.height = 540;

        // Intrinsic matrix (K)
        camera_info_msg_.k = 
        {
            440.56, 0.0, 376.5,  // fx,  0, cx
            0.0, 440.56, 240.5,  //  0, fy, cy
            0.0, 0.0, 1.0        //  0,  0,  1
            //364.45678, 0.0, 368.73125,  // fx,  0, cx
            //0.0, 364.04505, 288.08164,  //  0, fy, cy
            //0.0, 0.0, 1.0        //  0,  0,  1
        };

        // Distortion coefficients (D)
        camera_info_msg_.d = {-0.0, 0.0, 0.0, -0.0, 0.0};
        //camera_info_msg_.d = {-0.250686, 0.045401, 0.000187, -0.000215, 0.000000};

        // Projection matrix (P)
        camera_info_msg_.p = 
        {
            //249.52399, 0.0, 365.94672, 0.0,  // fx,  0, cx, Tx
            //0.0, 299.30328, 297.05873, 0.0,  //  0, fy, cy, Ty
            //0.0, 0.0, 1.0, 0.0        //  0,  0,  1,  0
            20.52399, 0.0, 30.94672, 0.0,  // fx,  0, cx, Tx
            0.0, 20.30328, 20.05873, 0.0,  //  0, fy, cy, Ty
            0.0, 0.0, 1.0, 0.0        //  0,  0,  1,  0
        };

        // Rectification matrix (R) - Identity for no rectification
        camera_info_msg_.r = 
        {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        // Set distortion model
        camera_info_msg_.distortion_model = "plumb_bob";

        camera_info_msg_.binning_x = 0;
        camera_info_msg_.binning_y = 0;

        //camera_info_msg_.roi.width = 752;
        //camera_info_msg_.roi.height = 480;
        camera_info_msg_.roi.width = 720;
        camera_info_msg_.roi.height = 540;

        // Wait for 3 seconds before publishing
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Publish once
        publishCameraInfo();
    }

private:
    void publishCameraInfo() 
    {
        camera_info_msg_.header.stamp = this->get_clock()->now();
        camera_info_pub_->publish(camera_info_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}
