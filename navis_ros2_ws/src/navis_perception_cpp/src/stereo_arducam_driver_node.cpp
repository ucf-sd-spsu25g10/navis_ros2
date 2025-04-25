#include "navis_perception/stereo_arducam_driver_node.cpp"

StereoArducamDriver::StereoArducamDriver() : Node('stereo_arducam_driver_node')
{
    left_image_raw_pub_ = create_publisher<sensor_msgs::msg::image>("/stereo/left/image_raw", 1);
    left_cam_info_pub_ = create_publisher<sensor_msgs::msg::camera_info>("/stereo/left/camera_info", 1);

    right_image_raw_pub_ = create_publisher<sensor_msgs::msg::image>("/stereo/right/image_raw", 1);
    right_cam_info_pub_ = create_publisher<sensor_msgs::msg::camera_info>("/stereo/right/camera_info", 1);

    
}

void StereoArducamDriver::cameraSetup()
{
    return;
}

void StereoArducamDriver::imageCapture()
{
    return;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto driver = std::make_shared<StereoArducamDriver>();
    rclcpp::spin(driver);
    rclcpp::shutdown();
    return 0;
  }