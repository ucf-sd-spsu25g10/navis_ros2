
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs//msg/camera_info"

using namespace std::chrono_literals;

class StereoArducamDriver : public rclcpp:: Node
{
public:
    explicit StereoArducamDriver();

private:
    void cameraSetup();
    void imageCapture();

    // /stereo/left/image_raw (sensor_msgs/Image)
    // /stereo/left/camera_info (sensor_msgs/CameraInfo)
    rclcpp::Publisher<sensor_msgs::msg::image>::SharedPtr left_image_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::camera_info>::SharedPtr left_cam_info_pub_;

    // /stereo/right/image_raw (sensor_msgs/Image)
    // /stereo/right/camera_info (sensor_msgs/CameraInfo)
    rclcpp::Publisher<sensor_msgs::msg::image>::SharedPtr right_image_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::camera_info>::SharedPtr right_cam_info_pub_;

}