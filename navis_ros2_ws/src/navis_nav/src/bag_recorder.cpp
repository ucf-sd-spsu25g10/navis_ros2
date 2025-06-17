#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <ctime>
#include <sstream>
#include <filesystem>

class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder()
  : Node("bag_recorder")
  {

    std::time_t now = std::time(nullptr);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    
    // Combine with base path
    std::string base_dir = "./src/navis_nav/bags/";
    std::string bag_path = base_dir + "imu_bag_" + timestamp;

    // Declare the parameter with a default value of true
    this->declare_parameter<bool>("record_imu", true);
    bool record_imu = this->get_parameter("record_imu").as_bool();

    if (record_imu) {
      imu_writer_ = std::make_unique<rosbag2_cpp::Writer>();
      imu_writer_->open(bag_path);

      auto imu_sub_callback_lambda = [this](std::shared_ptr<rclcpp::SerializedMessage> msg){
        rclcpp::Time time_stamp = this->now();
        imu_writer_->write(msg, "imu/data_raw", "sensor_msgs/msg/Imu", time_stamp);
      };

      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10, imu_sub_callback_lambda);

      RCLCPP_INFO(this->get_logger(), "IMU recording enabled.");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> imu_writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BagRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
