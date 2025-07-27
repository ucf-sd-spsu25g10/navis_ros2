#include <rclcpp/rclcpp.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <limits>
#include <vector>
#include <algorithm>

#include "navis_msgs/msg/control_out.hpp"

class ClosestObstacleDetector : public rclcpp::Node
{
public:
  ClosestObstacleDetector()
  : Node("obstacle_detector")
  {
    this->declare_parameter("baseline", 0.152);
    this->declare_parameter("focal_length_px", 479.14);
    this->declare_parameter("roi_width_fraction", 0.2);
    this->declare_parameter("roi_height_fraction", 0.3);

    baseline_ = this->get_parameter("baseline").as_double();
    focal_length_px_ = this->get_parameter("focal_length_px").as_double();
    roi_width_fraction_ = this->get_parameter("roi_width_fraction").as_double();
    roi_height_fraction_ = this->get_parameter("roi_height_fraction").as_double();

    sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
      "/disparity", rclcpp::SensorDataQoS(),
      std::bind(&ClosestObstacleDetector::disparityCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<navis_msgs::msg::ControlOut>(
      "/control_output", 10);
      
    RCLCPP_INFO(this->get_logger(), "ClosestObstacleDetector node started.");
  }

private:
  void disparityCallback(const stereo_msgs::msg::DisparityImage::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg->image, msg->image.encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat disparity = cv_ptr->image;

    if (disparity.type() != CV_32F)
    {
      disparity.convertTo(disparity, CV_32F);
    }

    if (cv::mean(disparity)[0] > 1000.0)
    {
      disparity = disparity / 16.0;
    }

    int w = disparity.cols;
    int h = disparity.rows;
    int roi_w = static_cast<int>(w * roi_width_fraction_);
    int roi_h = static_cast<int>(h * roi_height_fraction_);
    int x0 = (w - roi_w) / 2;
    int y0 = 0; // Start ROI from the top of the image
    cv::Rect roi_rect(x0, y0, roi_w, roi_h);

    if (x0 < 0 || y0 < 0 || roi_w <= 0 || roi_h <= 0 || (x0 + roi_w) > w || (y0 + roi_h) > h)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid ROI parameters. Skipping this frame.");
      return;
    }

    cv::Mat roi = disparity(roi_rect).clone();
    cv::medianBlur(roi, roi, 5);

    int hist_bins = 256;
    float hist_range_params[] = {0.0f, 256.0f};
    const float* hist_range = {hist_range_params};
    cv::Mat mask;
    cv::inRange(roi, cv::Scalar(0.1), cv::Scalar(256.0), mask);

    size_t num_valid_pixels = cv::countNonZero(mask);
    size_t min_valid_pixels = (roi.rows * roi.cols) * 0.05;
    if (num_valid_pixels < min_valid_pixels)
    {
      RCLCPP_WARN(this->get_logger(), "Not enough valid pixels in ROI: %zu (minimum required: %zu)", num_valid_pixels, min_valid_pixels);
      return;
    }

    cv::Mat hist;
    cv::calcHist(&roi, 1, 0, mask, hist, 1, &hist_bins, &hist_range);

    float significant_disparity = 0.0f;
    // A peak is significant if it contains at least 10% of the valid pixels,
    // AND at least 1% of the total pixels in the ROI. This provides a stable
    // minimum threshold, preventing false positives from noise in empty scenes.
    const int min_relative_peak_size = num_valid_pixels * 0.10;
    const int min_absolute_peak_size = (roi.rows * roi.cols) * 0.01;
    const int min_pixels_for_peak = std::max(min_relative_peak_size, min_absolute_peak_size);

    for (int i = hist_bins - 1; i >= 0; i--)
    {
        if (hist.at<float>(i) > min_pixels_for_peak)
        {
            float bin_width = (hist_range[1] - hist_range[0]) / hist_bins;
            significant_disparity = hist_range[0] + (i * bin_width) + (bin_width / 2.0f);
            break; // Found the closest, significant object
        }
    }

    if (significant_disparity < 0.1f) {
        RCLCPP_INFO(this->get_logger(), "No significant obstacle found.");
        if (obstacle_flag) {
            obstacle_flag = false;
            RCLCPP_INFO(this->get_logger(), "Obstacle flag reset.");
        }
        return;
    }

    float depth_m = (focal_length_px_ * baseline_) / significant_disparity;

    RCLCPP_INFO(this->get_logger(), "Closest distance: %.2f m (disparity: %.2f px)", depth_m, significant_disparity);

    if (obstacle_flag) {
      if (depth_m > 2.0f && counter_ >= 5) {
        RCLCPP_WARN(this->get_logger(), "Obstacle flag reset.");
        obstacle_flag = false;
        counter_ = 0;
      } else if (depth_m < 2.0f && counter_ < 5) {
        counter_++;
      } else {
        counter_ = 0;
      }
    } else {
      if (depth_m < 2.0f && counter_ >= 5) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m, setting flag and publishing.", depth_m);
        
        navis_msgs::msg::ControlOut obstacle_is_msg;
        obstacle_is_msg.buzzer_strength = 0;
        obstacle_is_msg.speaker_wav_index = 21; // "Obstacle is" index in audio mappings

        obstacle_flag = true;
        pub_->publish(obstacle_is_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(250));

        counter_ = 0;
      } else if (depth_m < 2.0f && counter_ < 5) {
        counter_++;
      } else {
        counter_ = 0;
      }
    }
  }

  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_;
  rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr pub_;
  double baseline_;
  double focal_length_px_;
  double roi_width_fraction_;
  double roi_height_fraction_;
  bool obstacle_flag = false;
  int counter_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClosestObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
