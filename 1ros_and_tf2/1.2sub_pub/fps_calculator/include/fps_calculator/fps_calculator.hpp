#ifndef FPS_CALCULATOR_HPP
#define FPS_CALCULATOR_HPP

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class FpsCalculatorNode : public rclcpp::Node
{
public:
  FpsCalculatorNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);
  void timerCallback();

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t image_count_;
  double fps_;
};

#endif // FPS_CALCULATOR_HPP
