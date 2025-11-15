#include <rclcpp/rclcpp.hpp>

#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

class CompressedImageViewer : public tobas::BaseNode
{
public:
  explicit CompressedImageViewer(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());
  ~CompressedImageViewer();

private:
  void msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& _msg);

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};
