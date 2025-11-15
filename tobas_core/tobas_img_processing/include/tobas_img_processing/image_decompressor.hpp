#include <rclcpp/rclcpp.hpp>

#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @brief sensor_msgs::msg::CompressedImage型の画像をsubscribeし，解凍して，sensor_msgs::msg::Image型としてpublishする．
 *  decompressしたimageはrviz2で見れる．
 */
class ImageDecompressor : public tobas::BaseNode
{
public:
  explicit ImageDecompressor(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  void msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& _msg);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};
