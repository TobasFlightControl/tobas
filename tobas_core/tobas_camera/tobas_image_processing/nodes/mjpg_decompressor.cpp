#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace tobas
{
namespace camera
{
/**
 * @brief sensor_msgs::msg::CompressedImage型の画像をsubscribeし，解凍して，sensor_msgs::msg::Image型としてpublishする．
 */
class MjpgDecompressor : public tobas::BaseNode
{
public:
  explicit MjpgDecompressor(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  void msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& _msg);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};

MjpgDecompressor::MjpgDecompressor(const rclcpp::NodeOptions& _options) : tobas::BaseNode("mjpg_decompressor", _options)
{
  const auto decoded_topic = getStringParam("decoded_topic", "image_decompressed");
  const auto mjpg_topic = getStringParam("mjpg_topic", "image_compressed");

  pub_ = createPublisher<sensor_msgs::msg::Image>(decoded_topic);
  sub_ = createSubscriber(mjpg_topic, &MjpgDecompressor::msgCb, this);
}

void MjpgDecompressor::msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->header = msg->header;
    cv_ptr->image = cv::imdecode(cv::Mat(msg->data), 1);
    switch (cv_ptr->image.channels()) {
      case 1:
        cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
        break;
      case 3:
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        break;
      default:
        TOBAS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
        break;
    }
    pub_->publish(*cv_ptr->toImageMsg());
  }
  catch (const cv_bridge::Exception&) {
    TOBAS_WARN("Could not convert to image!");
  }
}
}  // namespace camera
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::camera::MjpgDecompressor)
