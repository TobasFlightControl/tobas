#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

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

MjpgDecompressor::MjpgDecompressor(const rclcpp::NodeOptions& _options)
  : tobas::BaseNode("mjpg_decompressor", _options)
{
  std::string mjpg_topic = getStringParam("mjpg_topic", std::string("image_compressed"));
  sub_ = createSubscriber(mjpg_topic, &MjpgDecompressor::msgCb, this);

  std::string decoded_topic = getStringParam("decoded_topic", std::string("image_decompressed"));
  pub_ = createPublisher<sensor_msgs::msg::Image>(decoded_topic);
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
  catch (cv_bridge::Exception& e) {
    TOBAS_WARN("Could not convert to image!");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(MjpgDecompressor)
