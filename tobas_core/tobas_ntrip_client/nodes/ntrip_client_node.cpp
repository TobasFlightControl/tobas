#include <tobas_node/node.hpp>
#include <tobas_ntrip_client/ntrip_client.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/binary_packet.hpp>

using namespace std::chrono_literals;

class NtripClientNode : public tobas::BaseNode
{
  using self = NtripClientNode;
  using super = tobas::BaseNode;

public:
  explicit NtripClientNode(const rclcpp::NodeOptions& options);

private:
  static constexpr char kDefaultServerIp[] = "3.143.243.81";  // RTK2GO http://rtk2go.com/
  static constexpr int kDefaultServerPort = 2101;
  static constexpr char kDefaultPassword[] = "none";
  static constexpr std::chrono::duration kIntervalTime =
    1s;  // RTCM3.3 protocolのデータの受け取りに確認しに行く時間間隔

  ntrip::NtripClient ntrip_client_;
  ros2::TimerPtr timer_;
  ros2::PublisherPtr<tobas_msgs::msg::BinaryPacket> rtcm_pub_;

  void timerCallback();
};

NtripClientNode::NtripClientNode(const rclcpp::NodeOptions& options) : super("ntrip_client", options)
{
  auto server_ip = getStringParam("server_ip", kDefaultServerIp);
  auto server_port = getIntParam("server_port", kDefaultServerPort);
  auto mount_point = getStringParam("mount_point");
  auto user_name = getStringParam("user_name");
  auto password = getStringParam("password", kDefaultPassword);
  auto latitude = getDoubleParam("latitude");  // 最も近くのmount pointを探すために用いる 大体でok degree
  auto longitude = getDoubleParam("longitude");

  if (!ntrip_client_.initialize(
        server_ip.c_str(), server_port, mount_point.c_str(), user_name.c_str(), password.c_str(), latitude, longitude)) {
    TOBAS_ERROR("Failed to initialize NTRIP client.");
    return;
  }

  rtcm_pub_ = createPublisher<tobas_msgs::msg::BinaryPacket>(tobas::kRtcmCorrectionTopic);
  timer_ = createTimer(kIntervalTime, &NtripClientNode::timerCallback, this);
}

void NtripClientNode::timerCallback()
{
  auto packets = ntrip_client_.receiveRtcmData();
  for (size_t i = 0; i < packets.size(); i++) {
    auto msg = std::make_unique<tobas_msgs::msg::BinaryPacket>();
    msg->header.stamp = now();
    msg->data = packets[i];
    rtcm_pub_->publish(std::move(msg));
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(NtripClientNode)
