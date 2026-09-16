#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include <tobas_constants/ntrip.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_msgs/msg/binary_packet.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ntrip_client/nmea_utils.hpp>
#include <tobas_ntrip_client/ntrip_client.hpp>

using namespace std::chrono_literals;

namespace tobas
{

class NtripClientNode : public tobas::BaseNode
{
  using self = NtripClientNode;
  using super = tobas::BaseNode;

public:
  explicit NtripClientNode(const rclcpp::NodeOptions& options);

private:
  static constexpr char kDefaultServerIp[] = "3.14.70.106";  // RTK2GO http://rtk2go.com/
  static constexpr int kDefaultServerPort = 2101;
  static constexpr char kDefaultPassword[] = "none";
  static constexpr std::chrono::duration kIntervalTime =
    1s;  // RTCM3.3 protocolのデータの受け取りに確認しに行く時間間隔
  static constexpr double kDefaultSendPositionInterval = 1.0;  // [s]
  static constexpr double kDefaultReconnectInterval = 5.0;     // [s]

  std::string server_ip_;
  int server_port_;
  std::string mount_point_;
  std::string user_name_;
  std::string password_;
  double default_latitude_;
  double default_longitude_;
  bool send_position_;
  double send_position_interval_;
  bool auto_reconnect_;
  double reconnect_interval_;
  std::string gnss_topic_;

  ntrip::NtripClient ntrip_client_;
  ros2::PublisherPtr<tobas_msgs::msg::BinaryPacket> rtcm_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Gnss> gnss_sub_;

  ros2::TimerPtr timer_;
  ros2::TimerPtr position_timer_;
  ros2::TimerPtr reconnect_timer_;

  std::mutex gnss_mutex_;
  std::optional<tobas_msgs::msg::Gnss> latest_gnss_;

  bool tryConnect();
  void sendCurrentPosition();
  void timerCallback();
  void positionTimerCallback();
  void reconnectTimerCallback();
  void gnssCallback(const tobas_msgs::msg::Gnss::ConstSharedPtr& msg);
};

NtripClientNode::NtripClientNode(const rclcpp::NodeOptions& options) : super("ntrip_client", options)
{
  server_ip_ = getStringParam("server_ip", kDefaultServerIp);
  server_port_ = getIntParam("server_port", kDefaultServerPort);
  mount_point_ = getStringParam("mount_point");
  user_name_ = getStringParam("user_name");
  password_ = getStringParam("password", kDefaultPassword);
  default_latitude_ = getDoubleParam("latitude", 0.0);  // 最も近くのmount pointを探すため / 初期位置 degree
  default_longitude_ = getDoubleParam("longitude", 0.0);
  send_position_ = getBoolParam("send_position", true);
  send_position_interval_ = getDoubleParam("send_position_interval", kDefaultSendPositionInterval);
  auto_reconnect_ = getBoolParam("auto_reconnect", true);
  reconnect_interval_ = getDoubleParam("reconnect_interval", kDefaultReconnectInterval);
  gnss_topic_ = getStringParam("gnss_topic", topic::kGnss);

  rtcm_pub_ = createPublisher<tobas_msgs::msg::BinaryPacket>(topic::kRtcmCorrection);

  if (send_position_) {
    gnss_sub_ = createSubscriber<tobas_msgs::msg::Gnss>(gnss_topic_, &NtripClientNode::gnssCallback, this);
    auto pos_interval = std::chrono::duration<double>(send_position_interval_);
    position_timer_ = createTimer(pos_interval, &NtripClientNode::positionTimerCallback, this);
  }

  // 初回接続試行
  tryConnect();

  timer_ = createTimer(kIntervalTime, &NtripClientNode::timerCallback, this);

  if (auto_reconnect_) {
    auto rec_interval = std::chrono::duration<double>(reconnect_interval_);
    reconnect_timer_ = createTimer(rec_interval, &NtripClientNode::reconnectTimerCallback, this);
  }
}

bool NtripClientNode::tryConnect()
{
  if (ntrip_client_.isConnected()) {
    return true;
  }

  TOBAS_INFO("Connecting to NTRIP Caster (", server_ip_, ":", server_port_, ", mount_point: ", mount_point_, ")...");

  if (!ntrip_client_.initialize(
        server_ip_.c_str(),
        server_port_,
        mount_point_.c_str(),
        user_name_.c_str(),
        password_.c_str(),
        default_latitude_,
        default_longitude_)) {
    TOBAS_WARN("Failed to initialize NTRIP client.");
    return false;
  }

  TOBAS_INFO("Successfully connected to NTRIP Caster.");

  if (send_position_) {
    sendCurrentPosition();
  }

  return true;
}

void NtripClientNode::sendCurrentPosition()
{
  if (!ntrip_client_.isConnected()) {
    return;
  }

  std::string gga;
  {
    std::lock_guard<std::mutex> lock(gnss_mutex_);
    if (latest_gnss_.has_value()) {
      gga = ntrip::nmea::createGgaSentence(*latest_gnss_);
    }
  }

  if (gga.empty()) {
    // GNSS未受信時のフォールバック: 設定パラメータから生成
    gga = ntrip::nmea::createGgaSentence(default_latitude_, default_longitude_);
  }

  if (!ntrip_client_.sendNmeaGga(gga)) {
    TOBAS_WARN("Failed to send NMEA GGA to NTRIP caster. Connection might be lost.");
  }
}

void NtripClientNode::timerCallback()
{
  if (!ntrip_client_.isConnected()) {
    return;
  }

  auto packets = ntrip_client_.receiveRtcmData();
  for (size_t i = 0; i < packets.size(); i++) {
    auto msg = std::make_unique<tobas_msgs::msg::BinaryPacket>();
    msg->header.stamp = now();
    msg->data = packets[i];
    rtcm_pub_->publish(std::move(msg));
  }
}

void NtripClientNode::positionTimerCallback()
{
  if (send_position_ && ntrip_client_.isConnected()) {
    sendCurrentPosition();
  }
}

void NtripClientNode::reconnectTimerCallback()
{
  if (auto_reconnect_ && !ntrip_client_.isConnected()) {
    TOBAS_INFO("Attempting to reconnect to NTRIP Caster...");
    tryConnect();
  }
}

void NtripClientNode::gnssCallback(const tobas_msgs::msg::Gnss::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(gnss_mutex_);
  latest_gnss_ = *msg;
}

}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::NtripClientNode)
