// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include <tobas_constants/ros.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/binary_packet.hpp>
#include <tobas_msgs/msg/gnss.hpp>

#include "tobas_ntrip_client/nmea_utils.hpp"
#include "tobas_ntrip_client/ntrip_client.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace
{
bool hasValidFix(const tobas_msgs::msg::Gnss& _gnss)
{
  return _gnss.fix_type >= tobas_msgs::msg::Gnss::FIX_2D;
}
}  // namespace

class NtripClientNode : public BaseNode
{
  using self = NtripClientNode;
  using super = BaseNode;

public:
  explicit NtripClientNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  static constexpr char kDefaultServerIp[] = "rtk2go.com";  // Ref: http://rtk2go.com/
  static constexpr int kDefaultServerPort = 2101;
  static constexpr double kDefaultSendPositionInterval = 1.0;  ///< [s]
  static constexpr double kDefaultReconnectInterval = 5.0;     ///< [s]

  // Japan's geodetic origin
  static constexpr double kDefaultLatitude = 35.6580992;
  static constexpr double kDefaultLongitude = 139.7413575;

  static constexpr auto kReceiveInterval = 20ms;  ///< Interval at which the caster is polled for RTCM data
  /** Time to wait for a GNSS fix after connecting before sending the position given by the parameters instead. */
  static constexpr auto kGnssWaitTimeout = 10s;
  static constexpr auto kMaxGnssAge = 3s;        ///< GNSS messages older than this are not used for GGA.
  static constexpr size_t kRtcmQueueSize = 100;  ///< Deep enough not to drop a burst of RTCM messages

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

  ros2::TimerPtr receive_timer_;
  ros2::TimerPtr position_timer_;
  ros2::TimerPtr reconnect_timer_;

  std::mutex gnss_mutex_;
  std::optional<tobas_msgs::msg::Gnss> latest_gnss_;
  std::chrono::steady_clock::time_point latest_gnss_time_;  ///< Time when `latest_gnss_` was received
  std::chrono::steady_clock::time_point connected_time_;    ///< Time when the connection to the caster was established

  bool tryConnect();
  void sendCurrentPosition();
  void receiveTimerCb();
  void positionTimerCb();
  void reconnectTimerCb();
  void gnssSubCb(const tobas_msgs::msg::Gnss::ConstSharedPtr& _msg);
};

NtripClientNode::NtripClientNode(const rclcpp::NodeOptions& _options) : super("ntrip_client", _options)
{
  server_ip_ = getStringParam("server_ip", kDefaultServerIp);
  server_port_ = static_cast<int>(getIntParam("server_port", kDefaultServerPort));
  mount_point_ = getStringParam("mount_point", "");
  user_name_ = getStringParam("user_name", "");
  password_ = getStringParam("password", "");
  default_latitude_ = getDoubleParam("latitude", kDefaultLatitude);
  default_longitude_ = getDoubleParam("longitude", kDefaultLongitude);
  send_position_ = getBoolParam("send_position", true);
  send_position_interval_ = getDoubleParam("send_position_interval", kDefaultSendPositionInterval);
  auto_reconnect_ = getBoolParam("auto_reconnect", true);
  reconnect_interval_ = getDoubleParam("reconnect_interval", kDefaultReconnectInterval);
  gnss_topic_ = getStringParam("gnss_topic", topic::kGnss);

  rtcm_pub_ = createPublisher<tobas_msgs::msg::BinaryPacket>(
    topic::kRtcmCorrection,
    false,  // latch
    true,   // reliable
    kRtcmQueueSize);

  if (send_position_) {
    gnss_sub_ = createSubscriber<tobas_msgs::msg::Gnss>(gnss_topic_, &self::gnssSubCb, this);
    const auto position_interval = std::chrono::duration<double>(send_position_interval_);
    position_timer_ = createTimer(position_interval, &self::positionTimerCb, this);
  }

  tryConnect();

  receive_timer_ = createTimer(kReceiveInterval, &self::receiveTimerCb, this);

  if (auto_reconnect_) {
    const auto reconnect_interval = std::chrono::duration<double>(reconnect_interval_);
    reconnect_timer_ = createTimer(reconnect_interval, &self::reconnectTimerCb, this);
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
  connected_time_ = std::chrono::steady_clock::now();

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
    const std::lock_guard lock(gnss_mutex_);
    const auto gnss_age = std::chrono::steady_clock::now() - latest_gnss_time_;
    if (latest_gnss_.has_value() && hasValidFix(*latest_gnss_) && gnss_age < kMaxGnssAge) {
      gga = ntrip::nmea::createGgaSentence(*latest_gnss_);
    }
  }

  if (gga.empty()) {
    // Casters that select the reference station automatically use the first GGA received,
    // so wait for a GNSS fix instead of sending the parameter position immediately after connecting.
    if (std::chrono::steady_clock::now() - connected_time_ < kGnssWaitTimeout) {
      return;
    }

    // Fall back to the position given by the parameters.
    TOBAS_WARN_THROTTLE(10.0, "No valid GNSS fix is available. The position given by the parameters is sent instead.");
    gga = ntrip::nmea::createGgaSentence(default_latitude_, default_longitude_);
  }

  if (!ntrip_client_.sendNmeaGga(gga)) {
    TOBAS_WARN("Failed to send NMEA GGA to NTRIP caster. Connection might be lost.");
  }
}

void NtripClientNode::receiveTimerCb()
{
  if (!ntrip_client_.isConnected()) {
    return;
  }

  const auto packets = ntrip_client_.receiveRtcmData();
  for (const auto& packet : packets) {
    auto msg = std::make_unique<tobas_msgs::msg::BinaryPacket>();
    msg->header.stamp = now();
    msg->data = packet;
    rtcm_pub_->publish(std::move(msg));
  }
}

void NtripClientNode::positionTimerCb()
{
  if (send_position_ && ntrip_client_.isConnected()) {
    sendCurrentPosition();
  }
}

void NtripClientNode::reconnectTimerCb()
{
  if (auto_reconnect_ && !ntrip_client_.isConnected()) {
    TOBAS_INFO("Attempting to reconnect to NTRIP Caster...");
    tryConnect();
  }
}

void NtripClientNode::gnssSubCb(const tobas_msgs::msg::Gnss::ConstSharedPtr& _msg)
{
  const std::lock_guard lock(gnss_mutex_);
  latest_gnss_ = *_msg;
  latest_gnss_time_ = std::chrono::steady_clock::now();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::NtripClientNode)
