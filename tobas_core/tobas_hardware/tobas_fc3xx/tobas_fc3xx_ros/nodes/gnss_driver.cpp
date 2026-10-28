// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <condition_variable>
#include <mutex>
#include <stop_token>
#include <thread>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_hardware_common/constants.hpp>
#include <tobas_ic_drivers/ublox/gnss_receiver.hpp>
#include <tobas_ic_drivers/ublox/ubx_transport_uart.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/gnss.hpp>

using namespace std::chrono_literals;

namespace tobas::fc3xx
{
class GnssDriverNode : public BaseNode
{
  using super = BaseNode;

public:
  explicit GnssDriverNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  ublox::GnssReceiver gnss_;

  ublox::payload::NAV_PVT pvt_;
  ublox::payload::NAV_COV cov_;

  std::map<ublox::GnssReceiver::UbxNavId, bool> is_received_;

  ros2::PublisherPtr<tobas_msgs::Gnss> gnss_pub_;
  std::jthread read_thread_;

  bool initialize(std::stop_token _stop_token);
  bool configure(std::stop_token _stop_token);
  void warnUnnecessaryUBXMessage();

  void readThreadFunc(std::stop_token _stop_token);
  void handleMessage();
};

GnssDriverNode::GnssDriverNode(const rclcpp::NodeOptions& _options)
  : super("fc3xx_gnss_driver", nodeOptions_Default(_options))
  , gnss_(std::make_unique<ublox::UbxTransportUart>("/dev/ttyAMA1"))
{
  gnss_pub_ = createPublisher<tobas_msgs::Gnss>(topic::kGnss);
  read_thread_ = std::jthread([this](std::stop_token _stop_token) { readThreadFunc(_stop_token); });
}

bool GnssDriverNode::initialize(std::stop_token _stop_token)
{
  if (!gnss_.initialize()) {
    return false;
  }

  if (!configure(_stop_token)) {
    return false;
  }

  is_received_[ublox::GnssReceiver::NAV_PVT] = false;
  is_received_[ublox::GnssReceiver::NAV_COV] = false;

  return true;
}

bool GnssDriverNode::configure(std::stop_token _stop_token)
{
  if (_stop_token.stop_requested() || !gnss_.configureDynamicsModel(ublox::GnssReceiver::AIRBORNE_2G)) {
    TOBAS_ERROR("Failed to configure dynamics model.");
    return false;
  }

  // The measurement rate cannot be too short because an overly high frequency fills the FIFO and causes a time shift.
  constexpr uint16_t kMeasRate = 20;  // [Hz]
  if (_stop_token.stop_requested() || !gnss_.configureMeasurementRate(1000 / kMeasRate)) {
    TOBAS_ERROR("Failed to configure measurement rate.");
    return false;
  }

  // Enable GPS + SBAS + QZSS.
  // According to the datasheet, combining multiple main GNSS systems lowers the output frequency
  // while providing only a small accuracy improvement.
  if (_stop_token.stop_requested() || !gnss_.enableGps()) {
    TOBAS_ERROR("Failed to enable GPS.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.enableSbas()) {
    TOBAS_ERROR("Failed to enable SBAS.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.disableGalileo()) {
    TOBAS_ERROR("Failed to disable Galileo.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.disableBeiDou()) {
    TOBAS_ERROR("Failed to disable BeiDou.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.enableQzss()) {
    TOBAS_ERROR("Failed to enable QZSS.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.disableGlonass()) {
    TOBAS_ERROR("Failed to disable GLONASS.");
    return false;
  }

  // Enable messages.
  if (
    _stop_token.stop_requested() ||
    !gnss_.enableMessage(ublox::GnssReceiver::CLASS_NAV, ublox::GnssReceiver::NAV_PVT, true)) {
    TOBAS_ERROR("Failed to enable NAV_PVT message.");
    return false;
  }
  if (
    _stop_token.stop_requested() ||
    !gnss_.enableMessage(ublox::GnssReceiver::CLASS_NAV, ublox::GnssReceiver::NAV_COV, true)) {
    TOBAS_ERROR("Failed to enable NAV_COV message.");
    return false;
  }

  // Set the coaxial cable length.
  // TODO: Make this configurable from the GUI.
  if (_stop_token.stop_requested() || !gnss_.setAntennaLength(1)) {
    TOBAS_ERROR("Failed to set the antenna length.");
    return false;
  }

  // Enable/Disable protocols.
  if (_stop_token.stop_requested() || !gnss_.enableProtocol_UBX(true, true)) {
    TOBAS_ERROR("Failed to enable UBX protocol.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.enableProtocol_NMEA(false, false)) {
    TOBAS_ERROR("Failed to disable NMEA protocol.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.enableProtocol_RTCM3X(true, false)) {  // Only input is needed for RTK.
    TOBAS_ERROR("Failed to disable RTCM3X protocol.");
    return false;
  }
  if (_stop_token.stop_requested() || !gnss_.enableProtocol_SPARTN(false)) {
    TOBAS_ERROR("Failed to disable SPARTN protocol.");
    return false;
  }

  // Disable unnecessary interfaces.
  if (_stop_token.stop_requested() || !gnss_.enableUsb(false)) {
    TOBAS_ERROR("Failed to disable USB interface.");
    return false;
  }

  return true;
}

void GnssDriverNode::warnUnnecessaryUBXMessage()
{
  const auto cls = gnss_.latestClass();
  const auto id = gnss_.latestId();
  TOBAS_WARN("Unnecessary UBX message is received: (Class, ID) = (", (int)cls, ", ", (int)id, ")");
}

void GnssDriverNode::readThreadFunc(std::stop_token _stop_token)
{
  while (!_stop_token.stop_requested()) {
    if (initialize(_stop_token)) {
      break;
    }
    if (_stop_token.stop_requested()) {
      return;
    }
    TOBAS_ERROR("Failed to initialize or configure GNSS receiver. Retrying...");
    std::mutex mutex;
    std::condition_variable_any cv;
    std::unique_lock lock(mutex);
    cv.wait_for(lock, _stop_token, hardware::kRetryInitializationInterval, [] { return false; });
  }

  while (!_stop_token.stop_requested()) {
    if (gnss_.update(true, _stop_token)) {
      handleMessage();
    }
  }
}

void GnssDriverNode::handleMessage()
{
  if (gnss_.latestClass() != ublox::GnssReceiver::CLASS_NAV) {
    warnUnnecessaryUBXMessage();
    return;
  }

  switch (gnss_.latestId()) {
    case ublox::GnssReceiver::NAV_PVT:
      pvt_.decode(gnss_.payload());
      is_received_.at(ublox::GnssReceiver::NAV_PVT) = true;
      break;
    case ublox::GnssReceiver::NAV_COV:
      cov_.decode(gnss_.payload());
      is_received_.at(ublox::GnssReceiver::NAV_COV) = true;
      break;
    default:
      warnUnnecessaryUBXMessage();
      return;
  }

  // Publish after all messages have been updated.
  for (const auto& [_, received] : is_received_) {
    if (!received) {
      return;
    }
  }

  // Reset UBX message checker flags.
  for (auto& [_, received] : is_received_) {
    received = false;
  }

  // Create GNSS message.
  auto gnss_msg = std::make_unique<tobas_msgs::Gnss>();

  // Fill time stamp.
  // TODO: Measure GNSS signal delay.
  gnss_msg->header.stamp = now() - rclcpp::Duration::from_nanoseconds(80'000'000);

  // Fill position.
  gnss_msg->latitude = pvt_.lat;
  gnss_msg->longitude = pvt_.lon;
  gnss_msg->height_wgs84 = static_cast<double>(pvt_.height) * 1e-3;
  gnss_msg->height_msl = static_cast<double>(pvt_.hMSL) * 1e-3;
  gnss_msg->position_covariance(0, 0) = cov_.posCovNN;
  gnss_msg->position_covariance(0, 1) = cov_.posCovNE;
  gnss_msg->position_covariance(0, 2) = cov_.posCovND;
  gnss_msg->position_covariance(1, 0) = cov_.posCovNE;
  gnss_msg->position_covariance(1, 1) = cov_.posCovEE;
  gnss_msg->position_covariance(1, 2) = cov_.posCovED;
  gnss_msg->position_covariance(2, 0) = cov_.posCovND;
  gnss_msg->position_covariance(2, 1) = cov_.posCovED;
  gnss_msg->position_covariance(2, 2) = cov_.posCovDD;

  // Fill velocity.
  gnss_msg->ground_speed.x(static_cast<double>(pvt_.velE) * 1e-3);
  gnss_msg->ground_speed.y(static_cast<double>(pvt_.velN) * 1e-3);
  gnss_msg->ground_speed.z(-static_cast<double>(pvt_.velD) * 1e-3);
  gnss_msg->velocity_covariance(0, 0) = cov_.velCovNN;
  gnss_msg->velocity_covariance(0, 1) = cov_.velCovNE;
  gnss_msg->velocity_covariance(0, 2) = cov_.velCovND;
  gnss_msg->velocity_covariance(1, 0) = cov_.velCovNE;
  gnss_msg->velocity_covariance(1, 1) = cov_.velCovEE;
  gnss_msg->velocity_covariance(1, 2) = cov_.velCovED;
  gnss_msg->velocity_covariance(2, 0) = cov_.velCovND;
  gnss_msg->velocity_covariance(2, 1) = cov_.velCovED;
  gnss_msg->velocity_covariance(2, 2) = cov_.velCovDD;

  // Fill other status.
  gnss_msg->fix_type = pvt_.fixType;
  gnss_msg->num_satellites_used = pvt_.numSV;

  // Publish GNSS message.
  gnss_pub_->publish(std::move(gnss_msg));
}
}  // namespace tobas::fc3xx

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc3xx::GnssDriverNode)
