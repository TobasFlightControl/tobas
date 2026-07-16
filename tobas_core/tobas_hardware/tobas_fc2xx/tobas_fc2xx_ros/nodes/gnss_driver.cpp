// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ic_drivers/ublox/zed_f9p.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/gnss.hpp>

#include "./common.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace fc2xx
{
class GnssDriverNode : public BaseNode
{
  static constexpr char kSpiDevice[] = "/dev/spidev1.0";
  static constexpr auto kMainTimerPeriod = 1ms;

  // GNSS receiver update period [ms]
  // This cannot be too short because an overly high frequency fills the FIFO and causes a time shift.
  static constexpr size_t kMeasPeriod = 1000 / 20;

  using self = GnssDriverNode;
  using super = BaseNode;

public:
  explicit GnssDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ublox::ZEDF9P gnss_;

  ublox::payload::NAV_PVT pvt_;
  ublox::payload::NAV_COV cov_;

  std::map<ublox::ZEDF9P::UbxNavId, bool> is_received_;

  ros2::PublisherPtr<tobas_msgs::Gnss> gnss_pub_;
  ros2::TimerPtr initialize_timer_, main_timer_;

  void initialize();
  bool configure();
  void warnUnnecessaryUBXMessage();

  void mainTimerCb();
};

GnssDriverNode::GnssDriverNode(const rclcpp::NodeOptions& options)
  : super("fc2xx_gnss_driver", nodeOptions_Default(options))
{
  initialize_timer_ = createWallTimer(kRetryInitializationInterval, &self::initialize, this);
}

void GnssDriverNode::initialize()
{
  if (!gnss_.initialize(kSpiDevice)) {
    TOBAS_ERROR("Failed to initialize GNSS driver. Retrying...");
    return;
  }

  if (!configure()) {
    TOBAS_ERROR("Failed to configure GNSS receiver. Retrying...");
    return;
  }

  is_received_[ublox::ZEDF9P::NAV_PVT] = false;
  is_received_[ublox::ZEDF9P::NAV_COV] = false;

  gnss_pub_ = createPublisher<tobas_msgs::Gnss>(topic::kGnss);

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

bool GnssDriverNode::configure()
{
  if (!gnss_.configureDynamicsModel(ublox::ZEDF9P::AIRBORNE_2G)) {
    TOBAS_ERROR("Failed to configure dynamics model.");
    return false;
  }

  if (!gnss_.configureMeasurementRate(kMeasPeriod)) {
    TOBAS_ERROR("Failed to configure measurement rate.");
    return false;
  }

  // Enable GPS + SBAS + QZSS.
  // According to the datasheet, combining multiple main GNSS systems lowers the output frequency
  // while providing only a small accuracy improvement.
  if (!gnss_.enableGps()) {
    TOBAS_ERROR("Failed to enable GPS.");
    return false;
  }
  if (!gnss_.enableSbas()) {
    TOBAS_ERROR("Failed to enable SBAS.");
    return false;
  }
  if (!gnss_.disableGalileo()) {
    TOBAS_ERROR("Failed to disable Galileo.");
    return false;
  }
  if (!gnss_.disableBeiDou()) {
    TOBAS_ERROR("Failed to disable BeiDou.");
    return false;
  }
  if (!gnss_.enableQzss()) {
    TOBAS_ERROR("Failed to enable QZSS.");
    return false;
  }
  if (!gnss_.disableGlonass()) {
    TOBAS_ERROR("Failed to disable GLONASS.");
    return false;
  }

  // Enable messages.
  if (!gnss_.enableSpiMessage(ublox::ZEDF9P::CLASS_NAV, ublox::ZEDF9P::NAV_PVT, true)) {
    TOBAS_ERROR("Failed to enable NAV_PVT message.");
    return false;
  }
  if (!gnss_.enableSpiMessage(ublox::ZEDF9P::CLASS_NAV, ublox::ZEDF9P::NAV_COV, true)) {
    TOBAS_ERROR("Failed to enable NAV_COV message.");
    return false;
  }

  // Set the coaxial cable length.
  // TODO: Make this configurable from the GUI.
  if (!gnss_.setAntennaLength(1)) {
    TOBAS_ERROR("Failed to set the antenna length.");
    return false;
  }

  // Enable/Disable protocols.
  if (!gnss_.enableSpiProtocol_UBX(true, true)) {
    TOBAS_ERROR("Failed to enable UBX protocol.");
    return false;
  }
  if (!gnss_.enableSpiProtocol_NMEA(false, false)) {
    TOBAS_ERROR("Failed to disable NMEA protocol.");
    return false;
  }
  if (!gnss_.enableSpiProtocol_RTCM3X(true, false)) {  // Only input is needed for RTK.
    TOBAS_ERROR("Failed to disable RTCM3X protocol.");
    return false;
  }
  if (!gnss_.enableSpiProtocol_SPARTN(false)) {
    TOBAS_ERROR("Failed to disable SPARTN protocol.");
    return false;
  }

  // Disable unnecessary interfaces.
  // I2C and UART should already be disabled because D_SEL is off.
  if (!gnss_.enableUsb(false)) {
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

void GnssDriverNode::mainTimerCb()
{
  if (!gnss_.update()) {
    return;
  }

  if (gnss_.latestClass() != ublox::ZEDF9P::CLASS_NAV) {
    warnUnnecessaryUBXMessage();
    return;
  }

  switch (gnss_.latestId()) {
    case ublox::ZEDF9P::NAV_PVT:
      pvt_.decode(gnss_.payload());
      is_received_.at(ublox::ZEDF9P::NAV_PVT) = true;
      break;
    case ublox::ZEDF9P::NAV_COV:
      cov_.decode(gnss_.payload());
      is_received_.at(ublox::ZEDF9P::NAV_COV) = true;
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
}  // namespace fc2xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc2xx::GnssDriverNode)
