// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_hardware_common/constants.hpp>
#include <tobas_ic_drivers/ublox/gnss_receiver.hpp>
#include <tobas_ic_drivers/ublox/ubx_transport_spi.hpp>
#include <tobas_std_tools/gnss.hpp>

#include <tobas_msgs/msg/binary_packet.hpp>
#include <tobas_msgs_adapter/gnss.hpp>

using namespace std::chrono_literals;

namespace tobas::fc1xx
{
namespace
{
/**
 * Converts the UBX fix status into a `tobas_msgs::msg::Gnss` fix type.
 *
 * @param _fix_type Fix type reported by UBX-NAV-PVT.
 * @param _carr_soln Carrier phase range solution status reported by UBX-NAV-PVT.
 * @return One of the `fix_type` constants of `tobas_msgs::msg::Gnss`.
 *
 * @note Dead-reckoning and time-only fixes are reported as `NO_FIX` because they carry no GNSS position.
 */
uint8_t toGnssFixType(ublox::payload::FixType _fix_type, ublox::payload::NAV_PVT::CarrierPhaseRangeSolution _carr_soln)
{
  switch (_fix_type) {
    // A 3D fix may carry an RTK solution, so the carrier phase range solution status refines the result.
    case ublox::payload::FIX_3D:
    case ublox::payload::GPS_DEAD_RECKONING_COMBINED: {
      switch (_carr_soln) {
        case ublox::payload::NAV_PVT::FIXED:
          return tobas_msgs::msg::Gnss::RTK_FIX;
        case ublox::payload::NAV_PVT::FLOATING:
          return tobas_msgs::msg::Gnss::RTK_FLOAT;
        case ublox::payload::NAV_PVT::NONE:
          return tobas_msgs::msg::Gnss::FIX_3D;
      }
      return tobas_msgs::msg::Gnss::FIX_3D;
    }
    case ublox::payload::FIX_2D:
      return tobas_msgs::msg::Gnss::FIX_2D;
    case ublox::payload::NO_FIX:
    case ublox::payload::DEAD_RECKONING_ONLY:
    case ublox::payload::TIME_ONLY_FIX:
      return tobas_msgs::msg::Gnss::NO_FIX;
  }

  // The fix type is decoded from the wire, so an undefined value must be treated as no fix.
  return tobas_msgs::msg::Gnss::NO_FIX;
}
}  // namespace

class GnssDriverNode : public hardware::BaseSensorNode
{
  using self = GnssDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit GnssDriverNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  ublox::GnssReceiver gnss_;

  ublox::payload::NAV_PVT pvt_;
  ublox::payload::NAV_COV cov_;

  std::map<ublox::GnssReceiver::UbxNavId, bool> is_received_;

  ros2::PublisherPtr<tobas_msgs::Gnss> gnss_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::BinaryPacket> rtcm_correction_sub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  bool configure();
  void warnUnnecessaryUBXMessage();

  void mainTimerCb();
  void rtcmCorrectionSubCb(const tobas_msgs::msg::BinaryPacket::ConstSharedPtr& _rtcm_correction);
};

GnssDriverNode::GnssDriverNode(const rclcpp::NodeOptions& _options)
  : super("fc1xx_gnss_driver", nodeOptions_Default(_options))
  , gnss_(std::make_unique<ublox::UbxTransportSpi>("/dev/spidev1.2"))
{
  initialize_timer_ = createWallTimer(hardware::kRetryInitializationInterval, &self::initialize, this);
}

void GnssDriverNode::initialize()
{
  if (!gnss_.initialize()) {
    TOBAS_ERROR("Failed to initialize GNSS driver. Retrying...");
    return;
  }

  if (!configure()) {
    TOBAS_ERROR("Failed to configure GNSS receiver. Retrying...");
    return;
  }

  is_received_[ublox::GnssReceiver::NAV_PVT] = false;
  is_received_[ublox::GnssReceiver::NAV_COV] = false;

  gnss_pub_ = createPublisher<tobas_msgs::Gnss>(topic::kGnss);

  // The queue must be deep enough not to drop the burst of RTCM messages that arrives in one epoch.
  constexpr size_t kRtcmQueueSize = 100;
  rtcm_correction_sub_ = createSubscriber(
    topic::kRtcmCorrection,
    &self::rtcmCorrectionSubCb,
    this,
    false,  // latch
    true,   // reliable
    kRtcmQueueSize);

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(1ms, &self::mainTimerCb, this);
}

bool GnssDriverNode::configure()
{
  if (!gnss_.configureDynamicsModel(ublox::GnssReceiver::AIRBORNE_2G)) {
    TOBAS_ERROR("Failed to configure dynamics model.");
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
  if (!gnss_.enableMessage(ublox::GnssReceiver::CLASS_NAV, ublox::GnssReceiver::NAV_PVT, true)) {
    TOBAS_ERROR("Failed to enable NAV_PVT message.");
    return false;
  }
  if (!gnss_.enableMessage(ublox::GnssReceiver::CLASS_NAV, ublox::GnssReceiver::NAV_COV, true)) {
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
  if (!gnss_.enableProtocol_UBX(true, true)) {
    TOBAS_ERROR("Failed to enable UBX protocol.");
    return false;
  }
  if (!gnss_.enableProtocol_NMEA(false, false)) {
    TOBAS_ERROR("Failed to disable NMEA protocol.");
    return false;
  }
  if (!gnss_.enableProtocol_RTCM3X(true, false)) {  // Only input is needed for RTK.
    TOBAS_ERROR("Failed to disable RTCM3X protocol.");
    return false;
  }
  if (!gnss_.enableProtocol_SPARTN(false)) {
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
  TOBAS_WARN(
    "Unnecessary UBX message received: (Class, ID) = (", static_cast<int>(cls), ", ", static_cast<int>(id), ")");
}

void GnssDriverNode::mainTimerCb()
{
  if (!gnss_.update()) {
    return;
  }

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

  // Show GNSS message delay in debug mode only.
  if (get_logger().get_effective_level() <= rclcpp::Logger::Level::Debug) {
    const auto delay_ms = st::computeGpsDelayFromToW(pvt_.iTOW);
    TOBAS_DEBUG("GNSS delay: ", delay_ms, "[ms]");
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
  gnss_msg->position_covariance(0, 0) = cov_.posCovEE;   // EE
  gnss_msg->position_covariance(0, 1) = cov_.posCovNE;   // EN
  gnss_msg->position_covariance(0, 2) = -cov_.posCovED;  // EU
  gnss_msg->position_covariance(1, 0) = cov_.posCovNE;   // NE
  gnss_msg->position_covariance(1, 1) = cov_.posCovNN;   // NN
  gnss_msg->position_covariance(1, 2) = -cov_.posCovND;  // NU
  gnss_msg->position_covariance(2, 0) = -cov_.posCovED;  // UE
  gnss_msg->position_covariance(2, 1) = -cov_.posCovND;  // UN
  gnss_msg->position_covariance(2, 2) = cov_.posCovDD;   // UU

  // Fill velocity.
  gnss_msg->ground_speed.x(static_cast<double>(pvt_.velE) * 1e-3);
  gnss_msg->ground_speed.y(static_cast<double>(pvt_.velN) * 1e-3);
  gnss_msg->ground_speed.z(-static_cast<double>(pvt_.velD) * 1e-3);
  gnss_msg->velocity_covariance(0, 0) = cov_.velCovEE;   // EE
  gnss_msg->velocity_covariance(0, 1) = cov_.velCovNE;   // EN
  gnss_msg->velocity_covariance(0, 2) = -cov_.velCovED;  // EU
  gnss_msg->velocity_covariance(1, 0) = cov_.velCovNE;   // NE
  gnss_msg->velocity_covariance(1, 1) = cov_.velCovNN;   // NN
  gnss_msg->velocity_covariance(1, 2) = -cov_.velCovND;  // NU
  gnss_msg->velocity_covariance(2, 0) = -cov_.velCovED;  // UE
  gnss_msg->velocity_covariance(2, 1) = -cov_.velCovND;  // UN
  gnss_msg->velocity_covariance(2, 2) = cov_.velCovDD;   // UU

  // Fill other status.
  gnss_msg->fix_type = toGnssFixType(pvt_.fixType, pvt_.carrSoln);
  gnss_msg->num_satellites_used = pvt_.numSV;

  // Publish GNSS message.
  gnss_pub_->publish(std::move(gnss_msg));
}

void GnssDriverNode::rtcmCorrectionSubCb(const tobas_msgs::msg::BinaryPacket::ConstSharedPtr& _rtcm_correction)
{
  // The data is queued in the transport and sent while UBX messages are received in `mainTimerCb()`.
  if (!gnss_.registerRtcmCorrectionData(_rtcm_correction->data)) {
    TOBAS_WARN_THROTTLE(1.0, "Failed to send RTCM correction data to the GNSS receiver.");
  }
}
}  // namespace tobas::fc1xx

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc1xx::GnssDriverNode)
