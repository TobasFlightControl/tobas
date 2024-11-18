#include <tobas_std_tools/gps.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_msgs_adapter/gps.hpp>

#include <tobas_aso_core/zed_f9p.hpp>

using namespace std;

class GNSSDriverNode : public hardware::BaseSensorNode
{
  // GNSSレシーバの更新周期 [ms]
  // 周波数が高すぎるとFIFOにデータが溜まってタイムシフトが生じるため，そんなに大きくできない
  static constexpr size_t kMeasPeriod = 1000 / 20;

  static constexpr auto kMainTimerPeriod = 1ms;
  static constexpr double kWarnPeriod = 3.;  // [s]

  using self = GNSSDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit GNSSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::ZEDF9P gnss_;

  aso::payload::NAV_STATUS status_;
  aso::payload::NAV_HPPOSLLH hpposllh_;
  aso::payload::NAV_VELNED velned_;
  aso::payload::NAV_COV cov_;

  std::map<aso::ZEDF9P::ubx_nav_id_t, bool> is_received_;

  ros2::PublisherPtr<tobas_msgs::Gps> gnss_pub_;

  bool configure();
  void warnUnnecessaryUBXMessage();

  void mainTimerCb();
};

GNSSDriverNode::GNSSDriverNode(const rclcpp::NodeOptions& options) : super("aso_gnss_driver", options)
{
  if (!gnss_.initialize())
  {
    TOBAS_EXIT("Failed to initialize GNSS driver.");
    return;
  }

  if (!configure())
  {
    TOBAS_EXIT("Failed to configure GNSS receiver.");
    return;
  }

  is_received_[aso::ZEDF9P::NAV_STATUS] = false;
  is_received_[aso::ZEDF9P::NAV_HPPOSLLH] = false;
  is_received_[aso::ZEDF9P::NAV_VELNED] = false;
  is_received_[aso::ZEDF9P::NAV_COV] = false;

  gnss_pub_ = createPublisher<tobas_msgs::Gps>(tobas::kGNSSTopic);

  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

bool GNSSDriverNode::configure()
{
  if (!gnss_.configureDynamicsModel(aso::ZEDF9P::AIRBORNE_2G))
  {
    TOBAS_FATAL("Failed to configure dynamics model.");
    return false;
  }

  if (!gnss_.configureMeasurementRate(kMeasPeriod))
  {
    TOBAS_FATAL("Failed to configure measurement rate.");
    return false;
  }

  // GPS + SBAS + QZSSを有効化
  // データシートを見るに，複数のメインGNSSを組み合わせると精度はあまり変化しない割に出力周波数が落ちる
  if (!gnss_.enableGPS(true))
  {
    TOBAS_FATAL("Failed to enable GPS.");
    return false;
  }
  if (!gnss_.enableSBAS(true))
  {
    TOBAS_FATAL("Failed to enable SBAS.");
    return false;
  }
  if (!gnss_.enableGalileo(false))
  {
    TOBAS_FATAL("Failed to disable Galileo.");
    return false;
  }
  if (!gnss_.enableBeiDou(false))
  {
    TOBAS_FATAL("Failed to disable BeiDou.");
    return false;
  }
  if (!gnss_.enableQZSS(true))
  {
    TOBAS_FATAL("Failed to enable QZSS.");
    return false;
  }
  if (!gnss_.enableGLONASS(false))
  {
    TOBAS_FATAL("Failed to disable GLONASS.");
    return false;
  }

  // Enable messages
  if (!gnss_.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_STATUS, true))
    TOBAS_EXIT("Failed to enable NAV_STATUS message.");
  if (!gnss_.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_HPPOSLLH, true))
    TOBAS_EXIT("Failed to enable NAV_HPPOSLLH message.");
  if (!gnss_.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_VELNED, true))
    TOBAS_EXIT("Failed to enable NAV_VELNED message.");
  if (!gnss_.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_COV, true))
    TOBAS_EXIT("Failed to enable NAV_COV message.");

  // 同軸ケーブルの長さを設定
  // TODO: GUIから設定できるようにする
  if (!gnss_.setAntennaLength(1))
    TOBAS_WARN("Failed to set the antenna length.");

  // 不要なプロトコルを無効化
  if (!gnss_.enableProtocol(aso::ZEDF9P::NMEA, false))
    TOBAS_WARN("Failed to disable NMEA protocol.");
  if (!gnss_.enableProtocol(aso::ZEDF9P::RTCM3X, false))
    TOBAS_WARN("Failed to disable RTCM3X protocol.");
  if (!gnss_.enableProtocol(aso::ZEDF9P::SPARTN, false))
    TOBAS_WARN("Failed to disable SPARTN protocol.");

  // 不要なインターフェースを無効化
  // D_SELをオフにしているため，I2CとUARTは始めから無効化されているはず．
  if (!gnss_.enableUSB(false))
    TOBAS_WARN("Failed to disable USB interface.");

  return true;
}

void GNSSDriverNode::warnUnnecessaryUBXMessage()
{
  const auto cls = gnss_.latestClass();
  const auto id = gnss_.latestId();
  TOBAS_WARN("Unnecessary UBX message is received: (Class, ID) = (", (int)cls, ", ", (int)id, ")");
}

void GNSSDriverNode::mainTimerCb()
{
  if (!gnss_.update())
    return;

  if (gnss_.latestClass() != aso::ZEDF9P::CLASS_NAV)
  {
    warnUnnecessaryUBXMessage();
    return;
  }

  switch (gnss_.latestId())
  {
    case aso::ZEDF9P::NAV_STATUS:
      status_.decode(gnss_.payload());
      is_received_.at(aso::ZEDF9P::NAV_STATUS) = true;
      break;
    case aso::ZEDF9P::NAV_HPPOSLLH:
      hpposllh_.decode(gnss_.payload());
      is_received_.at(aso::ZEDF9P::NAV_HPPOSLLH) = true;
      break;
    case aso::ZEDF9P::NAV_VELNED:
      velned_.decode(gnss_.payload());
      is_received_.at(aso::ZEDF9P::NAV_VELNED) = true;
      break;
    case aso::ZEDF9P::NAV_COV:
      cov_.decode(gnss_.payload());
      is_received_.at(aso::ZEDF9P::NAV_COV) = true;
      break;
    default:
      warnUnnecessaryUBXMessage();
      return;
  }

  // 全てのメッセージが更新されたら発行
  for (const auto& [_, received] : is_received_)
    if (!received)
      return;

  // Reset UBX message checker flags
  for (auto& [_, received] : is_received_)
    received = false;

  // GPSメッセージの遅延を表示 (デバッグモードのみ)
  if (get_logger().get_effective_level() <= rclcpp::Logger::Level::Debug)
  {
    const auto delay_ms = tobas_std::computeGPSDelayFromToW(hpposllh_.iTOW);
    TOBAS_DEBUG("GPS delay: ", delay_ms, "[ms]");
  }

  // Create GNSS message
  auto gnss_msg = std::make_unique<tobas_msgs::Gps>();

  // Fill time stamp
  // TODO: GNSS信号の遅延を測定
  gnss_msg->header.stamp = get_clock()->now() - rclcpp::Duration::from_nanoseconds(80'000'000);

  // Fill fix type
  gnss_msg->fix_type = status_.gpsFix;

  // Fill position
  gnss_msg->latitude = hpposllh_.lat + hpposllh_.latHp;                 // Latitude [deg]
  gnss_msg->longitude = hpposllh_.lon + hpposllh_.lonHp;                // Longitude [deg]
  gnss_msg->altitude = (hpposllh_.height + hpposllh_.heightHp) * 1e-3;  // Height above ellipsoid [m]

  // Fill velocity
  gnss_msg->ground_speed.x(velned_.velN * 1e-2);   // North velocity [m/s]
  gnss_msg->ground_speed.y(-velned_.velE * 1e-2);  // West velocity [m/s]
  gnss_msg->ground_speed.z(-velned_.velD * 1e-2);  // Up velocity [m/s]

  // Fill covariances
  gnss_msg->position_covariance(0, 0) = cov_.posCovNN;
  gnss_msg->position_covariance(0, 1) = cov_.posCovNE;
  gnss_msg->position_covariance(0, 2) = cov_.posCovND;
  gnss_msg->position_covariance(1, 0) = cov_.posCovNE;
  gnss_msg->position_covariance(1, 1) = cov_.posCovEE;
  gnss_msg->position_covariance(1, 2) = cov_.posCovED;
  gnss_msg->position_covariance(2, 0) = cov_.posCovND;
  gnss_msg->position_covariance(2, 1) = cov_.posCovED;
  gnss_msg->position_covariance(2, 2) = cov_.posCovDD;
  gnss_msg->velocity_covariance(0, 0) = cov_.velCovNN;
  gnss_msg->velocity_covariance(0, 1) = cov_.velCovNE;
  gnss_msg->velocity_covariance(0, 2) = cov_.velCovND;
  gnss_msg->velocity_covariance(1, 0) = cov_.velCovNE;
  gnss_msg->velocity_covariance(1, 1) = cov_.velCovEE;
  gnss_msg->velocity_covariance(1, 2) = cov_.velCovED;
  gnss_msg->velocity_covariance(2, 0) = cov_.velCovND;
  gnss_msg->velocity_covariance(2, 1) = cov_.velCovED;
  gnss_msg->velocity_covariance(2, 2) = cov_.velCovDD;

  // Publish GNSS message
  gnss_pub_->publish(move(gnss_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(GNSSDriverNode)
