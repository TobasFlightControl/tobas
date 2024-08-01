#include <tobas_std_tools/time.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Gps.h>

#include "../include/tobas_a1_ros/gnss_driver.hpp"
#include "../include/tobas_a1_ros/common.hpp"

using namespace std;

namespace a1
{
GNSSDriver::GNSSDriver(, const string& name) : super(node, pnh, name)
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

  is_received_[ZEDF9P::NAV_STATUS] = false;
  is_received_[ZEDF9P::NAV_POSLLH] = false;
  is_received_[ZEDF9P::NAV_VELNED] = false;
  is_received_[ZEDF9P::NAV_COV] = false;

  gnss_pub_ = node_.advertise<tobas_msgs::Gps>(tobas::kGpsTopic, 1);

  set_time_offset_timer_ = node_.createTimer(rclcpp::Duration(0), &self::setTimeOffsetTimerCb, this, true, false);
  main_timer_ = node_.createTimer(rclcpp::Duration(0), &self::mainTimerCb, this, false, false);

  // ROS時刻とGPS時刻の間のオフセットを取得する
  // コンストラクタではros::Timeを扱うことは推奨されないため，タイマーコールバックで行う．
  set_time_offset_timer_.start();
}

bool GNSSDriver::configure()
{
  if (!gnss_.configureDynamicsModel(ZEDF9P::AIRBORNE_2G))
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

  // 同軸ケーブルの長さを設定
  // TODO: GUIから設定できるようにする
  if (!gnss_.setAntennaLength(5))
  {
    TOBAS_FATAL("Failed to set the antenna length.");
    return false;
  }

  // 不要なプロトコルを無効化
  if (!gnss_.enableProtocol(ZEDF9P::NMEA, false))
  {
    TOBAS_FATAL("Failed to disable NMEA protocol.");
    return false;
  }
  if (!gnss_.enableProtocol(ZEDF9P::RTCM3X, false))
  {
    TOBAS_FATAL("Failed to disable RTCM3X protocol.");
    return false;
  }
  if (!gnss_.enableProtocol(ZEDF9P::SPARTN, false))
  {
    TOBAS_FATAL("Failed to disable SPARTN protocol.");
    return false;
  }

  // 不要なインターフェースを無効化
  // D_SELをオフにしているため，I2CとUARTは始めから無効化されているはず．
  if (!gnss_.enableUSB(false))
  {
    TOBAS_FATAL("Failed to disable USB interface.");
    return false;
  }

  return true;
}

void GNSSDriver::warnUnnecessaryUBXMessage()
{
  const auto cls = gnss_.latestClass();
  const auto id = gnss_.latestId();
  TOBAS_WARN("Unnecessary UBX message is received: (Class, ID) = (", (int)cls, ", ", (int)id, ")");
}

void GNSSDriver::setTimeOffsetTimerCb(const rclcpp::TimerEvent&)
{
  // Enable GPS time message only
  if (!gnss_.enableMsg(ZEDF9P::CLASS_NAV, ZEDF9P::NAV_TIMEGPS, true))
    TOBAS_EXIT("Failed to enable NAV_TIMEGPS message.");

  // Get the first GNSS message, which is expected to be NAV_TIMEGPS.
  // NAV_TIMEGPSを有効化した直後に最初に受け取ったメッセージがNAV_TIMEGPSだったら，バッファサイズは0で遅延は最小のはず．
  if (!gnss_.update())
    TOBAS_EXIT("Failed to update GNSS driver.");
  if (gnss_.latestClass() != ZEDF9P::CLASS_NAV || gnss_.latestId() != ZEDF9P::NAV_TIMEGPS)
    TOBAS_EXIT("The first message must be NAV_TIMEGPS.");

  // Decode NAV_TIMEGPS
  payload::NAV_TIMEGPS timegps;
  timegps.decode(gnss_.payload());

  // Compute the time offset
  const rclcpp::Time ros_time = node->get_clock()->now();
  const rclcpp::Time gps_time(timegps.iTOW / 1000, (timegps.iTOW % 1000) * 1'000'000 + timegps.fTOW);
  time_offset_ = ros_time - gps_time;

  // Disable GPS time message
  if (!gnss_.enableMsg(ZEDF9P::CLASS_NAV, ZEDF9P::NAV_TIMEGPS, false))
    TOBAS_EXIT("Failed to disable NAV_TIMEGPS message.");

  // Enable main messages
  if (!gnss_.enableMsg(ZEDF9P::CLASS_NAV, ZEDF9P::NAV_STATUS, true))
    TOBAS_EXIT("Failed to enable NAV_STATUS message.");
  if (!gnss_.enableMsg(ZEDF9P::CLASS_NAV, ZEDF9P::NAV_HPPOSLLH, true))
    TOBAS_EXIT("Failed to enable NAV_HPPOSLLH message.");
  if (!gnss_.enableMsg(ZEDF9P::CLASS_NAV, ZEDF9P::NAV_VELNED, true))
    TOBAS_EXIT("Failed to enable NAV_VELNED message.");
  if (!gnss_.enableMsg(ZEDF9P::CLASS_NAV, ZEDF9P::NAV_COV, true))
    TOBAS_EXIT("Failed to enable NAV_COV message.");

  // Start main timer with maximum rate
  set_time_offset_timer_.stop();
  main_timer_.start();
}

void GNSSDriver::mainTimerCb(const rclcpp::TimerEvent&)
{
  if (!gnss_.update())
  {
    TOBAS_FATAL("Failed to update GNSS driver.");
    return;
  }

  if (gnss_.latestClass() != ZEDF9P::CLASS_NAV)
  {
    warnUnnecessaryUBXMessage();
    return;
  }

  switch (gnss_.latestId())
  {
    case ZEDF9P::NAV_STATUS:
      status_.decode(gnss_.payload());
      is_received_.at(ZEDF9P::NAV_STATUS) = true;
      break;
    case ZEDF9P::NAV_HPPOSLLH:
      hpposllh_.decode(gnss_.payload());
      is_received_.at(ZEDF9P::NAV_HPPOSLLH) = true;
      break;
    case ZEDF9P::NAV_VELNED:
      velned_.decode(gnss_.payload());
      is_received_.at(ZEDF9P::NAV_VELNED) = true;
      break;
    case ZEDF9P::NAV_COV:
      cov_.decode(gnss_.payload());
      is_received_.at(ZEDF9P::NAV_COV) = true;
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

  // Create GNSS message
  const auto gnss_msg = boost::make_shared<tobas_msgs::Gps>();

  // Fill time stamp
  const auto& iTOW = hpposllh_.iTOW;  // [ms]
  const rclcpp::Time gps_time(iTOW / 1000, (iTOW % 1000) * 1'000'000);
  gnss_msg->header.stamp = gps_time + time_offset_;

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
  gnss_pub_.publish(gnss_msg);
}
}  // namespace a1
