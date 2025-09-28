#include <tobas_constants/constants.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/vibration_level.hpp>

using namespace std::chrono_literals;

class HealthMonitorNode : public tobas::BaseNode
{
  static constexpr auto kMainTimerPeriod = 100ms;

  static constexpr long kImuSamplingTimeThresh = 5000;  // [us]
  static constexpr auto kRTComplianceCheckTimeWindow = 5s;
  static constexpr double kPosDriftThresh = 1.;  // [m]
  static constexpr auto kPosDriftCheckTimeWindow = 5s;
  static constexpr double kCPUTempThresh = 80.;               // [degC]
  static constexpr double kAttitudeThresh = M_PI / 12;        // [rad]
  static constexpr double kHorPosStddevThresh = 1.;           // [m]
  static constexpr double kVerPosStddevThresh = 2.;           // [m]
  static constexpr double kVelStddevThresh = 0.3;             // [m/s]
  static constexpr double kAttiStddevThresh = M_PI / 24;      // [rad]
  static constexpr double kHeadStddevThresh = M_PI / 12;      // [rad]
  static constexpr double kMagLengthErrorThresh = 0.2;        // [-]
  static constexpr double kMagDeclinationThresh = M_PI / 12;  // [rad]
  static constexpr double kVibrationLevelThresh = 10.;        // [m/s^2]

  using self = HealthMonitorNode;
  using super = tobas::BaseNode;

public:
  explicit HealthMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // rosparams
  struct DoCheck
  {
    bool rt_violation;
    bool battery_voltage_too_low;
    bool cpu_temperature_too_high;
    bool rotor_communication_error;
    bool attitude_too_steep;
    bool position_unstable;
    bool position_inaccurate;
    bool velocity_inaccurate;
    bool attitude_inaccurate;
    bool heading_inaccurate;
    bool mag_offset_too_large;
    bool mag_misalignment;
    bool vibration_too_high;
  } do_check_;

  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::msg::Cpu::ConstSharedPtr cpu_;
  tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr rotor_liv_;
  tobas_msgs::msg::Latency::ConstSharedPtr sampling_time_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::MagneticField::ConstSharedPtr mag_;
  tobas_msgs::VibrationLevel::ConstSharedPtr vibe_;

  rclcpp::Time t_last_large_interval_;
  std::array<tobas_std::TimestampedBufferDouble, 3> pos_bufs_;

  ros2::PublisherPtr<tobas_msgs::msg::VehicleHealth> health_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> batt_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Cpu> cpu_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liv_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Latency> sampling_time_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticField> mag_sub_;
  ros2::SubscriberPtr<tobas_msgs::VibrationLevel> vibe_sub_;

  ros2::TimerPtr main_timer_;

  void getStaticRosParams();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
  void rotorLivCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liv);
  void samplingTimeCb(const tobas_msgs::msg::Latency::ConstSharedPtr& sampling_time);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag);
  void vibrationLevelCb(const tobas_msgs::VibrationLevel::ConstSharedPtr& vibe);

  void mainTimerCb();
};

HealthMonitorNode::HealthMonitorNode(const rclcpp::NodeOptions& options)
  : super("health_monitor", options)
  , t_last_large_interval_(now())
  , pos_bufs_{ tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
               tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
               tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow) }
{
  getStaticRosParams();

  health_pub_ = createPublisher<tobas_msgs::msg::VehicleHealth>(tobas::kVehicleHealthTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  batt_sub_ = createSubscriber(tobas::addThrotNS(tobas::kBatteryTopic), &self::battCb, this);
  cpu_sub_ = createSubscriber(tobas::kCpuTopic, &self::cpuCb, this);
  rotor_liv_sub_ = createSubscriber(tobas::kRotorLivTopic, &self::rotorLivCb, this);
  sampling_time_sub_ = createSubscriber(tobas::kImuSamplingTimeTopic, &self::samplingTimeCb, this);
  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  mag_sub_ = createSubscriber(tobas::kMagTopic, &self::magCb, this);
  vibe_sub_ = createSubscriber(tobas::kVibrationLevelTopic, &self::vibrationLevelCb, this);

  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

void HealthMonitorNode::getStaticRosParams()
{
  do_check_.rt_violation = getBoolParam("check_realtime_compliance", true);
  do_check_.battery_voltage_too_low = getBoolParam("check_battery_voltage", true);
  do_check_.cpu_temperature_too_high = getBoolParam("check_cpu_temperature", true);
  do_check_.rotor_communication_error = getBoolParam("check_rotor_communication", true);
  do_check_.attitude_too_steep = getBoolParam("check_attitude_level", true);
  do_check_.position_unstable = getBoolParam("check_position_stability", true);
  do_check_.position_inaccurate = getBoolParam("check_position_accuracy", true);
  do_check_.velocity_inaccurate = getBoolParam("check_velocity_accuracy", true);
  do_check_.attitude_inaccurate = getBoolParam("check_attitude_accuracy", true);
  do_check_.heading_inaccurate = getBoolParam("check_heading_accuracy", true);
  do_check_.mag_offset_too_large = getBoolParam("check_mag_offset", true);
  do_check_.mag_misalignment = getBoolParam("check_mag_alignment", true);
  do_check_.vibration_too_high = getBoolParam("check_vibration_level", true);
}

void HealthMonitorNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void HealthMonitorNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // ディスアームされたら位置ドリフトチェック用のバッファを初期化
  if (!arming_ || (arming_->data && !arming->data)) {
    for (auto& buf : pos_bufs_) {
      buf.clear();
    }
  }

  arming_ = arming;
}

void HealthMonitorNode::battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void HealthMonitorNode::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  cpu_ = cpu;
}

void HealthMonitorNode::rotorLivCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liv)
{
  rotor_liv_ = rotor_liv;
}

void HealthMonitorNode::samplingTimeCb(const tobas_msgs::msg::Latency::ConstSharedPtr& sampling_time)
{
  // メッセージの時間差を確認
  if (ros2::microseconds(sampling_time->data) > kImuSamplingTimeThresh) {
    t_last_large_interval_ = now();
  }

  sampling_time_ = sampling_time;
}

void HealthMonitorNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  // アームされていなければ位置の履歴を保存
  if (arming_ && !arming_->data) {
    const auto stamp = ros2::chronoFromRosTime(odom->header.stamp);
    for (size_t i = 0; i < 3; ++i) {
      pos_bufs_[i].add(stamp, odom->frame.p(i));
    }
  }

  odom_ = odom;
}

void HealthMonitorNode::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag)
{
  mag_ = mag;
}

void HealthMonitorNode::vibrationLevelCb(const tobas_msgs::VibrationLevel::ConstSharedPtr& vibe)
{
  vibe_ = vibe;
}

void HealthMonitorNode::mainTimerCb()
{
  if (!drone_) {
    return;
  }

  if (!arming_) {
    return;
  }

  const auto cur_time = now();

  auto health = std::make_unique<tobas_msgs::msg::VehicleHealth>();

  health->header.stamp = cur_time;
  health->ok = true;

  // 制御スレッドのリアルタイム性
  // FIFOスケジューリングのシングルスレッドなのでIMUのサンプリング間隔だけで判定できる
  // 通信環境が悪くノードグラフの構築に時間がかかっている場合にリアルタイム性が落ちることがあるため確認必須
  if (do_check_.rt_violation) {
    if (sampling_time_) {
      if (cur_time - t_last_large_interval_ < kRTComplianceCheckTimeWindow) {
        health->rt_violation = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->rt_violation = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->rt_violation = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 推進系のタイプよる場合分け
  switch (drone_->prop->type()) {
    case tobas::PropulsionSystem::kElectric: {
      // 未使用項目を無視
      // TODO

      const auto eprop = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone_->prop);

      // バッテリー電圧が定格電圧以上
      if (do_check_.battery_voltage_too_low && drone_->prop->type() == tobas::PropulsionSystem::kElectric) {
        if (battery_) {
          if (battery_->voltage < eprop->battery.nominal_voltage) {
            health->battery_voltage_too_low = tobas_msgs::msg::VehicleHealth::FAILED;
            health->ok = false;
          }
        }
        else {
          health->battery_voltage_too_low = tobas_msgs::msg::VehicleHealth::UNKNOWN;
          health->ok = false;
        }
      }
      else {
        health->battery_voltage_too_low = tobas_msgs::msg::VehicleHealth::IGNORED;
      }

      break;
    }
    case tobas::PropulsionSystem::kIce: {
      // 未使用項目を無視
      health->battery_voltage_too_low = tobas_msgs::msg::VehicleHealth::IGNORED;

      const auto iprop = boost::polymorphic_pointer_downcast<tobas::IcePropulsionSystemConfig>(drone_->prop);
      (void)iprop;  // TODO

      break;
    }
    default: {
      TOBAS_ERROR("Invalid propulsion system type: ", (int)drone_->prop->type());
      break;
    }
  }

  // CPU温度
  if (do_check_.cpu_temperature_too_high) {
    if (cpu_) {
      if (cpu_->temperature > kCPUTempThresh) {
        health->cpu_temperature_too_high = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->cpu_temperature_too_high = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->cpu_temperature_too_high = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // モータ状態
  if (do_check_.rotor_communication_error) {
    if (rotor_liv_) {
      for (const auto& elem : rotor_liv_->data) {
        if (!elem.alive) {
          health->rotor_communication_error = tobas_msgs::msg::VehicleHealth::FAILED;
          health->ok = false;
          break;
        }
      }
    }
    else {
      health->rotor_communication_error = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->rotor_communication_error = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 姿勢角
  if (do_check_.attitude_too_steep && !arming_->data) {
    if (odom_) {
      const auto [roll, pitch, _] = odom_->frame.M.getRPY();
      if (std::max(fabs(roll), fabs(pitch)) > kAttitudeThresh) {
        health->attitude_too_steep = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->attitude_too_steep = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->attitude_too_steep = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 位置のドリフト
  if (do_check_.position_unstable && !arming_->data) {
    if (odom_) {
      for (const auto& buf : pos_bufs_) {
        if (!buf.isFilled() || buf.range() > kPosDriftThresh) {
          health->position_unstable = tobas_msgs::msg::VehicleHealth::FAILED;
          health->ok = false;
          break;
        }
      }
    }
    else {
      health->position_unstable = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->position_unstable = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 位置推定の確かさ
  if (do_check_.position_inaccurate) {
    if (odom_) {
      const auto pos_cov_diag = odom_->position_covariance.diagonal().eval();
      const auto hor_pos_var = std::max(pos_cov_diag.x(), pos_cov_diag.y());
      const auto ver_pos_var = pos_cov_diag.z();
      if (hor_pos_var > math::sqr(kHorPosStddevThresh) || ver_pos_var > math::sqr(kVerPosStddevThresh)) {
        health->position_inaccurate = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->position_inaccurate = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->position_inaccurate = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 速度推定の確かさ
  if (do_check_.velocity_inaccurate) {
    if (odom_) {
      const auto vel_var = odom_->velocity_covariance.diagonal().maxCoeff();
      if (vel_var > math::sqr(kVelStddevThresh)) {
        health->velocity_inaccurate = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->velocity_inaccurate = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->velocity_inaccurate = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 姿勢推定の確かさ
  if (do_check_.attitude_inaccurate) {
    if (odom_) {
      const auto atti_var = odom_->orientation_covariance.diagonal().head<2>().maxCoeff();
      if (atti_var > math::sqr(kAttiStddevThresh)) {
        health->attitude_inaccurate = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->attitude_inaccurate = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->attitude_inaccurate = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 方位推定の確かさ
  if (do_check_.heading_inaccurate) {
    if (odom_) {
      const auto head_var = odom_->orientation_covariance(2, 2);
      if (head_var > math::sqr(kHeadStddevThresh)) {
        health->heading_inaccurate = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->heading_inaccurate = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->heading_inaccurate = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 地磁気オフセット
  if (do_check_.mag_offset_too_large) {
    if (mag_) {
      if (abs(mag_->mag.norm() - 1.) > kMagLengthErrorThresh) {
        health->mag_offset_too_large = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->mag_offset_too_large = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->mag_offset_too_large = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 世界座標系から見た地磁気ベクトルが参照と一致するか
  if (do_check_.mag_misalignment) {
    if (mag_) {
      // TODO: ESKFから参照地磁気ベクトルを発行して評価
    }
    else {
      health->mag_misalignment = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->mag_misalignment = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // 振動レベル
  if (do_check_.vibration_too_high) {
    if (vibe_) {
      // Vibration levels below 30m/s/s are normally acceptable
      // cf. https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html#vibrations
      if (vibe_->data.max() > kVibrationLevelThresh) {
        health->vibration_too_high = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->vibration_too_high = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->vibration_too_high = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  health_pub_->publish(std::move(health));
}

RCLCPP_COMPONENTS_REGISTER_NODE(HealthMonitorNode)
