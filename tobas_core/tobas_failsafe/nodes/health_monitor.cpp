// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
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
#include <tobas_msgs/msg/user_defined_health_status.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_msgs_adapter/vibration_level.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class HealthMonitorNode : public BaseNode
{
  static constexpr auto kMainTimerPeriod = 100ms;

  static constexpr auto kImuSamplingTimeThresh = 5ms;
  static constexpr auto kRTComplianceCheckTimeWindow = 1s;
  static constexpr auto kBattVoltageDownTimeThresh = 10s;
  static constexpr auto kBattVoltageUpTimeThresh = 30s;
  static constexpr auto kRadioLinkLostTimeThresh = 500ms;
  static constexpr double kPosDriftThresh = 1.0;  // [m]
  static constexpr auto kPosDriftCheckTimeWindow = 5s;
  static constexpr double kCPUTempThresh = 80.0;             // [degC]
  static constexpr double kAttitudeThresh = M_PI / 12;       // [rad]
  static constexpr double kHorPosStddevThresh = 1.0;         // [m]
  static constexpr double kVerPosStddevThresh = 2.0;         // [m]
  static constexpr double kVelStddevThresh = 0.3;            // [m/s]
  static constexpr double kAttiStddevThresh = M_PI / 24;     // [rad]
  static constexpr double kHeadStddevThresh = M_PI / 12;     // [rad]
  static constexpr double kMagLpfCutoff = 1.0;               // [s]
  static constexpr double kMagLengthErrorThresh = 0.2;       // [-]
  static constexpr double kMagAlignErrorThresh = M_PI / 12;  // [rad]
  static constexpr double kVibrationLevelThresh = 10.0;      // [m/s^2]

  using self = HealthMonitorNode;
  using super = BaseNode;

public:
  explicit HealthMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ROS parameters.
  struct DoCheck
  {
    bool realtime_compliance;
    bool battery_voltage;
    bool cpu_temperature;
    bool radio_link;
    bool rotor_links;
    bool attitude_level;
    bool position_stability;
    bool position_accuracy;
    bool velocity_accuracy;
    bool attitude_accuracy;
    bool heading_accuracy;
    bool mag_offset;
    bool mag_alignment;
    bool vibration_level;
    bool user_defined_condition;
  } do_check_;

  Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::msg::Cpu::ConstSharedPtr cpu_;
  tobas_msgs::RCInput::ConstSharedPtr rcin_;
  tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr rotor_liv_;
  tobas_msgs::msg::Latency::ConstSharedPtr sampling_time_;
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_;
  tobas_msgs::MagneticField::ConstSharedPtr mag_;
  tobas_msgs::MagneticField::ConstSharedPtr mag_ref_;
  tobas_msgs::VibrationLevel::ConstSharedPtr vibe_;
  tobas_msgs::msg::UserDefinedHealthStatus::ConstSharedPtr user_health_;

  rclcpp::Time t_last_rt_violation_;
  bool batt_voltage_ok_ = true;
  builtin_interfaces::msg::Time t_last_voltage_ok_, t_last_voltage_ng_;
  rclcpp::Time t_last_valid_rcin_;
  std::array<st::TimestampedBufferDouble, 3> pos_bufs_;
  dsp::LowPassFilter<kdl::Vector> mag_B_lpf_, mag_W_lpf_;

  bool last_ok_ = false;

  ros2::PublisherPtr<tobas_msgs::msg::VehicleHealth> health_pub_;

  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> batt_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Cpu> cpu_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liv_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Latency> sampling_time_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticField> mag_sub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticField> mag_ref_sub_;
  ros2::SubscriberPtr<tobas_msgs::VibrationLevel> vibe_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::UserDefinedHealthStatus> user_health_sub_;

  ros2::TimerPtr main_timer_;

  void getStaticRosParams();
  std::unique_ptr<tobas_msgs::msg::VehicleHealth> createHealthMessage() const;

  void droneCb(const Drone::ConstSharedPtr& drone);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
  void rotorLivCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liv);
  void samplingTimeCb(const tobas_msgs::msg::Latency::ConstSharedPtr& sampling_time);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag);
  void magRefCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_ref);
  void vibrationLevelCb(const tobas_msgs::VibrationLevel::ConstSharedPtr& vibe);
  void userDefinedHealthStatusCb(const tobas_msgs::msg::UserDefinedHealthStatus::ConstSharedPtr& user_health);

  void mainTimerCb();
};

HealthMonitorNode::HealthMonitorNode(const rclcpp::NodeOptions& options)
  : super("health_monitor", nodeOptions_Default(options))
  , t_last_rt_violation_(now())
  , pos_bufs_{ st::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
               st::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
               st::TimestampedBufferDouble(kPosDriftCheckTimeWindow) }
{
  getStaticRosParams();

  mag_B_lpf_.setCutoffFrequency(kMagLpfCutoff);
  mag_W_lpf_.setCutoffFrequency(kMagLpfCutoff);

  health_pub_ = createPublisher<tobas_msgs::msg::VehicleHealth>(topic::kVehicleHealth);

  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  batt_sub_ = createSubscriber(addThrotNS(topic::kBattery), &self::battCb, this);
  cpu_sub_ = createSubscriber(topic::kCpu, &self::cpuCb, this);
  rcin_sub_ = createSubscriber(topic::kRcInput, &self::rcInputCb, this);
  rotor_liv_sub_ = createSubscriber(topic::kRotorLiv, &self::rotorLivCb, this);
  sampling_time_sub_ = createSubscriber(topic::kImuSamplingTime, &self::samplingTimeCb, this);
  odom_sub_ = createSubscriber(addThrotNS(topic::kOdometry), &self::odomCb, this);
  mag_sub_ = createSubscriber(topic::kMagneticField, &self::magCb, this);
  mag_ref_sub_ = createSubscriber(topic::kMagRef, &self::magRefCb, this);
  vibe_sub_ = createSubscriber(topic::kVibrationLevel, &self::vibrationLevelCb, this);

  if (do_check_.user_defined_condition) {
    user_health_sub_ = createSubscriber(topic::kUserDefinedHealthStatus, &self::userDefinedHealthStatusCb, this);
  }

  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

void HealthMonitorNode::getStaticRosParams()
{
  do_check_.realtime_compliance = getBoolParam("check_realtime_compliance");
  do_check_.battery_voltage = getBoolParam("check_battery_voltage");
  do_check_.cpu_temperature = getBoolParam("check_cpu_temperature");
  do_check_.radio_link = getBoolParam("check_radio_link");
  do_check_.rotor_links = getBoolParam("check_rotor_links");
  do_check_.attitude_level = getBoolParam("check_attitude_level");
  do_check_.position_stability = getBoolParam("check_position_stability");
  do_check_.position_accuracy = getBoolParam("check_position_accuracy");
  do_check_.velocity_accuracy = getBoolParam("check_velocity_accuracy");
  do_check_.attitude_accuracy = getBoolParam("check_attitude_accuracy");
  do_check_.heading_accuracy = getBoolParam("check_heading_accuracy");
  do_check_.mag_offset = getBoolParam("check_mag_offset");
  do_check_.mag_alignment = getBoolParam("check_mag_alignment");
  do_check_.vibration_level = getBoolParam("check_vibration_level");
  do_check_.user_defined_condition = getBoolParam("check_user_defined_condition");
}

std::unique_ptr<tobas_msgs::msg::VehicleHealth> HealthMonitorNode::createHealthMessage() const
{
  const auto cur_time = now();

  auto health = std::make_unique<tobas_msgs::msg::VehicleHealth>();

  health->header.stamp = cur_time;
  health->ok = true;

  // Real-time compliance of the control thread.
  // Since this is a single FIFO-scheduled thread, checking only the IMU sampling interval is enough.
  // This must be checked because poor communication can slow node graph construction and degrade real-time behavior.
  if (do_check_.realtime_compliance) {
    if (sampling_time_) {
      if (cur_time - t_last_rt_violation_ < kRTComplianceCheckTimeWindow) {
        health->realtime_compliance = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->realtime_compliance = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->realtime_compliance = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Branch by propulsion system type.
  switch (drone_->prop->type()) {
    case PropulsionSystem::kElectric: {
      const auto eprop = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone_->prop);

      // Battery voltage is above the threshold.
      if (do_check_.battery_voltage) {
        if (battery_) {
          if (!batt_voltage_ok_) {
            health->battery_voltage = tobas_msgs::msg::VehicleHealth::FAILED;
            health->ok = false;
          }
        }
        else {
          health->battery_voltage = tobas_msgs::msg::VehicleHealth::UNKNOWN;
          health->ok = false;
        }
      }
      else {
        health->battery_voltage = tobas_msgs::msg::VehicleHealth::IGNORED;
      }

      break;
    }
    case PropulsionSystem::kIce: {
      // Ignore unused item.
      health->battery_voltage = tobas_msgs::msg::VehicleHealth::IGNORED;

      const auto iprop = boost::polymorphic_pointer_downcast<IcePropulsionSystemConfig>(drone_->prop);
      (void)iprop;  // TODO

      break;
    }
    default: {
      TOBAS_ERROR("Invalid propulsion system type: ", (int)drone_->prop->type());
      break;
    }
  }

  // CPU temperature
  if (do_check_.cpu_temperature) {
    if (cpu_) {
      if (cpu_->temperature > kCPUTempThresh) {
        health->cpu_temperature = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->cpu_temperature = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->cpu_temperature = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Communication between RC transmitter and receiver
  if (do_check_.radio_link) {
    const auto frame_lost = rcin_ && (rcin_->status == tobas_msgs::msg::RCInput::STATUS_FRAME_LOST);
    if (frame_lost || cur_time - t_last_valid_rcin_ > kRadioLinkLostTimeThresh) {
      health->radio_link = tobas_msgs::msg::VehicleHealth::FAILED;
      health->ok = false;
    }
  }
  else {
    health->radio_link = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Rotor state
  if (do_check_.rotor_links) {
    if (rotor_liv_) {
      for (const auto& elem : rotor_liv_->data) {
        if (!elem.alive) {
          health->rotor_links = tobas_msgs::msg::VehicleHealth::FAILED;
          health->ok = false;
          break;
        }
      }
    }
    else {
      health->rotor_links = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->rotor_links = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Attitude angle
  if (do_check_.attitude_level && !arming_->data) {
    if (odom_) {
      const auto [roll, pitch, _] = odom_->odom.odom.frame.M.getRPY();
      if (std::max(std::abs(roll), std::abs(pitch)) > kAttitudeThresh) {
        health->attitude_level = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->attitude_level = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->attitude_level = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Position drift
  if (do_check_.position_stability && !arming_->data) {
    if (odom_) {
      for (const auto& buf : pos_bufs_) {
        if (!buf.isFilled() || buf.range() > kPosDriftThresh) {
          health->position_stability = tobas_msgs::msg::VehicleHealth::FAILED;
          health->ok = false;
          break;
        }
      }
    }
    else {
      health->position_stability = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->position_stability = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Position estimation accuracy
  if (do_check_.position_accuracy) {
    if (odom_) {
      const auto pos_cov_diag = odom_->odom.position_covariance.diagonal().eval();
      const auto hor_pos_var = std::max(pos_cov_diag.x(), pos_cov_diag.y());
      const auto ver_pos_var = pos_cov_diag.z();
      if (hor_pos_var > math::sqr(kHorPosStddevThresh) || ver_pos_var > math::sqr(kVerPosStddevThresh)) {
        health->position_accuracy = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->position_accuracy = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->position_accuracy = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Velocity estimation accuracy
  if (do_check_.velocity_accuracy) {
    if (odom_) {
      const auto vel_var = odom_->odom.velocity_covariance.diagonal().maxCoeff();
      if (vel_var > math::sqr(kVelStddevThresh)) {
        health->velocity_accuracy = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->velocity_accuracy = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->velocity_accuracy = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Attitude estimation accuracy
  if (do_check_.attitude_accuracy) {
    if (odom_) {
      const auto atti_var = odom_->odom.orientation_covariance.diagonal().head<2>().maxCoeff();
      if (atti_var > math::sqr(kAttiStddevThresh)) {
        health->attitude_accuracy = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->attitude_accuracy = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->attitude_accuracy = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Heading estimation accuracy
  if (do_check_.heading_accuracy) {
    if (odom_) {
      const auto head_var = odom_->odom.orientation_covariance(2, 2);
      if (head_var > math::sqr(kHeadStddevThresh)) {
        health->heading_accuracy = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->heading_accuracy = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->heading_accuracy = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Magnetic field offset
  if (do_check_.mag_offset) {
    if (mag_) {
      if (std::abs(mag_B_lpf_.getValue().norm() - 1.0) > kMagLengthErrorThresh) {
        health->mag_offset = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->mag_offset = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->mag_offset = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Whether the magnetic field vector in the world frame matches the reference
  if (do_check_.mag_alignment) {
    if (mag_ && mag_ref_) {
      const auto align_error = mag_W_lpf_.getValue().argument(mag_ref_->mag);  // [rad]
      if (align_error > kMagAlignErrorThresh) {
        health->mag_alignment = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->mag_alignment = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->mag_alignment = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Vibration level
  if (do_check_.vibration_level) {
    if (vibe_) {
      // Vibration levels below 30m/s/s are normally acceptable.
      // cf. https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html#vibrations
      if (vibe_->data.max() > kVibrationLevelThresh) {
        health->vibration_level = tobas_msgs::msg::VehicleHealth::FAILED;
        health->ok = false;
      }
    }
    else {
      health->vibration_level = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->vibration_level = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  // Custom check item
  if (do_check_.user_defined_condition) {
    if (user_health_) {
      health->user_defined_condition = user_health_->data;
      if (user_health_->data != tobas_msgs::msg::VehicleHealth::PASSED) {
        health->ok = false;
      }
    }
    else {
      health->user_defined_condition = tobas_msgs::msg::VehicleHealth::UNKNOWN;
      health->ok = false;
    }
  }
  else {
    health->user_defined_condition = tobas_msgs::msg::VehicleHealth::IGNORED;
  }

  return health;
}

void HealthMonitorNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void HealthMonitorNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // Clear `pos_bufs_` for position drift checks when disarmed.
  if (!arming_ || (arming_->data && !arming->data)) {
    for (auto& buf : pos_bufs_) {
      buf.clear();
    }
  }

  arming_ = arming;
}

void HealthMonitorNode::battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  if (!drone_) {
    return;
  }

  const auto eprop = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone_->prop);
  if (!eprop) {
    return;
  }

  // Calculate voltage compensated for the drop caused by internal resistance.
  const auto voltage = battery->voltage + eprop->battery.internal_resistance * battery->current;

  // Determine the initial state.
  if (!battery_) {
    battery_ = battery;
    batt_voltage_ok_ = voltage > eprop->battery.nominal_voltage;
    return;
  }

  // Update the latest battery state.
  battery_ = battery;

  // Switch state robustly using hysteresis and a time window.
  const auto& cur_time = battery->header.stamp;
  if (batt_voltage_ok_) {
    // Update the last time the voltage was above the threshold.
    if (voltage > eprop->battery.sag_voltage) {
      t_last_voltage_ok_ = cur_time;
    }

    // Switch state if the voltage stays below the threshold for a fixed time.
    if (cur_time - t_last_voltage_ok_ > kBattVoltageDownTimeThresh) {
      batt_voltage_ok_ = false;
      t_last_voltage_ng_ = cur_time;
    }
  }
  else {
    // Update the last time the voltage was below the threshold.
    if (voltage < eprop->battery.nominal_voltage) {
      t_last_voltage_ng_ = cur_time;
    }

    // Switch state if the voltage stays above the threshold for a fixed time.
    // Usually the vehicle does not naturally recover from low voltage, so this transition should be conservative.
    if (cur_time - t_last_voltage_ng_ > kBattVoltageUpTimeThresh) {
      batt_voltage_ok_ = true;
      t_last_voltage_ok_ = cur_time;
    }
  }
}

void HealthMonitorNode::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  cpu_ = cpu;
}

void HealthMonitorNode::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  if (rcin->status == tobas_msgs::msg::RCInput::STATUS_OK) {
    t_last_valid_rcin_ = rclcpp::Time(rcin->header.stamp, get_clock()->get_clock_type());
  }

  rcin_ = rcin;
}

void HealthMonitorNode::rotorLivCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liv)
{
  rotor_liv_ = rotor_liv;
}

void HealthMonitorNode::samplingTimeCb(const tobas_msgs::msg::Latency::ConstSharedPtr& sampling_time)
{
  // Check message time difference.
  if (sampling_time->data > kImuSamplingTimeThresh) {
    t_last_rt_violation_ = rclcpp::Time(sampling_time->header.stamp, get_clock()->get_clock_type());
  }

  sampling_time_ = sampling_time;
}

void HealthMonitorNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  // Store position history while the vehicle is not armed.
  if (arming_ && !arming_->data) {
    const auto stamp = ros2::chronoFromRosTime(odom->header.stamp);
    for (size_t i = 0; i < 3; ++i) {
      pos_bufs_[i].add(stamp, odom->odom.odom.frame.p(i));
    }
  }

  odom_ = odom;
}

void HealthMonitorNode::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag)
{
  if (!odom_) {
    return;
  }

  const auto& mag_B = mag->mag;

  // Magnetic field vector in the world frame, ideally an invariant vector.
  // Convert to the world frame before passing through the LPF to avoid attitude-magnetic-field time offset.
  const auto mag_W = odom_->odom.odom.frame.M * mag_B;

  if (!mag_) {
    mag_B_lpf_.setValue(mag_B);
    mag_W_lpf_.setValue(mag_W);

    mag_ = mag;
    return;
  }

  const auto dt = (mag->header.stamp - mag_->header.stamp).seconds();
  mag_ = mag;

  mag_B_lpf_.update(mag_B, dt);
  mag_W_lpf_.update(mag_W, dt);
}

void HealthMonitorNode::magRefCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_ref)
{
  mag_ref_ = mag_ref;
}

void HealthMonitorNode::vibrationLevelCb(const tobas_msgs::VibrationLevel::ConstSharedPtr& vibe)
{
  vibe_ = vibe;
}

void HealthMonitorNode::userDefinedHealthStatusCb(
  const tobas_msgs::msg::UserDefinedHealthStatus::ConstSharedPtr& user_health)
{
  user_health_ = user_health;
}

void HealthMonitorNode::mainTimerCb()
{
  if (!drone_) {
    return;
  }

  if (!arming_) {
    return;
  }

  auto health = createHealthMessage();

  if (!arming_->data && !last_ok_ && health->ok) {
    TOBAS_INFO("The vehicle is ready to arm.");
  }
  last_ok_ = health->ok;

  health_pub_->publish(std::move(health));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::HealthMonitorNode)
