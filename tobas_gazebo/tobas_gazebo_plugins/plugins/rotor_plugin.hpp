#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_tools/turning_direction.hpp>
#include <tobas_tools/esc.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/Wind.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/first_order_filter.hpp"

namespace gazebo
{
/* Simulates ESC, rotor and ropeller. */
class GazeboRotorPlugin : public ModelPlugin
{
  // Constants
  static constexpr char kDebugTopicPrefix[] = "ground_truth/rotor_debug";
  static constexpr double kRotorSpeedCheckMargin = 10.;     // [rad/s]
  static constexpr double kCommandBlankTimeThreshold = 1.;  // [s]
  static constexpr double kTimeConstWarnThreshold = 0.1;    // [s]
  static constexpr double kMinBatteryVoltage = 3.;          // [V]
  static constexpr double kDisarmDuration = 1.5;            // [s] 通常1~2秒らしい

  // Default parameters
  static constexpr double kDefaultMaxModelErrorRate = 0.;

  using self = GazeboRotorPlugin;


public:
  explicit GazeboRotorPlugin();

protected:
  void Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&) override;

private:


  // SDF parameters
  size_t motor_number_;
  std::string link_name_;
  std::string joint_name_;
  math::Vector2d rot_speed_coefs_;  // [Vs/rad, (Vs/rad)^2]
  double motor_const_;
  double moment_const_;
  double rotor_drag_coef_;
  tobas::TurningDirection direction_;  // Turning direction: 1(CCW) or -1(CW).
  double time_const_up_;
  double time_const_down_;
  double max_rot_speed_;  // [rad/s] 最大連続電流によって定まるモータ特性が成り立つ最大回転数
  size_t num_poles_;      // モータの極数
  double max_current_;    // [A] ESCの最大電流
  tobas::EscMode esc_mode_;  // ESCへの信号の解釈方式
  double max_model_error_rate_;

  double cmd_rot_speed_ = 0.;  // [rad/s]
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  gz::math::Vector3d wind_vel_W_ = math::Vector3d::Zero;  // [m/s]
  chrono::steady_clock::duration prev_sim_time_;
  chrono::steady_clock::duration last_cmd_time_;  // 最後にスロットルコマンドが指令された時刻
  chrono::steady_clock::duration disarm_start_time_ = chrono::steady_clock::duration::Maximum();  // Disarmコマンドの開始時刻
  bool is_intact_ = true;
  bool is_armed_ = false;
  bool wind_received_ = false;
  AsymmetricFirstOrderFilter<double> rotor_speed_filter_;

  // Gazebo objects
  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;


  // PubSub
  PublisherPtr<> rotor_state_pub_;
  PublisherPtr<> debug_pub_;
  SubscriberPtr<> throttle_sub_;
  SubscriberPtr<> battery_gt_sub_;
  SubscriberPtr<> wind_gt_sub_;

  void getSdfParams(const sdf::ElementPtr& sdf);

  void registerPubSub();
  bool isReady();
  void addModelError();
  void applyForceAndTorque(const double& rot_speed, const chrono::steady_clock::duration& cur_time);
  void updateRotationSpeed(const double& dt);
  double rotSpeedFromVoltage(const double& voltage);
  double rotSpeedFromERPM(const double& erpm);

  void throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& msg);
  void batteryGtCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind);
};
}  // namespace gazebo
