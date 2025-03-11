#include <tobas_math/core.hpp>
#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_kdl_msgs_adapter/euler_stamped.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

using namespace std;
using namespace Eigen;

class PreArmCheckerNode : public tobas::BaseNode
{
  static constexpr auto kMainTimerPeriod = 100ms;

  static constexpr long kImuSamplingTimeThresh = 5000;          // [us]
  static constexpr double kNodeConnectionCheckTimeWindow = 5.;  // [s]
  static constexpr double kPosDriftThresh = 1.;                 // [m]
  static constexpr double kCPUTempThresh = 70.;                 // [degC]
  static constexpr double kAttitudeThresh = M_PI / 12;          // [rad]
  static constexpr double kHorPosStddevThresh = 1.;             // [m]
  static constexpr double kVerPosStddevThresh = 2.;             // [m]
  static constexpr double kVelStddevThresh = 0.3;               // [m/s]
  static constexpr double kAttiStddevThresh = M_PI / 24;        // [rad]
  static constexpr double kHeadStddevThresh = M_PI / 12;        // [rad]
  static constexpr auto kPosDriftCheckTimeWindow = 5s;

  using self = PreArmCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit PreArmCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // rosparams
  struct DoCheck
  {
    bool node_connection_unstable;
    bool battery_voltage_too_low;
    bool cpu_temperature_too_high;
    bool rotor_communication_error;
    bool attitude_too_steep;
    bool position_unstable;
    bool position_inaccurate;
    bool velocity_inaccurate;
    bool attitude_inaccurate;
    bool heading_inaccurate;
  } do_check_;

  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::msg::Cpu::ConstSharedPtr cpu_;
  tobas_msgs::msg::RotorStateArray::ConstSharedPtr rotor_states_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_kdl_msgs::EulerStamped::ConstSharedPtr euler_;

  rclcpp::Time t_last_large_interval_;
  array<tobas_std::TimestampedBufferDouble, 3> pos_bufs_;

  ros2::PublisherPtr<tobas_msgs::msg::PreArmCheck> prearm_check_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> batt_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Cpu> cpu_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Latency> sampling_time_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::EulerStamped> euler_sub_;

  ros2::TimerPtr main_timer_;

  void getStaticRosParams();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states);
  void samplingTimeCb(const tobas_msgs::msg::Latency::ConstSharedPtr& sampling_time);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler);

  void mainTimerCb();
};

PreArmCheckerNode::PreArmCheckerNode(const rclcpp::NodeOptions& options)
  : super("pre_arm_checker", options),
    pos_bufs_{ tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
               tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
               tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow) }
{
  getStaticRosParams();

  prearm_check_pub_ = createPublisher<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  batt_sub_ = createSubscriber(tobas::addThrotNS(tobas::kBatteryTopic), &self::battCb, this);
  cpu_sub_ = createSubscriber(tobas::kCpuTopic, &self::cpuCb, this);
  rotor_states_sub_ = createSubscriber(tobas::addThrotNS(tobas::kRotorStatesTopic), &self::rotorStatesCb, this);
  sampling_time_sub_ = createSubscriber(tobas::kImuSamplingTimeTopic, &self::samplingTimeCb, this);
  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  euler_sub_ = createSubscriber(tobas::addThrotNS(tobas::kEulerTopic), &self::eulerCb, this);

  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

void PreArmCheckerNode::getStaticRosParams()
{
  do_check_.node_connection_unstable = getBoolParam("check_node_connection", true);
  do_check_.battery_voltage_too_low = getBoolParam("check_battery_voltage", true);
  do_check_.cpu_temperature_too_high = getBoolParam("check_cpu_temperature", true);
  do_check_.rotor_communication_error = getBoolParam("check_rotor_communication", true);
  do_check_.attitude_too_steep = getBoolParam("check_attitude_level", true);
  do_check_.position_unstable = getBoolParam("check_position_stability", true);
  do_check_.position_inaccurate = getBoolParam("check_position_accuracy", true);
  do_check_.velocity_inaccurate = getBoolParam("check_velocity_accuracy", true);
  do_check_.attitude_inaccurate = getBoolParam("check_attitude_accuracy", true);
  do_check_.heading_inaccurate = getBoolParam("check_heading_accuracy", true);
}

void PreArmCheckerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void PreArmCheckerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // ディスアームされたらインスタンス変数を初期化
  if (!arming_ || (arming_->data && !arming->data))
  {
    battery_.reset();
    cpu_.reset();
    rotor_states_.reset();
    odom_.reset();
    euler_.reset();

    t_last_large_interval_ = get_clock()->now();

    for (auto& buf : pos_bufs_)
      buf.clear();
  }

  arming_ = arming;
}

void PreArmCheckerNode::battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  if (!arming_ || arming_->data)
    return;

  battery_ = battery;
}

void PreArmCheckerNode::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  if (!arming_ || arming_->data)
    return;

  cpu_ = cpu;
}

void PreArmCheckerNode::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states)
{
  if (!arming_ || arming_->data)
    return;

  rotor_states_ = rotor_states;
}

void PreArmCheckerNode::samplingTimeCb(const tobas_msgs::msg::Latency::ConstSharedPtr& sampling_time)
{
  if (!arming_ || arming_->data)
    return;

  // メッセージの時間差を確認
  if (do_check_.node_connection_unstable)
    if (ros2::microseconds(sampling_time->data) > kImuSamplingTimeThresh)
      t_last_large_interval_ = get_clock()->now();
}

void PreArmCheckerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (!arming_ || arming_->data)
    return;

  // 位置の履歴を保存
  if (do_check_.position_unstable)
  {
    const auto stamp = ros2::chronoFromRosTime(odom->header.stamp);
    for (size_t i = 0; i < 3; ++i)
      pos_bufs_[i].add(stamp, odom->frame.p(i));
  }

  odom_ = odom;
}

void PreArmCheckerNode::eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler)
{
  if (!arming_ || arming_->data)
    return;

  euler_ = euler;
}

void PreArmCheckerNode::mainTimerCb()
{
  if (!drone_)
    return;

  if (!arming_ || arming_->data)
    return;

  auto prearm_check = std::make_unique<tobas_msgs::msg::PreArmCheck>();

  prearm_check->header.stamp = get_clock()->now();
  prearm_check->ok = true;

  // ノード同士の接続状態
  // TODO: IMUのインターバルではなく，リアルタイムスレッドのノード接続が完了したかどうかを直接観測する．
  if (do_check_.node_connection_unstable)
  {
    if ((get_clock()->now() - t_last_large_interval_).seconds() < kNodeConnectionCheckTimeWindow)
    {
      prearm_check->node_connection_unstable = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
  }
  else
  {
    prearm_check->node_connection_unstable = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // 推進系のタイプよる場合分け
  switch (drone_->prop->type())
  {
    case tobas::propulsion_system_t::ELECTRIC:
    {
      // 未使用項目を無視
      // TODO

      const auto eprop = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone_->prop);

      // バッテリー電圧が定格電圧以上
      if (do_check_.battery_voltage_too_low && drone_->prop->type() == tobas::propulsion_system_t::ELECTRIC)
      {
        if (!battery_)
        {
          prearm_check->battery_voltage_too_low = tobas_msgs::msg::PreArmCheck::FAILED;
          prearm_check->ok = false;
        }
        else
        {
          if (battery_->voltage < eprop->battery.nominal_voltage)
          {
            prearm_check->battery_voltage_too_low = tobas_msgs::msg::PreArmCheck::FAILED;
            prearm_check->ok = false;
          }
        }
      }
      else
      {
        prearm_check->battery_voltage_too_low = tobas_msgs::msg::PreArmCheck::IGNORED;
      }

      break;
    }
    case tobas::propulsion_system_t::ICE:
    {
      // 未使用項目を無視
      prearm_check->battery_voltage_too_low = tobas_msgs::msg::PreArmCheck::IGNORED;

      const auto iprop = boost::polymorphic_pointer_downcast<tobas::ICEPropulsionSystemConfig>(drone_->prop);
      (void)iprop;  // TODO

      break;
    }
    default:
    {
      TOBAS_ERROR("Invalid propulsion system type: ", (int)drone_->prop->type());
      break;
    }
  }

  // CPU温度
  if (do_check_.cpu_temperature_too_high)
  {
    if (!cpu_)
    {
      prearm_check->cpu_temperature_too_high = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
    else
    {
      if (cpu_->temperature > kCPUTempThresh)
      {
        prearm_check->cpu_temperature_too_high = tobas_msgs::msg::PreArmCheck::FAILED;
        prearm_check->ok = false;
      }
    }
  }
  else
  {
    prearm_check->cpu_temperature_too_high = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // モータ状態
  if (do_check_.rotor_communication_error)
  {
    if (!rotor_states_)
    {
      prearm_check->rotor_communication_error = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
    else
    {
      for (const auto& state : rotor_states_->states)
      {
        if (state.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE)
        {
          prearm_check->rotor_communication_error = tobas_msgs::msg::PreArmCheck::FAILED;
          prearm_check->ok = false;
          break;
        }
      }
    }
  }
  else
  {
    prearm_check->rotor_communication_error = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // 姿勢角
  if (do_check_.attitude_too_steep)
  {
    if (!euler_)
    {
      prearm_check->attitude_too_steep = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
    else
    {
      if (max(fabs(euler_->euler.roll), fabs(euler_->euler.pitch)) > kAttitudeThresh)
      {
        prearm_check->attitude_too_steep = tobas_msgs::msg::PreArmCheck::FAILED;
        prearm_check->ok = false;
      }
    }
  }
  else
  {
    prearm_check->attitude_too_steep = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // 位置のドリフト
  if (do_check_.position_unstable)
  {
    for (const auto& buf : pos_bufs_)
    {
      if (!buf.isFilled() || buf.range() > kPosDriftThresh)
      {
        prearm_check->position_unstable = tobas_msgs::msg::PreArmCheck::FAILED;
        prearm_check->ok = false;
        break;
      }
    }
  }
  else
  {
    prearm_check->position_unstable = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // 位置推定の共分散
  if (do_check_.position_inaccurate)
  {
    if (!odom_)
    {
      prearm_check->position_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
    else
    {
      const Vector3d pos_cov_diag = odom_->position_covariance.diagonal();
      const auto hor_pos_var = max(pos_cov_diag.x(), pos_cov_diag.y());
      const auto ver_pos_var = pos_cov_diag.z();
      if (hor_pos_var > math::sqr(kHorPosStddevThresh) || ver_pos_var > math::sqr(kVerPosStddevThresh))
      {
        prearm_check->position_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
        prearm_check->ok = false;
      }
    }
  }
  else
  {
    prearm_check->position_inaccurate = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // 速度推定の共分散
  if (do_check_.velocity_inaccurate)
  {
    if (!odom_)
    {
      prearm_check->velocity_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
    else
    {
      const auto vel_var = odom_->velocity_covariance.diagonal().maxCoeff();
      if (vel_var > math::sqr(kVelStddevThresh))
      {
        prearm_check->velocity_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
        prearm_check->ok = false;
      }
    }
  }
  else
  {
    prearm_check->velocity_inaccurate = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // 姿勢推定の共分散
  if (do_check_.attitude_inaccurate)
  {
    if (!odom_)
    {
      prearm_check->attitude_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
    else
    {
      const auto atti_var = odom_->orientation_covariance.diagonal().head<2>().maxCoeff();
      if (atti_var > math::sqr(kAttiStddevThresh))
      {
        prearm_check->attitude_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
        prearm_check->ok = false;
      }
    }
  }
  else
  {
    prearm_check->attitude_inaccurate = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  // 方位推定の共分散
  if (do_check_.heading_inaccurate)
  {
    if (!odom_)
    {
      prearm_check->heading_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
      prearm_check->ok = false;
    }
    else
    {
      const auto head_var = odom_->orientation_covariance(2, 2);
      if (head_var > math::sqr(kHeadStddevThresh))
      {
        prearm_check->heading_inaccurate = tobas_msgs::msg::PreArmCheck::FAILED;
        prearm_check->ok = false;
      }
    }
  }
  else
  {
    prearm_check->heading_inaccurate = tobas_msgs::msg::PreArmCheck::IGNORED;
  }

  prearm_check_pub_->publish(move(prearm_check));
}

RCLCPP_COMPONENTS_REGISTER_NODE(PreArmCheckerNode)
