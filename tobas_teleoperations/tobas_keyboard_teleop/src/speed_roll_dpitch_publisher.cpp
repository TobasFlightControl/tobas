#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_keyboard/utils.hpp>
#include <tobas_ros2_tools/rosparam.hpp>
#include <tobas_ros2_tools/rate.hpp>

#include <tobas_constants/constants.hpp>

#include "../include/tobas_keyboard_teleop/speed_roll_dpitch_publisher.hpp"
#include "../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
SpeedRollDeltaPitchPublisher::SpeedRollDeltaPitchPublisher(
  rclcpp::Node::SharedPtr node,
  rclcpp::Node::SharedPtr pnh,
  const string& name)
  : super(node, pnh, name),
    trim_(drone_),
    check_topics_timer_(node_, tobas::kCheckTopicsMsgPeriod, &self::checkTopicsTimerCb, this, false),
    instruction_timer_(node_, kInstructionTimerPeriod, &self::instructionTimerCb, this, false)
{
  instruction_ = "Control your drone!\n"
                 "---------------------------\n"
                 "W/S       : Increase/Decrease speed\n"
                 "Up/Down   : Nose up/down\n"
                 "Left/Right: Turn left/right\n"
                 "Ctrl-C    : Quit\n";

  getRosParams();
  drone_.loadFromParam(node_);

  trim_.updateInternalDataStructures();
  q_0_ = kdl::JntArray::Zero(tree_.getNrOfJoints());

  const auto repeat_interval = keyboard::getKeyboardRepeatInterval();
  delta_speed_ = max_linacc_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  cmd_pub_ = node_.advertise<tobas_msgs::msg::SpeedRollDeltaPitch>(tobas::kSpeedRollDpitchCmdTopic, 1);
  air_pressure_sub_ = node_.subscribe(tobas::kAirPressureTopic, 1, &self::airPressureCb, this, tcpNoDelay());
}

void SpeedRollDeltaPitchPublisher::run()
{
  // TODO: 離陸アクションを呼ぶ or Arming
  // TODO: 終了時にDisarming

  check_topics_timer_.start();

  rclcpp::Rate rate(kUpdateRate);
  while (node_.ok())
  {
    if (!is_initialized_)
    {
      if (isReady())
      {
        check_topics_timer_.stop();
        initialize();
        is_initialized_ = true;
      }
      rclcpp::spinOnce();
      rate.sleep();
      continue;
    }

    if (trim_.update(cmd_.speed, air_density_, q_0_) < 0)
    {
      RCLCPP_ERROR_STREAM(trim_.errorMessage());
      continue;
    }

    // コマンドを更新
    const auto c = key_reader_.readKey();
    if (c < 0)
      RCLCPP_ERROR_STREAM("Failed to read keyboard.");

    switch (c)
    {
      case 'w':
      {
        cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed + delta_speed_);
        RCLCPP_INFO_STREAM("Increase speed");
        break;
      }
      case 's':
      {
        cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed - delta_speed_);
        RCLCPP_INFO_STREAM("Decrease speed");
        break;
      }
      case keyboard::UP:
      {
        cmd_.delta_pitch = clamp(cmd_.delta_pitch - delta_rot_, -max_delta_pitch_, max_delta_pitch_);
        RCLCPP_INFO_STREAM("Nose up");
        break;
      }
      case keyboard::DOWN:
      {
        cmd_.delta_pitch = clamp(cmd_.delta_pitch + delta_rot_, -max_delta_pitch_, max_delta_pitch_);
        RCLCPP_INFO_STREAM("Nose down");
        break;
      }
      case keyboard::LEFT:
      {
        cmd_.roll = clamp(cmd_.roll - delta_rot_, -max_roll_, max_roll_);
        RCLCPP_INFO_STREAM("Turn left");
        break;
      }
      case keyboard::RIGHT:
      {
        cmd_.roll = clamp(cmd_.roll + delta_rot_, -max_roll_, max_roll_);
        RCLCPP_INFO_STREAM("Turn right");
        break;
      }
    }

    // コマンドを発行
    const auto cmd_ptr = make_unique<tobas_msgs::msg::SpeedRollDeltaPitch>(cmd_);
    cmd_pub_.publish(cmd_ptr);

    rclcpp::spinOnce();
    rate.sleep();
  }
}

void SpeedRollDeltaPitchPublisher::getRosParams()
{
  ros2::getParam(pnh_, "max_linear_acceleration", max_linacc_, kDefaultMaxLinearAcceleration, ros2::POSITIVE);
  ros2::getParam(pnh_, "max_angular_velocity", max_angvel_, kDefaultMaxAngularVelocity, ros2::POSITIVE);
  ros2::getParam(pnh_, "maximum_roll", max_roll_, kDefaultMaximumRoll, ros2::POSITIVE);
  ros2::getParam(pnh_, "maximum_delta_pitch", max_delta_pitch_, kDefaultMaximumDeltaPitch, ros2::POSITIVE);
}

bool SpeedRollDeltaPitchPublisher::isReady()
{
  return pressure_received_;
}

void SpeedRollDeltaPitchPublisher::initialize()
{
  cmd_.speed = trim_.takeOffSpeed(air_density_);
  cmd_.roll = 0.;
  cmd_.delta_pitch = 0.;

  // インストラクションを開始
  instruction_timer_.start();
  RCLCPP_INFO_STREAM(instruction_);
}

void SpeedRollDeltaPitchPublisher::airPressureCb(const sensor_msgs::msg::FluidPressureConstPtr& msg)
{
  air_density_ = tobas_std::pressureToDensity(msg->fluid_pressure);

  if (!pressure_received_)
    pressure_received_ = true;
}

void SpeedRollDeltaPitchPublisher::checkTopicsTimerCb(const rclcpp::TimerEvent&)
{
  if (!pressure_received_)
    RCLCPP_INFO_STREAM("Waiting for " << ns() << tobas::kAirPressureTopic);
}

void SpeedRollDeltaPitchPublisher::instructionTimerCb(const rclcpp::TimerEvent&)
{
  RCLCPP_INFO_STREAM(instruction_);
}
}  // namespace tobas_keyboard_teleop
