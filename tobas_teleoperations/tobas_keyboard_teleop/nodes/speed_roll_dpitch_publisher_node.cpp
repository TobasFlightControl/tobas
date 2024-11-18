#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_keyboard/utils.hpp>
#include <tobas_keyboard/keyboard_reader.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_tools/fw_trim_conditions.hpp>

#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>
#include <tobas_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs/msg/fluid_pressure_with_variance_stamped.hpp>

#include "../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class SpeedRollDeltaPitchPublisherNode : public tobas::BaseNode
{
  static constexpr double kDefaultMaxLinearAcceleration = 2.;
  static constexpr double kDefaultMaxAngularVelocity = M_PI_2;
  static constexpr double kDefaultMaximumRoll = M_PI_2;
  static constexpr double kDefaultMaximumDeltaPitch = M_PI_4;

  static constexpr char kInstruction[] = "Control your drone!\n"
                                         "---------------------------\n"
                                         "W/S       : Increase/Decrease speed\n"
                                         "Up/Down   : Nose up/down\n"
                                         "Left/Right: Turn left/right\n"
                                         "Ctrl-C    : Quit\n";

  using self = SpeedRollDeltaPitchPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit SpeedRollDeltaPitchPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  kdl::Tree tree_;

  tobas::TrimConditions trim_;
  keyboard::KeyboardReader key_reader_;

  // 固定値
  kdl::JntArray q_0_;
  double delta_speed_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;    // 1度のキーボード入力での回転位置の変化量

  // 可変値
  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool pressure_received_ = false;
  double air_density_;  // 現在の大気密度
  tobas_msgs::msg::SpeedRollDeltaPitch cmd_;

  // rosparams
  double max_linacc_;       // 並進加速度の大きさの最大値
  double max_angvel_;       // 回転速度の大きさの最大値
  double max_roll_;         // ロール角の最大値
  double max_delta_pitch_;  // ピッチ角の釣り合いからの偏差の最大値

  // PubSub
  ros2::PublisherPtr<tobas_msgs::msg::SpeedRollDeltaPitch> cmd_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressureWithVarianceStamped> air_pressure_sub_;

  // Timer
  ros2::TimerPtr process_timer_;
  ros2::TimerPtr check_topics_timer_;
  ros2::TimerPtr instruction_timer_;

  void getStaticRosParams();
  void initialize();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void airPressureCb(const tobas_msgs::msg::FluidPressureWithVarianceStamped::ConstSharedPtr& msg);

  void mainTimerCb();
  void checkTopicsTimerCb();
  void instructionTimerCb();
};

SpeedRollDeltaPitchPublisherNode::SpeedRollDeltaPitchPublisherNode(const rclcpp::NodeOptions& options)
  : super("speed_roll_dpitch_publisher", options), trim_(drone_, tree_)
{
  getStaticRosParams();

  const auto repeat_interval = keyboard::getKeyboardRepeatInterval();
  delta_speed_ = max_linacc_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  cmd_pub_ = createPublisher<tobas_msgs::msg::SpeedRollDeltaPitch>(tobas::kSpeedRollDpitchCmdTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true);
  air_pressure_sub_ = createSubscriber(tobas::kAirPressureTopic, &self::airPressureCb, this);

  process_timer_ = createTimer(kCommandPeriod, &self::mainTimerCb, this);
  check_topics_timer_ = createTimer(kCheckTopicsTimerPeriod, &self::checkTopicsTimerCb, this);

  // TODO: メインプロセスを起動する前に離陸アクションを呼ぶ or Arming
}

void SpeedRollDeltaPitchPublisherNode::getStaticRosParams()
{
  max_linacc_ = getDoubleParam("max_linear_acceleration", kDefaultMaxLinearAcceleration);
  max_angvel_ = getDoubleParam("max_angular_velocity", kDefaultMaxAngularVelocity);
  max_roll_ = getDoubleParam("maximum_roll", kDefaultMaximumRoll);
  max_delta_pitch_ = getDoubleParam("maximum_delta_pitch", kDefaultMaximumDeltaPitch);

  TOBAS_ASSERT(max_linacc_ > 0);
  TOBAS_ASSERT(max_angvel_ > 0);
  TOBAS_ASSERT(max_roll_ > 0);
  TOBAS_ASSERT(max_delta_pitch_ > 0);
}

void SpeedRollDeltaPitchPublisherNode::initialize()
{
  trim_.updateInternalDataStructures();
  q_0_ = kdl::JntArray::Zero(tree_.getNrOfJoints());

  cmd_.speed = trim_.takeOffSpeed(air_density_);
  cmd_.roll = 0.;
  cmd_.delta_pitch = 0.;

  // インストラクションの表示を開始
  cout << kInstruction << endl;
  instruction_timer_ = createTimer(kInstructionTimerPeriod, &self::instructionTimerCb, this);
}

void SpeedRollDeltaPitchPublisherNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;
  drone_received_ = true;
}

void SpeedRollDeltaPitchPublisherNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  tree_received_ = true;
}

void SpeedRollDeltaPitchPublisherNode::airPressureCb(
  const tobas_msgs::msg::FluidPressureWithVarianceStamped::ConstSharedPtr& msg)
{
  air_density_ = tobas_std::pressureToDensity(msg->pressure.pressure);
  pressure_received_ = true;
}

void SpeedRollDeltaPitchPublisherNode::mainTimerCb()
{
  if (!is_initialized_)
  {
    if (drone_received_ && tree_received_ && pressure_received_)
    {
      check_topics_timer_->cancel();
      initialize();
      is_initialized_ = true;
    }
    return;
  }

  if (trim_.update(cmd_.speed, air_density_, q_0_) < 0)
  {
    TOBAS_ERROR(trim_.errorMessage());
    return;
  }

  // コマンドを更新
  const auto c = key_reader_.readKey();
  if (c < 0)
    TOBAS_ERROR("Failed to read keyboard.");

  switch (c)
  {
    case 'w':
    {
      cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed + delta_speed_);
      TOBAS_INFO("Increase speed");
      break;
    }
    case 's':
    {
      cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed - delta_speed_);
      TOBAS_INFO("Decrease speed");
      break;
    }
    case keyboard::UP:
    {
      cmd_.delta_pitch = clamp(cmd_.delta_pitch - delta_rot_, -max_delta_pitch_, max_delta_pitch_);
      TOBAS_INFO("Nose up");
      break;
    }
    case keyboard::DOWN:
    {
      cmd_.delta_pitch = clamp(cmd_.delta_pitch + delta_rot_, -max_delta_pitch_, max_delta_pitch_);
      TOBAS_INFO("Nose down");
      break;
    }
    case keyboard::LEFT:
    {
      cmd_.roll = clamp(cmd_.roll - delta_rot_, -max_roll_, max_roll_);
      TOBAS_INFO("Turn left");
      break;
    }
    case keyboard::RIGHT:
    {
      cmd_.roll = clamp(cmd_.roll + delta_rot_, -max_roll_, max_roll_);
      TOBAS_INFO("Turn right");
      break;
    }
  }

  // コマンドを発行
  auto cmd_ptr = std::make_unique<tobas_msgs::msg::SpeedRollDeltaPitch>(cmd_);
  cmd_pub_->publish(move(cmd_ptr));
}

void SpeedRollDeltaPitchPublisherNode::checkTopicsTimerCb()
{
  if (!drone_received_)
    TOBAS_INFO("Waiting for \"", tobas::kDroneTopic, "\".");

  if (!tree_received_)
    TOBAS_INFO("Waiting for \"", tobas::kKDLTreeTopic, "\".");

  if (!pressure_received_)
    TOBAS_INFO("Waiting for \"", tobas::kAirPressureTopic, "\".");
}

void SpeedRollDeltaPitchPublisherNode::instructionTimerCb()
{
  TOBAS_INFO(kInstruction);
}
}  // namespace tobas_keyboard_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(tobas_keyboard_teleop::SpeedRollDeltaPitchPublisherNode)
