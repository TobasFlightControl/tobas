#include <rclcpp/wait_for_message.hpp>

#include <tobas_std_tools/range.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_keyboard/keyboard_reader.hpp>
#include <tobas_keyboard/utils.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_action_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>
#include <tobas_mission_msgs/action/takeoff.hpp>

#include "../include/tobas_keyboard_teleop/constants.hpp"

#define TAKEOFF_TARGET_ALTITUDE 3.      // [m]
#define TAKEOFF_ALTITUDE_TOLERANCE 0.1  // [m]
#define TAKEOFF_DURATION 5.             // [s]

using namespace std;

namespace tobas_keyboard_teleop
{
/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class PositionYawPublisherNode : public tobas::BaseNode
{
  static constexpr double kDefaultMaxLinearVelocity = 3.;       // [m/s]
  static constexpr double kDefaultMaxAngularVelocity = M_PI_2;  // [rad/s]
  static constexpr double kDefaultMinimumX = -10.;              // [m]
  static constexpr double kDefaultMaximumX = +10.;              // [m]
  static constexpr double kDefaultMinimumY = -10.;              // [m]
  static constexpr double kDefaultMaximumY = +10.;              // [m]
  static constexpr double kDefaultMinimumZ = -10.;              // [m]
  static constexpr double kDefaultMaximumZ = +10.;              // [m]
  static constexpr double kDefaultMinimumYaw = -M_PI;           // [rad]
  static constexpr double kDefaultMaximumYaw = M_PI;            // [rad]

  static constexpr char kInstruction[] = "Control your drone!\n"
                                         "---------------------------\n"
                                         "W/S       : Move in the positive/negative direction along X-axis in WCSs\n"
                                         "A/D       : Move in the positive/negative direction along Y-axis in WCSs\n"
                                         "Up/Down   : Move in the positive/negative direction along Z-axis in WCSs\n"
                                         "Left/Right: Turn left/right along Z-axis in WCSs\n"
                                         "Ctrl-C    : Quit\n";

  using self = PositionYawPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit PositionYawPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  keyboard::KeyboardReader key_reader_;

  kdl::Vector cmd_pos_;
  double cmd_yaw_;

  // 固定値
  double delta_pos_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;  // 1度のキーボード入力での回転位置の変化量

  // rosparams
  double max_linvel_;  // 並進速度の大きさの最大値
  double max_angvel_;  // 回転速度の大きさの最大値
  tobas_std::Range<double> x_limit_;
  tobas_std::Range<double> y_limit_;
  tobas_std::Range<double> z_limit_;
  tobas_std::Range<double> yaw_limit_;

  // Publishers
  ros2::PublisherPtr<tobas_command_msgs::PosVelYaw> cmd_pub_;

  // Timers
  ros2::TimerPtr process_timer_;
  ros2::TimerPtr instruction_timer_;

  void getStaticRosParams();

  void initializeTimerCb();
  void mainTimerCb();
  void instructionTimerCb();
};

PositionYawPublisherNode::PositionYawPublisherNode(const rclcpp::NodeOptions& options)
  : super("position_yaw_publiser", options)
{
  getStaticRosParams();

  const auto repeat_interval = keyboard::getKeyboardRepeatInterval();
  delta_pos_ = max_linvel_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  cmd_pub_ = createPublisher<tobas_command_msgs::PosVelYaw>(tobas::kPosVelYawCmdTopic);
  process_timer_ = createTimer(0s, &self::initializeTimerCb, this);
}

void PositionYawPublisherNode::getStaticRosParams()
{
  max_linvel_ = getDoubleParam("max_linear_velocity", kDefaultMaxLinearVelocity);
  max_angvel_ = getDoubleParam("max_angular_velocity", kDefaultMaxAngularVelocity);

  x_limit_.lower = getDoubleParam("pose_limit/x/min", kDefaultMinimumX);
  x_limit_.upper = getDoubleParam("pose_limit/x/max", kDefaultMaximumX);
  y_limit_.lower = getDoubleParam("pose_limit/y/min", kDefaultMinimumY);
  y_limit_.upper = getDoubleParam("pose_limit/y/max", kDefaultMaximumY);
  z_limit_.lower = getDoubleParam("pose_limit/z/min", kDefaultMinimumZ);
  z_limit_.upper = getDoubleParam("pose_limit/z/max", kDefaultMaximumZ);
  yaw_limit_.lower = getDoubleParam("pose_limit/yaw/min", kDefaultMinimumYaw);
  yaw_limit_.upper = getDoubleParam("pose_limit/yaw/max", kDefaultMaximumYaw);

  TOBAS_ASSERT(max_linvel_ > 0);
  TOBAS_ASSERT(max_angvel_ > 0);
  TOBAS_ASSERT(x_limit_.isValid());
  TOBAS_ASSERT(y_limit_.isValid());
  TOBAS_ASSERT(z_limit_.isValid());
  TOBAS_ASSERT(yaw_limit_.isValid());
}

void PositionYawPublisherNode::initializeTimerCb()
{
  // 離陸アクションクライアントを用意
  // FIXME: コールバックの中でfuture.wait()を呼ぶとデッドロックするため，keyboard_teleopはメイン関数にベタ書きする．
  ros2::SyncActionClient<tobas_mission_msgs::action::Takeoff> takeoff_ac(shared_from_this(), tobas::kTakeoffAction);

  // 離陸
  TOBAS_INFO("Requesting takeoff_ac action.");
  tobas_mission_msgs::action::Takeoff::Goal takeoff_goal;
  takeoff_goal.level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
  takeoff_goal.target_altitude = TAKEOFF_TARGET_ALTITUDE;
  takeoff_goal.altitude_tolerance = TAKEOFF_ALTITUDE_TOLERANCE;
  takeoff_goal.duration = TAKEOFF_DURATION;
  if (!takeoff_ac.sendGoalAndWait(takeoff_goal)) {
    TOBAS_EXIT("Takeoff action failed.");
  }
  const auto takeoff_result = takeoff_ac.getResult();
  if (takeoff_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    TOBAS_EXIT("Takeoff action failed: ", takeoff_result.result->message);
  }
  TOBAS_INFO("Takeoff finished successfully.");

  // 初期コマンドを設定
  tobas_msgs::Odometry odom;
  if (
    rclcpp::wait_for_message(odom, shared_from_this(), tobas::kOdometryTopic)
    && odom.status == tobas_msgs::msg::Odometry::NO_ERROR) {
    cmd_pos_ = odom.frame.p;
    cmd_yaw_ = odom.frame.M.getYaw();
  }
  else {
    TOBAS_ERROR("Failed to get ", tobas::kOdometryTopic, ".");
    cmd_pos_.x() = 0;
    cmd_pos_.y() = 0;
    cmd_pos_.z() = takeoff_goal.target_altitude;
    cmd_yaw_ = 0;
  }

  // メインプロセスに以降
  process_timer_->cancel();
  process_timer_ = createTimer(kCommandPeriod, &self::mainTimerCb, this);

  // インストラクションの表示を開始
  cout << kInstruction << endl;
  instruction_timer_ = createTimer(kInstructionTimerPeriod, &self::instructionTimerCb, this);
}

void PositionYawPublisherNode::mainTimerCb()
{
  // キーボード入力に依ってコマンドを更新
  const auto c = key_reader_.readKey();
  if (c < 0) {
    TOBAS_ERROR("Failed to read keyboard.");
    return;
  }

  switch (c) {
    case 'w':  // X+
    {
      cmd_pos_.x(x_limit_.clamp(cmd_pos_.x() + delta_pos_));
      TOBAS_INFO("[Moving forward] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
    case 's':  // X-
    {
      cmd_pos_.x(x_limit_.clamp(cmd_pos_.x() - delta_pos_));
      TOBAS_INFO("[Moving backward] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
    case 'a':  // Y+
    {
      cmd_pos_.y(y_limit_.clamp(cmd_pos_.y() + delta_pos_));
      TOBAS_INFO("[Moving left] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
    case 'd':  // Y-
    {
      cmd_pos_.y(y_limit_.clamp(cmd_pos_.y() - delta_pos_));
      TOBAS_INFO("[Moving right] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
    case keyboard::UP:  // Z+
    {
      cmd_pos_.z(z_limit_.clamp(cmd_pos_.z() + delta_pos_));
      TOBAS_INFO("[Moving up] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
    case keyboard::DOWN:  // Z-
    {
      cmd_pos_.z(z_limit_.clamp(cmd_pos_.z() - delta_pos_));
      TOBAS_INFO("[Moving down] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
    case keyboard::LEFT:  // Yaw+
    {
      cmd_yaw_ = yaw_limit_.clamp(cmd_yaw_ + delta_rot_);
      TOBAS_INFO("[Rotating left] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
    case keyboard::RIGHT:  // Yaw-
    {
      cmd_yaw_ = yaw_limit_.clamp(cmd_yaw_ - delta_rot_);
      TOBAS_INFO("[Rotating right] pos[m]: ", cmd_pos_, ", yaw[rad]: ", cmd_yaw_);
      break;
    }
  }

  // コマンドを発行
  auto cmd = std::make_unique<tobas_command_msgs::PosVelYaw>();
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::NORMAL;
  cmd->pos = cmd_pos_;
  cmd->vel.setZero();
  cmd->yaw = cmd_yaw_;
  cmd_pub_->publish(move(cmd));
}

void PositionYawPublisherNode::instructionTimerCb()
{
  cout << kInstruction << endl;
}
}  // namespace tobas_keyboard_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(tobas_keyboard_teleop::PositionYawPublisherNode)
