#include <actionlib/client/simple_action_client.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/util.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/TakeoffAction.h>

#include "../include/tobas_keyboard_teleop/position_yaw_publisher.hpp"
#include "../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
PositionYawPublisher::PositionYawPublisher(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name), keyboard_(getKeyboardControls())
{
  instruction_ = "Control your drone!\n"
                 "---------------------------\n"
                 "W/S       : Move in the positive/negative direction along X-axis in WCSs\n"
                 "A/D       : Move in the positive/negative direction along Y-axis in WCSs\n"
                 "Up/Down   : Move in the positive/negative direction along Z-axis in WCSs\n"
                 "Left/Right: Turn left/right along Z-axis in WCSs\n"
                 "Ctrl-C    : Quit\n";

  getRosParams();

  const auto repeat_interval = keyboard_->repeat_interval * 1e-3;  // ms -> s
  rosInfo(name_, "Keyboard repeat interval is " << keyboard_->repeat_interval << " [ms].");

  delta_pos_ = max_linvel_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  registerPublishers();
  registerSubscribers();
}

void PositionYawPublisher::run()
{
  // 離陸アクションクライアントを用意
  actionlib::SimpleActionClient<tobas_msgs::TakeoffAction> takeoff(tobas::kTakeoffAction);
  rosInfo(name_, "Waiting for '" << tobas::kTakeoffAction << "' action server.");
  if (!takeoff.waitForServer(ros::Duration(kWaitForExternalActionServer)))
  {
    rosError(name_, "Failed to connect to '" << tobas::kTakeoffAction << "' action server.");
    return;
  }

  // 離陸
  rosInfo(name_, "Requesting takeoff action.");
  tobas_msgs::TakeoffGoal takeoff_goal;
  takeoff_goal.level.data = tobas_msgs::CommandLevel::NORMAL;
  takeoff.sendGoalAndWait(takeoff_goal);
  const auto takeoff_result = takeoff.getResult();
  const auto takeoff_state = takeoff.getState();
  if (takeoff_result->error_code != tobas_msgs::TakeoffResult::NO_ERROR)
  {
    rosError(name_, "'" << tobas::kTakeoffAction << "' action failed: " << takeoff_state.getText());
    return;
  }
  rosInfo(name_, "Takeoff finished successfully. Start teleoperation!");

  // 初期コマンドを設定
  tobas_msgs::PoseTwist pt;
  if (dh_ros::subscribeOnce(pt, tobas::kPoseTwistTopic, nh_))
  {
    cmd_pos_ = pt.pose.pos;
    cmd_yaw_ = pt.pose.euler.yaw;
  }
  else
  {
    rosError(name_, "Failed to get " << nh_.getNamespace() << "/" << tobas::kPoseTwistTopic << ".");
    // TODO: 初期コマンドを離陸コマンドと同じに設定
  }

  // キーボード入力による位置コマンドを発行し続ける
  dh_ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    // インストラクション
    rosInfoThrottle(kInstructionTimerPeriod, name_, instruction_);

    // キーボード入力に依ってコマンドを更新
    const auto c = key_reader_.readKey();
    switch (c)
    {
      case kKeyCode_W:  // X+
      {
        cmd_pos_.x(x_limit_.clamp(cmd_pos_.x() + delta_pos_));
        rosInfo(name_, "[Moving forward] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case kKeyCode_S:  // X-
      {
        cmd_pos_.x(x_limit_.clamp(cmd_pos_.x() - delta_pos_));
        rosInfo(name_, "[Moving backward] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case kKeyCode_A:  // Y+
      {
        cmd_pos_.y(y_limit_.clamp(cmd_pos_.y() + delta_pos_));
        rosInfo(name_, "[Moving left] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case kKeyCode_D:  // Y-
      {
        cmd_pos_.y(y_limit_.clamp(cmd_pos_.y() - delta_pos_));
        rosInfo(name_, "[Moving right] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case kKeyCode_Up:  // Z+
      {
        cmd_pos_.z(z_limit_.clamp(cmd_pos_.z() + delta_pos_));
        rosInfo(name_, "[Moving up] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case kKeyCode_Down:  // Z-
      {
        cmd_pos_.z(z_limit_.clamp(cmd_pos_.z() - delta_pos_));
        rosInfo(name_, "[Moving down] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case kKeyCode_Left:  // Yaw+
      {
        cmd_yaw_ = yaw_limit_.clamp(cmd_yaw_ + delta_rot_);
        rosInfo(name_, "[Rotating left] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case kKeyCode_Right:  // Yaw-
      {
        cmd_yaw_ = yaw_limit_.clamp(cmd_yaw_ - delta_rot_);
        rosInfo(name_, "[Rotating right] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
    }

    // コマンドを発行
    const auto pos_yaw_msg = boost::make_shared<tobas_msgs::PositionYaw>();
    pos_yaw_msg->level.data = tobas_msgs::CommandLevel::NORMAL;
    pos_yaw_msg->pos = cmd_pos_;
    pos_yaw_msg->yaw = cmd_yaw_;
    pos_yaw_pub_.publish(pos_yaw_msg);

    const auto pvay_msg = boost::make_shared<tobas_msgs::PosVelAccYaw>();
    pvay_msg->level.data = tobas_msgs::CommandLevel::NORMAL;
    pvay_msg->pos = cmd_pos_;
    pvay_msg->yaw = cmd_yaw_;
    pvay_pub_.publish(pvay_msg);

    ros::spinOnce();
    rate.sleep();
  }
}

void PositionYawPublisher::getRosParams()
{
  dh_ros::getParam(
    pnh_, "max_linear_velocity", max_linvel_, kDefaultMaxLinearVelocity, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh_, "max_angular_velocity", max_angvel_, kDefaultMaxAngularVelocity, dh_ros::POSITIVE);

  dh_ros::getParam(pnh_, "pose_limit/x/min", x_limit_.lower, kDefaultMinimumX);
  dh_ros::getParam(pnh_, "pose_limit/x/max", x_limit_.upper, kDefaultMaximumX);
  dh_ros::getParam(pnh_, "pose_limit/y/min", y_limit_.lower, kDefaultMinimumY);
  dh_ros::getParam(pnh_, "pose_limit/y/max", y_limit_.upper, kDefaultMaximumY);
  dh_ros::getParam(pnh_, "pose_limit/z/min", z_limit_.lower, kDefaultMinimumZ);
  dh_ros::getParam(pnh_, "pose_limit/z/max", z_limit_.upper, kDefaultMaximumZ);
  dh_ros::getParam(pnh_, "pose_limit/yaw/min", yaw_limit_.lower, kDefaultMinimumYaw);
  dh_ros::getParam(pnh_, "pose_limit/yaw/max", yaw_limit_.upper, kDefaultMaximumYaw);
  ROS_ASSERT(x_limit_.isValid());
  ROS_ASSERT(y_limit_.isValid());
  ROS_ASSERT(z_limit_.isValid());
  ROS_ASSERT(yaw_limit_.isValid());
}

void PositionYawPublisher::registerPublishers()
{
  pos_yaw_pub_ = nh_.advertise<tobas_msgs::PositionYaw>(tobas::kPositionYawCmdTopic, 1);
  pvay_pub_ = nh_.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
}

void PositionYawPublisher::registerSubscribers()
{
  super::registerSubscribers();
}

void PositionYawPublisher::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_keyboard_teleop
