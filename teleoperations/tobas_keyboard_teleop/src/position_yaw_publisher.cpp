#include <actionlib/client/simple_action_client.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_multirotor_takeoff/MultirotorTakeoffAction.h>

#include "../include/tobas_keyboard_teleop/position_yaw_publisher.hpp"
#include "../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
ostream& operator<<(ostream& os, const tobas_msgs::PositionYaw& arg)
{
  os << "x = " << arg.pos.x() << "[m], ";
  os << "y = " << arg.pos.y() << "[m], ";
  os << "z = " << arg.pos.z() << "[m], ";
  os << "yaw = " << arg.yaw << "[rad]";
  return os;
}

PositionYawPublisher::PositionYawPublisher(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
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
  while (pt_ == nullptr)
  {
    rosWarnThrottle(tobas::kCheckTopicsTimerPeriod, name_, "Pose & Twist is not received yet.");
    sleep(0.1);
    ros::spinOnce();
  }

  // 離陸アクションクライアントを用意
  actionlib::SimpleActionClient<tobas_multirotor_takeoff::MultirotorTakeoffAction> takeoff(
    tobas::kTakeoffAction);
  rosInfo(name_, "Waiting for '" << tobas::kTakeoffAction << "' action server.");
  if (!takeoff.waitForServer(ros::Duration(kWaitForExternalActionServer)))
  {
    rosInfo(name_, "Failed to connect to '" << tobas::kTakeoffAction << "' action server.");
    return;
  }

  // 離陸
  rosInfo(name_, "Requesting takeoff action.");
  tobas_multirotor_takeoff::MultirotorTakeoffGoal takeoff_goal;
  takeoff_goal.level.data = tobas_msgs::CommandLevel::NORMAL;
  takeoff.sendGoalAndWait(takeoff_goal);
  const auto takeoff_result = takeoff.getResult();
  if (takeoff_result->error_code != tobas_multirotor_takeoff::MultirotorTakeoffResult::NO_ERROR)
  {
    rosInfo(name_, "'" << tobas::kTakeoffAction << "' action failed.");
    return;
  }
  rosInfo(name_, "Takeoff finished successfully. Start teleoperation!");

  // 初期コマンドを設定
  auto cmd = boost::make_shared<tobas_msgs::PositionYaw>();
  cmd->pos = pt_->pose.pos;
  cmd->yaw = pt_->pose.euler.yaw;

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
        cmd->pos.x(x_limit_.clamp(cmd->pos.x() + delta_pos_));
        rosInfo(name_, "Moving forward: " << cmd);
        break;
      }
      case kKeyCode_S:  // X-
      {
        cmd->pos.x(x_limit_.clamp(cmd->pos.x() - delta_pos_));
        rosInfo(name_, "Moving backward: " << cmd);
        break;
      }
      case kKeyCode_A:  // Y+
      {
        cmd->pos.y(y_limit_.clamp(cmd->pos.y() + delta_pos_));
        rosInfo(name_, "Moving left: " << cmd);
        break;
      }
      case kKeyCode_D:  // Y-
      {
        cmd->pos.y(y_limit_.clamp(cmd->pos.y() - delta_pos_));
        rosInfo(name_, "Moving right: " << cmd);
        break;
      }
      case kKeyCode_Up:  // Z+
      {
        cmd->pos.z(z_limit_.clamp(cmd->pos.z() + delta_pos_));
        rosInfo(name_, "Moving up: " << cmd);
        break;
      }
      case kKeyCode_Down:  // Z-
      {
        cmd->pos.z(z_limit_.clamp(cmd->pos.z() - delta_pos_));
        rosInfo(name_, "Moving down: " << cmd);
        break;
      }
      case kKeyCode_Left:  // Yaw+
      {
        cmd->yaw = yaw_limit_.clamp(cmd->yaw + delta_rot_);
        rosInfo(name_, "Rotating left: " << cmd);
        break;
      }
      case kKeyCode_Right:  // Yaw-
      {
        cmd->yaw = yaw_limit_.clamp(cmd->yaw - delta_rot_);
        rosInfo(name_, "Rotating right: " << cmd);
        break;
      }
    }

    // コマンドを発行
    cmd_pub_.publish(cmd);

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
  cmd_pub_ = nh_.advertise<tobas_msgs::PositionYaw>("command/position_yaw", 1);
}

void PositionYawPublisher::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &PositionYawPublisher::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe("pose_twist", 1, &PositionYawPublisher::poseTwistCb, this, tcpNoDelay());
}

void PositionYawPublisher::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  pt_ = pt;
}

void PositionYawPublisher::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_keyboard_teleop
