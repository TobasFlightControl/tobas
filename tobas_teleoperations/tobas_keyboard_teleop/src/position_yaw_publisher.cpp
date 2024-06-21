#include <actionlib/client/simple_action_client.h>

#include <tobas_std_tools/algorithm.hpp>
#include <tobas_keyboard/utils.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/rate.hpp>
#include <tobas_ros_tools/util.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/TakeoffAction.h>

#include "../include/tobas_keyboard_teleop/position_yaw_publisher.hpp"
#include "../include/tobas_keyboard_teleop/constants.hpp"

#define TAKEOFF_TARGET_ALTITUDE 3.      // [m]
#define TAKEOFF_ALTITUDE_TOLERANCE 0.1  // [m]
#define TAKEOFF_DURATION 5.             // [s]

using namespace std;

namespace tobas_keyboard_teleop
{
PositionYawPublisher::PositionYawPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  instruction_ = "Control your drone!\n"
                 "---------------------------\n"
                 "W/S       : Move in the positive/negative direction along X-axis in WCSs\n"
                 "A/D       : Move in the positive/negative direction along Y-axis in WCSs\n"
                 "Up/Down   : Move in the positive/negative direction along Z-axis in WCSs\n"
                 "Left/Right: Turn left/right along Z-axis in WCSs\n"
                 "Ctrl-C    : Quit\n";

  getRosParams();

  const auto repeat_interval = keyboard::getKeyboardRepeatInterval();
  delta_pos_ = max_linvel_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  pos_yaw_pub_ = nh_.advertise<tobas_msgs::PositionYaw>(tobas::kPositionYawCmdTopic, 1);
  pvay_pub_ = nh_.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
}

void PositionYawPublisher::run()
{
  // 離陸アクションクライアントを用意
  actionlib::SimpleActionClient<tobas_msgs::TakeoffAction> takeoff(tobas::kTakeoffAction);
  ROS_INFO_STREAM("Waiting for '" << tobas::kTakeoffAction << "' action server.");
  if (!takeoff.waitForServer(ros::Duration(kWaitForExternalActionServer)))
  {
    ROS_ERROR_STREAM("Failed to connect to '" << tobas::kTakeoffAction << "' action server.");
    return;
  }

  // 離陸
  ROS_INFO_STREAM("Requesting takeoff action.");
  tobas_msgs::TakeoffGoal takeoff_goal;
  takeoff_goal.level.data = tobas_msgs::CommandLevel::NORMAL;
  takeoff_goal.target_altitude = TAKEOFF_TARGET_ALTITUDE;
  takeoff_goal.altitude_tolerance = TAKEOFF_ALTITUDE_TOLERANCE;
  takeoff_goal.duration = TAKEOFF_DURATION;
  takeoff.sendGoalAndWait(takeoff_goal);
  const auto takeoff_result = takeoff.getResult();
  const auto takeoff_state = takeoff.getState();
  if (takeoff_state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR_STREAM("'" << tobas::kTakeoffAction << "' action failed: " << takeoff_state.getText());
    return;
  }
  ROS_INFO_STREAM("Takeoff finished successfully.");

  // 初期コマンドを設定
  tobas_msgs::Odometry odom;
  if (tobas_ros::subscribeOnce(odom, tobas::kOdometryTopic, nh_) && odom.status == tobas_msgs::Odometry::NO_ERROR)
  {
    cmd_pos_ = odom.frame.p;
    cmd_yaw_ = kdl::Euler(odom.frame.M).yaw;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get " << nh_.getNamespace() << "/" << tobas::kOdometryTopic << ".");
    cmd_pos_.x() = 0;
    cmd_pos_.y() = 0;
    cmd_pos_.z() = takeoff_goal.target_altitude;
    cmd_yaw_ = 0;
  }

  // キーボード入力による位置コマンドを発行し続ける
  tobas_ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    // インストラクション
    TOBAS_INFO_THROTTLE(kInstructionTimerPeriod, instruction_);

    // キーボード入力に依ってコマンドを更新
    const auto c = key_reader_.readKey();
    if (c < 0)
      ROS_ERROR_STREAM("Failed to read keyboard.");

    switch (c)
    {
      case 'w':  // X+
      {
        cmd_pos_.x(x_limit_.clamp(cmd_pos_.x() + delta_pos_));
        ROS_INFO_STREAM("[Moving forward] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case 's':  // X-
      {
        cmd_pos_.x(x_limit_.clamp(cmd_pos_.x() - delta_pos_));
        ROS_INFO_STREAM("[Moving backward] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case 'a':  // Y+
      {
        cmd_pos_.y(y_limit_.clamp(cmd_pos_.y() + delta_pos_));
        ROS_INFO_STREAM("[Moving left] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case 'd':  // Y-
      {
        cmd_pos_.y(y_limit_.clamp(cmd_pos_.y() - delta_pos_));
        ROS_INFO_STREAM("[Moving right] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case keyboard::UP:  // Z+
      {
        cmd_pos_.z(z_limit_.clamp(cmd_pos_.z() + delta_pos_));
        ROS_INFO_STREAM("[Moving up] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case keyboard::DOWN:  // Z-
      {
        cmd_pos_.z(z_limit_.clamp(cmd_pos_.z() - delta_pos_));
        ROS_INFO_STREAM("[Moving down] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case keyboard::LEFT:  // Yaw+
      {
        cmd_yaw_ = yaw_limit_.clamp(cmd_yaw_ + delta_rot_);
        ROS_INFO_STREAM("[Rotating left] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
        break;
      }
      case keyboard::RIGHT:  // Yaw-
      {
        cmd_yaw_ = yaw_limit_.clamp(cmd_yaw_ - delta_rot_);
        ROS_INFO_STREAM("[Rotating right] pos[m]: " << cmd_pos_ << ", yaw[rad]: " << cmd_yaw_);
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
    pvay_msg->vel.setZero();
    pvay_msg->acc.setZero();
    pvay_msg->yaw = cmd_yaw_;
    pvay_pub_.publish(pvay_msg);

    ros::spinOnce();
    rate.sleep();
  }
}

void PositionYawPublisher::getRosParams()
{
  tobas_ros::getParam(pnh_, "max_linear_velocity", max_linvel_, kDefaultMaxLinearVelocity, tobas_ros::POSITIVE);
  tobas_ros::getParam(pnh_, "max_angular_velocity", max_angvel_, kDefaultMaxAngularVelocity, tobas_ros::POSITIVE);

  tobas_ros::getParam(pnh_, "pose_limit/x/min", x_limit_.lower, kDefaultMinimumX);
  tobas_ros::getParam(pnh_, "pose_limit/x/max", x_limit_.upper, kDefaultMaximumX);
  tobas_ros::getParam(pnh_, "pose_limit/y/min", y_limit_.lower, kDefaultMinimumY);
  tobas_ros::getParam(pnh_, "pose_limit/y/max", y_limit_.upper, kDefaultMaximumY);
  tobas_ros::getParam(pnh_, "pose_limit/z/min", z_limit_.lower, kDefaultMinimumZ);
  tobas_ros::getParam(pnh_, "pose_limit/z/max", z_limit_.upper, kDefaultMaximumZ);
  tobas_ros::getParam(pnh_, "pose_limit/yaw/min", yaw_limit_.lower, kDefaultMinimumYaw);
  tobas_ros::getParam(pnh_, "pose_limit/yaw/max", yaw_limit_.upper, kDefaultMaximumYaw);

  ROS_CHECK(nh_, x_limit_.isValid(), "X range is invalid.");
  ROS_CHECK(nh_, y_limit_.isValid(), "Y range is invalid.");
  ROS_CHECK(nh_, z_limit_.isValid(), "Z range is invalid.");
  ROS_CHECK(nh_, yaw_limit_.isValid(), "Yaw range is invalid.");
}
}  // namespace tobas_keyboard_teleop
