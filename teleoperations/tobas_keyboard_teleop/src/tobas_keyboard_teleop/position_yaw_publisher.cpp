#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_keyboard_teleop/position_yaw_publisher.hpp"
#include "../../include/tobas_keyboard_teleop/constants.hpp"

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

PositionYawPublisher::PositionYawPublisher()
  : super(), keyboard_(getKeyboardControls()), bs_received_(false)
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
  rosInfo("Keyboard repeat interval is " << keyboard_->repeat_interval << " [ms].");

  delta_pos_ = max_linvel_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  cmd_.level.data = tobas_msgs::CommandLevel::NORMAL;

  registerPublishers();
  registerSubscribers();
}

void PositionYawPublisher::run()
{
  // 初期状態が得られるまで待機
  while (ros::ok() && !bs_received_)
  {
    rosWarnThrottle(kCheckTopicsTimerPeriod, "Base state is not received yet.");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // 初期コマンド
  cmd_.pos.x(bs_.pose.pos.x());
  cmd_.pos.y(bs_.pose.pos.y());
  cmd_.pos.z(bs_.pose.pos.z() + init_elevation_);
  cmd_.yaw = bs_.pose.euler.yaw;

  dh_ros::Rate rate(kUpdateRate);
  while (ros::ok())
  {
    // インストラクション
    rosInfoThrottle(kInstructionTimerPeriod, instruction_);

    // キーボード入力に依ってコマンドを更新
    const auto c = key_reader_.readKey();
    switch (c)
    {
      case kKeyCode_W:  // X+
      {
        cmd_.pos.x(x_limit_.clamp(cmd_.pos.x() + delta_pos_));
        rosInfo("Moving forward: " << cmd_);
        break;
      }
      case kKeyCode_S:  // X-
      {
        cmd_.pos.x(x_limit_.clamp(cmd_.pos.x() - delta_pos_));
        rosInfo("Moving backward: " << cmd_);
        break;
      }
      case kKeyCode_A:  // Y+
      {
        cmd_.pos.y(y_limit_.clamp(cmd_.pos.y() + delta_pos_));
        rosInfo("Moving left: " << cmd_);
        break;
      }
      case kKeyCode_D:  // Y-
      {
        cmd_.pos.y(y_limit_.clamp(cmd_.pos.y() - delta_pos_));
        rosInfo("Moving right: " << cmd_);
        break;
      }
      case kKeyCode_Up:  // Z+
      {
        cmd_.pos.z(z_limit_.clamp(cmd_.pos.z() + delta_pos_));
        rosInfo("Moving up: " << cmd_);
        break;
      }
      case kKeyCode_Down:  // Z-
      {
        cmd_.pos.z(z_limit_.clamp(cmd_.pos.z() - delta_pos_));
        rosInfo("Moving down: " << cmd_);
        break;
      }
      case kKeyCode_Left:  // Yaw+
      {
        cmd_.yaw = yaw_limit_.clamp(cmd_.yaw + delta_rot_);
        rosInfo("Rotating left: " << cmd_);
        break;
      }
      case kKeyCode_Right:  // Yaw-
      {
        cmd_.yaw = yaw_limit_.clamp(cmd_.yaw - delta_rot_);
        rosInfo("Rotating right: " << cmd_);
        break;
      }
    }

    // コマンドを発行
    cmd_pub_.publish(cmd_);

    ros::spinOnce();
    rate.sleep();
  }
}

void PositionYawPublisher::getRosParams()
{
  dh_ros::getParam("~initial_elevation", init_elevation_, kDefaultInitialElevation);
  dh_ros::getParam("~max_linear_velocity", max_linvel_, kDefaultMaxLinearVelocity);
  dh_ros::getParam("~max_angular_velocity", max_angvel_, kDefaultMaxAngularVelocity);
  dh_ros::getParam("~pose_limit/x/min", x_limit_.lower, kDefaultMinimumX);
  dh_ros::getParam("~pose_limit/x/max", x_limit_.upper, kDefaultMaximumX);
  dh_ros::getParam("~pose_limit/y/min", y_limit_.lower, kDefaultMinimumY);
  dh_ros::getParam("~pose_limit/y/max", y_limit_.upper, kDefaultMaximumY);
  dh_ros::getParam("~pose_limit/z/min", z_limit_.lower, kDefaultMinimumZ);
  dh_ros::getParam("~pose_limit/z/max", z_limit_.upper, kDefaultMaximumZ);
  dh_ros::getParam("~pose_limit/yaw/min", yaw_limit_.lower, kDefaultMinimumYaw);
  dh_ros::getParam("~pose_limit/yaw/max", yaw_limit_.upper, kDefaultMaximumYaw);

  ROS_ASSERT(init_elevation_ >= 0.);
  ROS_ASSERT(max_linvel_ > 0.);
  ROS_ASSERT(max_angvel_ > 0.);
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
  event_sub_ = nh_.subscribe("event", 1, &PositionYawPublisher::eventCb, this);
  bs_sub_ = nh_.subscribe("base_state", 1, &PositionYawPublisher::baseStateCb, this);
}

void PositionYawPublisher::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}

void PositionYawPublisher::baseStateCb(const tobas_msgs::BaseState& bs)
{
  bs_ = bs;

  if (!bs_received_)
  {
    rosInfo("First base state is received.");
    bs_received_ = true;
  }
}
}  // namespace tobas_keyboard_teleop
