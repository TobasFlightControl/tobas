#include <Eigen/Core>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_std_tools/stopwatch.hpp>
#include <dh_kdl/treejntnameparser.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/position_pid.hpp>
#include <tobas_tools/orientation_pid.hpp>
#include <tobas_mr_common/accel_attitude_converter.hpp>
#include <tobas_mr_common/mixer.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_pid/ControllerConfig.h>

namespace tobas_mr_pid
{
class ControllerRos : public tobas::BaseNode
{
  using self = ControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_pid::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ControllerRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // Drone
  tobas::Drone drone_;
  KDL::TreeJointNameParser jnt_name_parser_;
  tobas::RotorAxisExtractor z_rotors_;

  // Controllers
  tobas::PositionPid pos_ctrl_;
  tobas_mr_common::AccelAttitudeConverter acc_ctrl_;
  tobas::OrientationPid ori_ctrl_;
  tobas_mr_common::Mixer mixer_;

  // Dynamic parameters
  tobas::PositionPidConfig pos_cfg_;
  tobas_mr_common::AccelAttitudeConverterConfig acc_cfg_;
  tobas::OrientationPidConfig ori_cfg_;

  // Mutable variables
  tobas_msgs::OdometryConstPtr odom_;
  tobas_msgs::BatteryConstPtr battery_;
  sensor_msgs::JointStateConstPtr js_;
  tobas_msgs::PosVelAccYawPtr tar_pvay_;        // PosVelYawの目標値 (世界座標系)
  tobas_msgs::RollPitchYawThrustPtr tar_rpyt_;  // RollPitchYawThrustの目標値
  bool is_initialized_ = false;
  uint8_t cmd_level_ = tobas_msgs::CommandLevel::NORMAL;
  KDL::JntArray q_;  // 全ての非固定関節の角度
  ros::Time t_last_loop_;

  // Publishers
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher feedback_pub_;

  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber pvay_sub_;
  ros::Subscriber rpyt_sub_;

  // Timers
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady() const;
  void updateJointArray();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& js);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_pid
