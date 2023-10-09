#include <Eigen/Core>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_std_tools/stopwatch.hpp>
#include <dh_kdl/treejntnameparser.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_common/acceleration_controller.hpp>
#include <tobas_mr_common/mixer.hpp>

#include <tobas_mr_pid/ControllerConfig.h>

#include "./translation_controller.hpp"
#include "./rotation_controller.hpp"

namespace tobas_mr_pid
{
class ControllerRos : public tobas::BaseNode
{
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_pid::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ControllerRos(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  // Drone
  tobas::Drone drone_;
  KDL::TreeJointNameParser jnt_name_parser_;
  tobas::RotorAxisExtractor z_rotors_;

  // Controllers
  TranslationController trans_controller_;
  tobas_mr_common::AccelerationController acc_controller_;
  RotationController rot_controller_;
  tobas_mr_common::Mixer mixer_;

  // Dynamic parameters
  TranslationControllerConfig trans_config_;
  tobas_mr_common::AccelerationControllerConfig acc_config_;
  RotationControllerConfig rot_config_;

  // Constant variables
  bool is_transformable_;  // プロペラ以外の可動関節を持つか否か

  // Mutable variables
  tobas_msgs::PoseTwistConstPtr pt_;
  tobas_msgs::BatteryConstPtr battery_;
  sensor_msgs::JointStateConstPtr js_;
  tobas_msgs::PosVelAccYawPtr tar_pvay_;        // PosVelYawの目標値 (世界座標系)
  tobas_msgs::RollPitchYawThrustPtr tar_rpyt_;  // RollPitchYawThrustの目標値
  bool is_initialized_ = false;
  uint8_t cmd_level_ = tobas_msgs::CommandLevel::NORMAL;
  KDL::JntArray q_;  // 全ての非固定関節の角度
  KDL::Vector tar_acc_fb_;
  ros::Time t_last_loop_;

  // Publishers
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher feedback_pub_;

  // Subscribers
  ros::Subscriber pt_sub_;
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
  bool isCommandLevelOk(const tobas_msgs::CommandLevel& level);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& js);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_pid
