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
#include <tobas_msgs/Wind.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_translation_lqr/controller.hpp>
#include <tobas_mr_common/accel_attitude_converter.hpp>
#include <tobas_mr_rotation_mpc/rotation_mpc.hpp>

#include <tobas_mr_mpc/ControllerConfig.h>

namespace tobas_mr_mpc
{
class ControllerRos : public tobas::BaseNode
{
  static constexpr double kCommandLevelErrorPeriod = 1.;  // [s]
  static constexpr double kCheckTopicsTimerPeriod = 5.;   // [s]

  using self = ControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_mpc::ControllerConfig;
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
  tobas_mr_translation_lqr::TranslationController trans_ctrl_;
  tobas_mr_common::AccelAttitudeConverter acc_ctrl_;
  tobas_mr_rotation_mpc::RotationMpc rot_ctrl_;

  // Dynamic parameters
  tobas_mr_translation_lqr::Config trans_cfg_;
  tobas_mr_common::AccelAttitudeConverterConfig acc_cfg_;
  tobas_mr_rotation_mpc::RotationMpcConfig rot_cfg_;

  // Mutable variables
  tobas_msgs::PoseTwistConstPtr pt_;
  tobas_msgs::BatteryConstPtr battery_;
  tobas_msgs::WindConstPtr wind_;  // 風速 (世界座標系)
  tobas_msgs::RotorSpeedsConstPtr rotor_speeds_;
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
  ros::Subscriber wind_sub_;
  ros::Subscriber rotor_speeds_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber pvay_sub_;
  ros::Subscriber rpyt_sub_;

  // Timers
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  // Other
  dh_std::Stopwatch stopwatch_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady() const;
  bool isCommandLevelOk(const tobas_msgs::CommandLevel& level);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void windCb(const tobas_msgs::WindConstPtr& wind);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& js);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpy_thrust);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_mpc
