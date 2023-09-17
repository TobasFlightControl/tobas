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
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/AccelerationYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_translation_lqr/controller.hpp>
#include <tobas_mr_rotation_mpc/acceleration_controller.hpp>
#include <tobas_mr_rotation_mpc/rotation_controller.hpp>

#include <tobas_mr_lqrmpc/ControllerConfig.h>

namespace tobas_mr_lqrmpc
{
class ControllerRos : public tobas::BaseNode
{
  static constexpr double kWarnPeriod = 1.;              // [s]
  static constexpr double kErrorPeriod = 1.;             // [s]
  static constexpr double kCheckTopicsTimerPeriod = 5.;  // [s]

  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_lqrmpc::ControllerConfig;
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
  tobas_mr_translation_lqr::Controller pos_controller_;
  tobas_mr_rotation_mpc::AccelerationController acc_controller_;
  tobas_mr_rotation_mpc::RotationController rot_controller_;

  // Dynamic parameters
  tobas_mr_translation_lqr::Config pos_params_;
  tobas_mr_rotation_mpc::AccelerationControllerDynamicParams acc_params_;
  tobas_mr_rotation_mpc::RotationControllerDynamicParams rot_params_;

  // Constant variables
  bool is_transformable_;  // プロペラ以外の可動関節を持つか否か

  // Mutable variables
  tobas_msgs::PoseTwistConstPtr pt_;
  tobas_msgs::BatteryConstPtr battery_;
  sensor_msgs::JointStateConstPtr js_;
  tobas_msgs::PositionYawPtr tar_pos_yaw_;      // PositionYawの目標値
  tobas_msgs::AccelerationYawPtr tar_acc_yaw_;  // AccelerationYawの目標値 (世界座標系)
  tobas_msgs::RollPitchYawThrustPtr tar_rpy_thrust_;  // RollPitchYawThrustの目標値
  bool is_initialized_ = false;
  uint8_t cmd_level_ = tobas_msgs::CommandLevel::NORMAL;
  KDL::JntArray q_;  // 全ての非固定関節の角度
  Eigen::VectorXd u_opt_;
  ros::Time t_last_loop_;

  // Publishers
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher feedback_pub_;

  // Subscribers
  ros::Subscriber pt_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber pos_yaw_sub_;
  ros::Subscriber acc_yaw_sub_;
  ros::Subscriber rpy_thrust_sub_;

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
  void jointStateCb(const sensor_msgs::JointStateConstPtr& js);
  void posYawCb(const tobas_msgs::PositionYawConstPtr& pos_yaw);
  void accYawCb(const tobas_msgs::AccelerationYawConstPtr& acc_yaw);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpy_thrust);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_lqrmpc
