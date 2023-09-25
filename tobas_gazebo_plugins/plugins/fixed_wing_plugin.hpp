#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <dh_std_tools/range.hpp>

#include <tobas_tools/fixed_wing_tools.hpp>
#include <tobas_msgs/ControlSurfaceDeflections.h>
#include <tobas_msgs/Wind.h>
#include <tobas_gazebo_plugins/FixedWingDebug.h>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/simple_joint_model.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "fixed_wing_plugin";

// Default values
static const std::string kDefaultDebugPubTopic = "ground_truth/fixed_wing_debug";
static const std::string kDefaultDeflectionsSubTopic = "command/deflections";
static constexpr double kDefaultLowerStallAngle = -10. * kDegreeToRadian;
static constexpr double kDefaultUpperStallAngle = 20. * kDegreeToRadian;

/**
 * @brief 固定翼機に作用する空気力のプラグイン．
 * cf. 航空機の飛行力学と制御: https://www.morikita.co.jp/books/mid/069081
 *
 * @note
 * 全てSI単位系を用いる．
 * 舵面ごとに別々のプラグインにすることも検討したが，揚力係数等の計算が面倒になるため全ての舵面を統合している．
 */
class GazeboFixedWingPlugin : public ModelPlugin
{
  using super = ModelPlugin;

  using CmdMsg = tobas_msgs::ControlSurfaceDeflections;
  using WindMsg = tobas_msgs::Wind;

public:
  explicit GazeboFixedWingPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string debug_pub_topic_;
  std::string deflections_sub_topic_;
  std::string wind_sub_topic_;
  double alt_0_;  // 基準点の幾何的高度
  double check_delay_threshold_;
  double auto_reset_time_thr_;
  tobas::VehicleParameters vehicle_params_;
  tobas::AerodynamicsCoefficients aero_coefs_;
  std::vector<tobas::ControlSurface> control_surfaces_;

  std::vector<physics::JointPtr> cs_joints_;       // 制御面のジョイントへのポインタ
  std::vector<SimpleJointModel> cs_angle_models_;  // 制御面の角度モデル

  double prev_alpha_;
  double prev_sim_time_;
  double last_cmd_time_;
  bool is_initialized_;
  bool cs_activated_;
  ignition::math::Vector3d wind_vel_W_;                   // 風速 [m/s]
  tobas_msgs::ControlSurfaceDeflections cs_deflections_;  // 舵角 [rad]
  tobas_gazebo_plugins::FixedWingDebug debug_msg_;

  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  // PubSub
  ros::Publisher debug_pub_;
  ros::Subscriber deflections_sub_;
  ros::Subscriber wind_sub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPubSub();
  void onUpdate(const common::UpdateInfo& info);
  void updateDeflections(const double& dt);
  ignition::math::Vector3d nonDimentionalAeroCoefs_Force(const double& alpha, const double& beta);
  ignition::math::Vector3d nonDimentionalAeroCoefs_Moment(
    const double& alpha,
    const double& beta,
    const double& alpha_rate,
    const double& V);
  double liftCoefficient(const double& alpha);
  double dragCoefficient(const double& alpha);
  double sideCoefficient(const double& beta);
  double rollCoefficient(const double& beta, const double& p, const double& r, const double& V);
  double pitchCoefficient(
    const double& alpha,
    const double& beta,
    const double& alpha_rate,
    const double& q,
    const double& V);
  double yawCoefficient(const double& beta, const double& p, const double& r, const double& V);
  double dynamicPressure(const double& V);

  void deflectionsCb(const CmdMsg& deflections);
  void windSpeedCb(const WindMsg& wind);

  /* ControlSurfaceをindexで並べ替えるためのキー． */
  static bool sortKey(const tobas::ControlSurface& l, const tobas::ControlSurface& r);
};
}  // namespace gazebo
