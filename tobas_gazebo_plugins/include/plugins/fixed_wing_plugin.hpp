#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <dh_std_tools/range.hpp>

#include <tobas_tools/fixed_wing_tools.hpp>
#include <tobas_msgs/ControlSurfaceDeflections.h>
#include <tobas_msgs/WindSpeed.h>

#include "../tobas_gazebo_plugins/simple_joint_model.hpp"

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "fixed_wing_plugin";
static constexpr double deg2rad = M_PI / 180.;
static constexpr double kMinAirSpeedThresh = 0.1;

// Default values
static constexpr char kDefaultDeflectionsSubTopic[] = "deflections";
static constexpr double kDefaultLowerStallAngle = -10. * deg2rad;
static constexpr double kDefaultUpperStallAngle = 20. * deg2rad;

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
  using WindMsg = tobas_msgs::WindSpeed;

public:
  GazeboFixedWingPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string deflections_sub_topic_;
  std::string wind_speed_sub_topic_;
  double ref_alt_;  // 基準点の幾何的高度
  VehicleParameters vehicle_params_;
  AerodynamicsCoefficients aero_coefs_;
  std::vector<ControlSurface> control_surfaces_;
  std::vector<SimpleJointModel> cs_angle_models_;

  uint32_t num_control_surfaces_;
  bool is_initialized_;
  double prev_sim_time_;
  double prev_alpha_;
  tobas_msgs::ControlSurfaceDeflections cs_deflections_;  // 舵角 [rad]
  ignition::math::Vector3d wind_speed_W_;                 // 風速 [m/s]

  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  ros::Subscriber deflections_sub_;
  ros::Subscriber wind_speed_sub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPubSub();
  void onUpdate(const common::UpdateInfo& info);
  void updateDeflections(double dt);
  ignition::math::Vector3d nonDimentionalAeroCoefs_Force(double alpha, double beta);
  ignition::math::Vector3d
  nonDimentionalAeroCoefs_Moment(double alpha, double beta, double alpha_rate, double V);
  double liftCoefficient(double alpha);
  double dragCoefficient(double alpha);
  double sideCoefficient(double beta);
  double rollCoefficient(double beta, double p, double r, double V);
  double pitchCoefficient(double alpha, double beta, double alpha_rate, double q, double V);
  double yawCoefficient(double beta, double p, double r, double V);
  double dynamicPressure(double V);

  void deflectionsCb(const CmdMsg& deflections);
  void windSpeedCb(const WindMsg& wind);
};
}  // namespace gazebo
