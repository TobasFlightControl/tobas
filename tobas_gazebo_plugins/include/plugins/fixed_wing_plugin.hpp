#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <dh_std_tools/range.hpp>

#include <tobas_msgs/ControlSurfaceDeflections.h>
#include <tobas_msgs/WindSpeed.h>

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "fixed_wing_plugin";
static constexpr double kMinAirSpeedThresh = 0.1;

struct VehicleParameters
{
  double wing_surface;  // 主翼面積
  double wing_span;     // 翼幅
  double mac;           // 平均空力翼弦 (Mean Aerodynamic Chord)
};

struct AerodynamicsCoefficients
{
  double c_lift_0;      // [-]
  double c_lift_alpha;  // [/deg]
  double c_drag_0;      // [-]
  double c_drag_alpha;  // [/deg]
};

/**
 * @brief 固定翼機に作用する空気力のプラグイン．
 * cf. 航空機の飛行力学と制御: https://www.morikita.co.jp/books/mid/069081
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
  dh_std::Range<double> alpha_range_;
  VehicleParameters vehicle_params_;
  AerodynamicsCoefficients aero_coefs_;

  ignition::math::Vector3d wind_speed_W_;

  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  ros::Subscriber deflections_sub_;
  ros::Subscriber wind_speed_sub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPubSub();
  void onUpdate(const common::UpdateInfo& info);
  ignition::math::Vector3d nonDimentionalAeroCoefs_Force(double alpha, double beta);
  ignition::math::Vector3d nonDimentionalAeroCoefs_Moment(
    double alpha,
    double beta,
    const ignition::math::Vector3d& force_coefs);
  double liftCoefficient(double alpha);
  double dragCoefficient(double alpha);
  double sideCoefficient(double beta);
  double rollCoefficient(double beta);
  double pitchCoefficient(double alpha, double beta, double C_z);
  double yawCoefficient(double beta, double C_y);
  double dynamicPressure();

  void deflectionsCb(const CmdMsg& cmd);
  void windSpeedCb(const WindMsg& wind);
};
}  // namespace gazebo
