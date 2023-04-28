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

/**
 * @brief 舵面．軸が概ねY軸またはZ軸に平行であることを想定．
 */
struct ControlSurface
{
  uint32_t index;  // 舵角配列における添字
  dh_std::Range<double> limit;

  double c_lift_delta;      // [/deg]
  double c_drag_abs_delta;  // [/deg], 舵角の正負にかかわらず抗力が発生するモデル
  double c_side_delta;      // [/deg]
  double c_roll_delta;      // [/deg]
  double c_pitch_delta;     // [/deg]
  double c_yaw_delta;       // [/deg]
};

struct AerodynamicsCoefficients
{
  // Lift force
  double c_lift_0;      // [-]
  double c_lift_alpha;  // [/deg]

  // Drag force
  double c_drag_0;      // [-]
  double c_drag_alpha;  // [/deg]

  // Side force
  double c_side_beta;  // [/deg]

  // Roll moment
  double c_roll_beta;  // [/deg]
  double c_roll_p;     // [/rad]
  double c_roll_r;     // [/rad]

  // Pitch moment
  double c_pitch_0;           // [-]
  double c_pitch_alpha;       // [/deg]
  double c_pitch_abs_beta;    // [/deg]
  double c_pitch_alpha_rate;  // [/rad]
  double c_pitch_q;           // [/rad]

  // Yaw moment
  double c_yaw_beta;  // [/deg]
  double c_yaw_p;     // [/rad]
  double c_yaw_r;     // [/rad]

  std::vector<ControlSurface> control_surfaces;
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
  double ref_alt_;
  dh_std::Range<double> alpha_range_;
  uint32_t num_control_surfaces_;
  VehicleParameters vehicle_params_;
  AerodynamicsCoefficients aero_coefs_;

  bool is_initialized_;
  double prev_sim_time_;
  double prev_alpha_;
  tobas_msgs::ControlSurfaceDeflections cs_deflections_;  // 舵角 [deg]
  ignition::math::Vector3d wind_speed_W_;                 // 風速 [m/s]

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
    double alpha_rate,
    double V,
    const ignition::math::Vector3d& force_coefs);
  double liftCoefficient(double alpha);
  double dragCoefficient(double alpha);
  double sideCoefficient(double beta);
  double rollCoefficient(double beta, double p, double r, double V);
  double
  pitchCoefficient(double alpha, double beta, double alpha_rate, double q, double V, double C_z);
  double yawCoefficient(double beta, double p, double r, double V, double C_y);
  double dynamicPressure(double V);

  void deflectionsCb(const CmdMsg& deflections);
  void windSpeedCb(const WindMsg& wind);
};
}  // namespace gazebo
