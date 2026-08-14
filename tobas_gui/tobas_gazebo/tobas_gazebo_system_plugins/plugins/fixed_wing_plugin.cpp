// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

#include <tobas_constants/fixed_wing.hpp>
#include <tobas_drone_core/fixed_wing/fixed_wing.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_kdl.hpp>
#include <tobas_gazebo_conversions/gazebo_ros.hpp>
#include <tobas_gazebo_tools/math.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/fixed_wing.hpp>

#include <tobas_gazebo_msgs/msg/fixed_wing_debug.hpp>
#include <tobas_msgs_adapter/wind.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/world.hpp"

namespace ch = std::chrono;
namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
/**
 * @brief Plugin for aerodynamic forces acting on fixed-wing aircraft.
 * cf. Aircraft Flight Dynamics and Control: https://www.morikita.co.jp/books/mid/069081
 *
 * @note
 * All quantities use SI units.
 * Separate plugins per control surface were considered, but all control surfaces are integrated
 * because computing lift coefficients and related values would become cumbersome.
 */
class GazeboFixedWingPlugin : public BaseNode,
                              public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate
{
  // Constants
  static constexpr char kControlSurfaceKey[] = "controlSurface";
  static constexpr char kDebugTopic[] = "gazebo/fixed_wing_debug";

  using self = GazeboFixedWingPlugin;

public:
  explicit GazeboFixedWingPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string base_link_name_;
  double alt_0_;  // Geometric altitude of the reference point
  VehicleParameters vehicle_params_;
  AerodynamicCoefficients aero_coefs_;
  std::map<std::string, ControlSurface> control_surfaces_;

  std::shared_ptr<gz::sim::Link> base_link_;
  std::map<std::string, std::shared_ptr<gz::sim::Joint>> cs_joints_;  // Pointers to control-surface joints

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* vel_W_;
  const cmp::AngularVelocity* gyro_B_;

  double prev_alpha_ = 0.0;
  bool is_initialized_ = false;
  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;  // Wind velocity [m/s]

  // PubSub
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::FixedWingDebug> debug_pub_;
  ros2::SubscriberPtr<tobas_msgs::Wind> wind_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  double getDeflection(const gz::sim::EntityComponentManager& ecm, const std::string& link_name) const;

  double liftCoefficient(const gz::sim::EntityComponentManager& ecm, double alpha) const;
  double dragCoefficient(const gz::sim::EntityComponentManager& ecm, double alpha) const;
  double sideCoefficient(const gz::sim::EntityComponentManager& ecm, double beta) const;
  double rollCoefficient(const gz::sim::EntityComponentManager& ecm, double beta, double p, double r, double V) const;
  double pitchCoefficient(
    const gz::sim::EntityComponentManager& ecm,
    double alpha,
    double beta,
    double alpha_rate,
    double q,
    double V) const;
  double yawCoefficient(const gz::sim::EntityComponentManager& ecm, double beta, double p, double r, double V) const;

  gz::math::Vector3d
  nonDimentionalAeroCoefs_Force(const gz::sim::EntityComponentManager& ecm, double alpha, double beta) const;
  gz::math::Vector3d nonDimentionalAeroCoefs_Moment(
    const gz::sim::EntityComponentManager& ecm,
    double alpha,
    double beta,
    double alpha_rate,
    double V) const;

  void windSpeedCb(const tobas_msgs::Wind::ConstSharedPtr& wind);
};

GazeboFixedWingPlugin::GazeboFixedWingPlugin()
{
}

void GazeboFixedWingPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_fixed_wing_plugin", sdf);
  getSdfParams(sdf);

  // Get the world origin.
  const auto sc = getWorldSphericalCoordinates(ecm);
  if (!sc) {
    TOBAS_EXIT(sc.error());
  }
  alt_0_ = sc.value().ElevationReference();

  // Get robot model.
  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get base link.
  const auto base_link_entity = model.LinkByName(ecm, base_link_name_);
  base_link_ = std::make_shared<gz::sim::Link>(base_link_entity);
  if (!base_link_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find base link \"", base_link_name_, "\".");
  }

  // Create necessary components.
  TOBAS_CHECK(pose_W_ = getComponent<cmp::WorldPose>(base_link_entity, ecm));
  TOBAS_CHECK(vel_W_ = getComponent<cmp::WorldLinearVelocity>(base_link_entity, ecm));
  TOBAS_CHECK(gyro_B_ = getComponent<cmp::AngularVelocity>(base_link_entity, ecm));

  // Get control surface joint models.
  for (const auto& [link_name, _] : control_surfaces_) {
    // Get control surface joint.
    const auto joint_entity = findJointWithChildLink(ecm, link_name);
    if (!joint_entity.has_value()) {
      TOBAS_EXIT("Failed to find the parent joint of control surface link \"", link_name, "\".");
    }
    const auto joint = std::make_shared<gz::sim::Joint>(joint_entity.value());
    if (!joint->Valid(ecm)) {
      TOBAS_EXIT("Failed to find control surface \"", link_name, "\".");
    }

    // Get joint name.
    const auto joint_name = joint->Name(ecm).value();

    // Check joint type.
    const auto joint_type = joint->Type(ecm).value();
    if (joint_type != sdf::JointType::REVOLUTE) {
      TOBAS_EXIT("The type of control surface joint \"", link_name, "\" must be revolute.");
    }

    // Check joint limits.
    const auto joint_axis = joint->Axis(ecm).value().front();
    if (joint_axis.Lower() >= joint_axis.Upper()) {
      TOBAS_EXIT("The position limit of ", link_name, " is invalid.");
    }
    if (joint_axis.MaxVelocity() <= 0.0) {
      TOBAS_EXIT("The velocity limit of ", link_name, " must be positive.");
    }
    if (joint_axis.Effort() <= 0.0) {
      TOBAS_EXIT("The effort limit of ", link_name, " must be positive.");
    }

    // Add joint model.
    cs_joints_[link_name] = joint;
  }

  // Register ROS interfaces.
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::FixedWingDebug>(kDebugTopic);
  wind_sub_ = createSubscriber(kWindGtTopic, &self::windSpeedCb, this);
}

void GazeboFixedWingPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  // Base state.
  const auto& T_W_B = pose_W_->Data();
  const auto& P_W_B = T_W_B.Pos();
  const auto& R_W_B = T_W_B.Rot();

  // Aircraft velocity relative to wind.
  const auto vel_W = vel_W_->Data() - wind_vel_W_;
  auto vel_B = R_W_B.RotateVectorReverse(vel_W);

  // FLU -> FRD
  FLU2FRD(vel_B);

  // Relative wind velocity.
  const auto& u = vel_B.X();
  const auto& v = vel_B.Y();
  const auto& w = vel_B.Z();
  const auto V = std::max(vel_B.Length(), kMinAirSpeedThresh);  // Ensure `V > 0`.

  // Angle of attack and sideslip angle.
  const auto alpha = angleOfAttack(u, w);      // Angle of attack [rad].
  const auto beta = angleOfSideSlip(u, v, w);  // Sideslip angle [rad].

  // Check the angle-of-attack range.
  if (!vehicle_params_.alpha_limit.inRange(alpha)) {
    TOBAS_ERROR_THROTTLE(
      kErrorPeriod,
      "The angle of attack ",
      alpha,
      " is not within the valid range ",
      vehicle_params_.alpha_limit,
      ". The accuracy of the physics simulation may be compromised.");
  }

  // On the first update, only initialize variables and return.
  if (!is_initialized_) {
    prev_alpha_ = alpha;
    is_initialized_ = true;
    return;
  }

  // Update time and angle-of-attack rate.
  const auto dt = ch::duration<double>(info.dt).count();
  const auto alpha_rate = (alpha - prev_alpha_) / dt;  // [rad/s]
  prev_alpha_ = alpha;

  // Dimensionless aerodynamic coefficients.
  const auto force_coefs = nonDimentionalAeroCoefs_Force(ecm, alpha, beta);
  const auto moment_coefs = nonDimentionalAeroCoefs_Moment(ecm, alpha, beta, alpha_rate, V);

  // Precompute constant parts.
  const auto altitude = alt_0_ + P_W_B.Z();
  const auto rho = st::altitudeToDensity(altitude);
  const auto q_bar = dynamicPressure(rho, V);    // Dynamic pressure (p.15) [Pa].
  const auto& S = vehicle_params_.wing_surface;  // Main wing area [m^2].

  // Aerodynamic force acting on the aerodynamic center (1.8-1)
  auto force_B = q_bar * S * force_coefs;  // [N]

  // Aerodynamic moment acting on the aerodynamic center (1.8-7)
  const auto& C_l = moment_coefs.X();                                             // [-]
  const auto& C_m = moment_coefs.Y();                                             // [-]
  const auto& C_n = moment_coefs.Z();                                             // [-]
  const auto& b = vehicle_params_.wing_span;                                      // [m]
  const auto& c_bar = vehicle_params_.mac;                                        // [m]
  auto torque_B = q_bar * S * gz::math::Vector3d(b * C_l, c_bar * C_m, b * C_n);  // [Nm]

  // FRD coordinates -> FLU coordinates
  FRD2FLU(force_B);
  FRD2FLU(torque_B);

  // Convert to the world coordinate system.
  const auto force_W = R_W_B.RotateVector(force_B);
  const auto torque_W = R_W_B.RotateVector(torque_B);

  // Apply aerodynamic force.
  gz::math::Vector3d B_Pos_BC;
  vectorKDLToGazebo(vehicle_params_.ac, B_Pos_BC);
  base_link_->AddWorldWrench(ecm, force_W, torque_W, B_Pos_BC);

  // Publish debug messages.
  auto debug_msg = std::make_unique<tobas_gazebo_msgs::msg::FixedWingDebug>();
  ros2::timeChronoToMsg(info.simTime, debug_msg->header.stamp);
  vectorGazeboToRos(vel_B, debug_msg->relative_body_velocity);
  debug_msg->alpha = alpha;
  debug_msg->beta = beta;
  vectorGazeboToRos(force_B, debug_msg->air_force);
  vectorGazeboToRos(torque_B, debug_msg->air_moment);
  debug_pub_->publish(std::move(debug_msg));
}

void GazeboFixedWingPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "baseLinkName", base_link_name_);

  // Vehicle
  getSdfParam(sdf, "wingSurface", vehicle_params_.wing_surface, kPositive);
  getSdfParam(sdf, "wingSpan", vehicle_params_.wing_span, kPositive);
  getSdfParam(sdf, "meanAerodynamicChord", vehicle_params_.mac, kPositive);

  gz::math::Vector3d ac;
  getSdfParam(sdf, "aerodynamicCenter", ac);
  vectorGazeboToKDL(ac, vehicle_params_.ac);

  getSdfParam(sdf, "lowerStallAngle", vehicle_params_.alpha_limit.lower);
  getSdfParam(sdf, "upperStallAngle", vehicle_params_.alpha_limit.upper);
  if (!vehicle_params_.alpha_limit.isValid()) {
    TOBAS_EXIT("Invalid stall angles");
  }

  // Aerodynamics
  getSdfParam(sdf, "cLift0", aero_coefs_.c_lift_0, kPositive);
  getSdfParam(sdf, "cLiftAlpha", aero_coefs_.c_lift_alpha, kPositive);
  getSdfParam(sdf, "cDrag0", aero_coefs_.c_drag_0, kPositive);
  getSdfParam(sdf, "cDragAlpha", aero_coefs_.c_drag_alpha, kPositive);
  getSdfParam(sdf, "cSideBeta", aero_coefs_.c_side_beta, kNegative);

  getSdfParam(sdf, "cRollBeta", aero_coefs_.c_roll_beta, kNegative);
  getSdfParam(sdf, "cRollP", aero_coefs_.c_roll_p, kNegative);
  getSdfParam(sdf, "cRollR", aero_coefs_.c_roll_r);

  getSdfParam(sdf, "cPitch0", aero_coefs_.c_pitch_0);
  getSdfParam(sdf, "cPitchAlpha", aero_coefs_.c_pitch_alpha, kNegative);
  getSdfParam(sdf, "cPitchAbsBeta", aero_coefs_.c_pitch_abs_beta);
  getSdfParam(sdf, "cPitchAlphaRate", aero_coefs_.c_pitch_alpha_rate);
  getSdfParam(sdf, "cPitchQ", aero_coefs_.c_pitch_q, kNegative);

  getSdfParam(sdf, "cYawBeta", aero_coefs_.c_yaw_beta);
  getSdfParam(sdf, "cYawP", aero_coefs_.c_yaw_p);
  getSdfParam(sdf, "cYawR", aero_coefs_.c_yaw_r, kNegative);

  // ControlSurface
  if (sdf->HasElement(kControlSurfaceKey)) {
    std::unordered_set<std::string> joint_names;
    auto cs_elem = sdf->FindElement(kControlSurfaceKey);

    while (cs_elem) {
      ControlSurface cs;

      getSdfParam(cs_elem, "jointName", cs.link_name);
      if (joint_names.contains(cs.link_name)) {
        TOBAS_EXIT("The joint names of each control surface must be unique.");
      }

      getSdfParam(cs_elem, "cLiftDelta", cs.c_lift_delta, 0.0);
      getSdfParam(cs_elem, "cDragAbsDelta", cs.c_drag_abs_delta, 0.0);
      getSdfParam(cs_elem, "cSideDelta", cs.c_side_delta, 0.0);
      getSdfParam(cs_elem, "cRollDelta", cs.c_roll_delta, 0.0);
      getSdfParam(cs_elem, "cPitchDelta", cs.c_pitch_delta, 0.0);
      getSdfParam(cs_elem, "cYawDelta", cs.c_yaw_delta, 0.0);

      joint_names.emplace(cs.link_name);
      control_surfaces_[cs.link_name] = cs;
      cs_elem = cs_elem->GetNextElement(kControlSurfaceKey);
    }
  }
}

double
GazeboFixedWingPlugin::getDeflection(const gz::sim::EntityComponentManager& ecm, const std::string& link_name) const
{
  return cs_joints_.at(link_name)->Position(ecm).value().front();
}

double GazeboFixedWingPlugin::liftCoefficient(const gz::sim::EntityComponentManager& ecm, double alpha) const
{
  // Angle of attack.
  auto C_L = aero_coefs_.c_lift_0 + aero_coefs_.c_lift_alpha * alpha;

  // Control surfaces.
  for (const auto& [link_name, _] : control_surfaces_) {
    C_L += control_surfaces_.at(link_name).c_lift_delta * getDeflection(ecm, link_name);
  }

  return C_L;
}

double GazeboFixedWingPlugin::dragCoefficient(const gz::sim::EntityComponentManager& ecm, double alpha) const
{
  // Angle of attack.
  auto C_D = aero_coefs_.c_drag_0 + aero_coefs_.c_drag_alpha * alpha;

  // Control surfaces.
  for (const auto& [link_name, _] : control_surfaces_) {
    C_D += control_surfaces_.at(link_name).c_drag_abs_delta * std::abs(getDeflection(ecm, link_name));
  }

  return C_D;
}

double GazeboFixedWingPlugin::sideCoefficient(const gz::sim::EntityComponentManager& ecm, double beta) const
{
  // Sideslip angle.
  auto C_S = aero_coefs_.c_side_beta * beta;

  // Control surfaces.
  for (const auto& [link_name, _] : control_surfaces_) {
    C_S += control_surfaces_.at(link_name).c_side_delta * getDeflection(ecm, link_name);
  }

  return C_S;
}

double GazeboFixedWingPlugin::rollCoefficient(
  const gz::sim::EntityComponentManager& ecm,
  double beta,
  double p,
  double r,
  double V) const
{
  // Sideslip angle.
  auto C_l = aero_coefs_.c_roll_beta * beta;

  // Angular velocity.
  const auto& b = vehicle_params_.wing_span;
  C_l += b / (2 * V) * (aero_coefs_.c_roll_p * p + aero_coefs_.c_roll_r * r);

  // Control surfaces.
  for (const auto& [link_name, _] : control_surfaces_) {
    C_l += control_surfaces_.at(link_name).c_roll_delta * getDeflection(ecm, link_name);
  }

  return C_l;
}

double GazeboFixedWingPlugin::pitchCoefficient(
  const gz::sim::EntityComponentManager& ecm,
  double alpha,
  double beta,
  double alpha_rate,
  double q,
  double V) const
{
  // Angle of attack and sideslip angle.
  auto C_m = aero_coefs_.c_pitch_0 + aero_coefs_.c_pitch_alpha * alpha;
  C_m += aero_coefs_.c_pitch_abs_beta * std::abs(beta);

  // Angular velocity.
  const auto& c = vehicle_params_.mac;
  C_m += c / (2 * V) * (aero_coefs_.c_pitch_alpha_rate * alpha_rate + aero_coefs_.c_pitch_q * q);

  // Control surfaces.
  for (const auto& [link_name, _] : control_surfaces_) {
    C_m += control_surfaces_.at(link_name).c_pitch_delta * getDeflection(ecm, link_name);
  }

  return C_m;
}

double GazeboFixedWingPlugin::yawCoefficient(
  const gz::sim::EntityComponentManager& ecm,
  double beta,
  double p,
  double r,
  double V) const
{
  // Sideslip angle.
  auto C_n = aero_coefs_.c_yaw_beta * beta;

  // Angular velocity.
  const auto& b = vehicle_params_.wing_span;
  C_n += b / (2 * V) * (aero_coefs_.c_yaw_p * p + aero_coefs_.c_yaw_r * r);

  // Control surfaces.
  for (const auto& [link_name, _] : control_surfaces_) {
    C_n += control_surfaces_.at(link_name).c_yaw_delta * getDeflection(ecm, link_name);
  }

  return C_n;
}

gz::math::Vector3d GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Force(
  const gz::sim::EntityComponentManager& ecm,
  double alpha,
  double beta) const
{
  const auto C_L = liftCoefficient(ecm, alpha);  // Lift coefficient (1.8-3)
  const auto C_D = dragCoefficient(ecm, alpha);  // Drag coefficient (1.8-3)
  const auto C_S = sideCoefficient(ecm, beta);   // Side-force coefficient (1.8-5)

  const auto cos_alpha = std::cos(alpha);
  const auto sin_alpha = std::sin(alpha);

  const auto C_x = -C_D * cos_alpha + C_L * sin_alpha;  // (1.8-4)
  const auto C_z = -C_L * cos_alpha - C_D * sin_alpha;  // (1.8-4)
  const auto C_y = C_S;                                 // (1.8-5)

  return gz::math::Vector3d(C_x, C_y, C_z);
}

gz::math::Vector3d GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Moment(
  const gz::sim::EntityComponentManager& ecm,
  double alpha,
  double beta,
  double alpha_rate,
  double V) const
{
  // Angular velocity.
  auto gyro_B = gyro_B_->Data();
  FLU2FRD(gyro_B);
  const auto p = gyro_B.X();
  const auto q = gyro_B.Y();
  const auto r = gyro_B.Z();

  // (1.8-9): No moment correction term because force is applied at the center of lift.
  const auto C_l = rollCoefficient(ecm, beta, p, r, V);
  const auto C_m = pitchCoefficient(ecm, alpha, beta, alpha_rate, q, V);
  const auto C_n = yawCoefficient(ecm, beta, p, r, V);

  return gz::math::Vector3d(C_l, C_m, C_n);
}

void GazeboFixedWingPlugin::windSpeedCb(const tobas_msgs::Wind::ConstSharedPtr& wind)
{
  vectorKDLToGazebo(wind->vel, wind_vel_W_);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::GazeboFixedWingPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
