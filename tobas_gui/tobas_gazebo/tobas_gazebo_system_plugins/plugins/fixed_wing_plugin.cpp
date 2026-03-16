#include <ranges>

#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

#include <tobas_constants/fixed_wing.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_drone_core/fixed_wing/fixed_wing.hpp>
#include <tobas_drone_tools/utils/fixed_wing_tools.hpp>
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

#include <tobas_gazebo_msgs/msg/fixed_wing_debug.hpp>
#include <tobas_msgs_adapter/wind.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/world.hpp"

namespace ch = std::chrono;
namespace cmp = gz::sim::components;

namespace gazebo
{
/**
 * @brief 固定翼機に作用する空気力のプラグイン．
 * cf. 航空機の飛行力学と制御: https://www.morikita.co.jp/books/mid/069081
 *
 * @note
 * 全てSI単位系を用いる．
 * 舵面ごとに別々のプラグインにすることも検討したが，揚力係数等の計算が面倒になるため全ての舵面を統合している．
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
  double alt_0_;  // 基準点の幾何的高度
  tobas::VehicleParameters vehicle_params_;
  tobas::AerodynamicCoefficients aero_coefs_;
  std::map<std::string, tobas::ControlSurface> control_surfaces_;

  std::shared_ptr<gz::sim::Link> base_link_;
  std::map<std::string, std::shared_ptr<gz::sim::Joint>> cs_joints_;  // 制御面のジョイントへのポインタ

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* vel_W_;
  const cmp::AngularVelocity* gyro_B_;

  double prev_alpha_ = 0.;
  bool is_initialized_ = false;
  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;  // 風速 [m/s]

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

  // Get the world origin
  const auto sc = getWorldSphericalCoordinates(ecm);
  if (!sc) {
    TOBAS_EXIT(sc.error());
  }
  alt_0_ = sc.value().ElevationReference();

  // Get robot model
  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get base link
  const auto base_link_entity = model.LinkByName(ecm, base_link_name_);
  base_link_ = std::make_shared<gz::sim::Link>(base_link_entity);
  if (!base_link_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find base link \"", base_link_name_, "\".");
  }

  // Create necessary components
  TOBAS_CHECK(pose_W_ = getComponent<cmp::WorldPose>(base_link_entity, ecm));
  TOBAS_CHECK(vel_W_ = getComponent<cmp::WorldLinearVelocity>(base_link_entity, ecm));
  TOBAS_CHECK(gyro_B_ = getComponent<cmp::AngularVelocity>(base_link_entity, ecm));

  // Get control surface joint models
  for (const auto& [link_name, _] : control_surfaces_) {
    // Get control surface joint
    const auto joint_entity = findJointWithChildLink(ecm, link_name);
    if (!joint_entity.has_value()) {
      TOBAS_EXIT("Failed to find the parent joint of control surface link \"", link_name, "\".");
    }
    const auto joint = std::make_shared<gz::sim::Joint>(joint_entity.value());
    if (!joint->Valid(ecm)) {
      TOBAS_EXIT("Failed to find control surface \"", link_name, "\".");
    }

    // Get joint name
    const auto joint_name = joint->Name(ecm).value();

    // Check joint type
    const auto joint_type = joint->Type(ecm).value();
    if (joint_type != sdf::JointType::REVOLUTE) {
      TOBAS_EXIT("The type of control surface joint \"", link_name, "\" must be revolute.");
    }

    // Check joint limits
    const auto joint_axis = joint->Axis(ecm).value().front();
    if (joint_axis.Lower() >= joint_axis.Upper()) {
      TOBAS_EXIT("The position limit of ", link_name, " is invalid.");
    }
    if (joint_axis.MaxVelocity() <= 0.) {
      TOBAS_EXIT("The velocity limit of ", link_name, " must be positive.");
    }
    if (joint_axis.Effort() <= 0.) {
      TOBAS_EXIT("The effort limit of ", link_name, " must be positive.");
    }

    // Add joint model
    cs_joints_[link_name] = joint;
  }

  // Register ROS interfaces
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::FixedWingDebug>(kDebugTopic);
  wind_sub_ = createSubscriber(kWindGtTopic, &self::windSpeedCb, this);
}

void GazeboFixedWingPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  // ベースの状態
  const auto& T_W_B = pose_W_->Data();
  const auto& P_W_B = T_W_B.Pos();
  const auto& R_W_B = T_W_B.Rot();

  // 風に対する相対的な機体速度
  const auto vel_W = vel_W_->Data() - wind_vel_W_;
  auto vel_B = R_W_B.RotateVectorReverse(vel_W);

  // FLU -> FRD
  FLU2FRD(vel_B);

  // 相対風速
  const auto& u = vel_B.X();
  const auto& v = vel_B.Y();
  const auto& w = vel_B.Z();
  const auto V = std::max(vel_B.Length(), tobas::kMinAirSpeedThresh);  // V > 0 を保証する

  // 迎角と横滑り角
  const auto alpha = tobas::angleOfAttack(u, w);      // 迎角 [rad]
  const auto beta = tobas::angleOfSideSlip(u, v, w);  // 横滑り角 [rad]

  // 迎角の範囲チェック
  if (!vehicle_params_.alpha_limit.inRange(alpha)) {
    TOBAS_ERROR_THROTTLE(
      kErrorPeriod,
      "The angle of attack ",
      alpha,
      " is not within the valid range ",
      vehicle_params_.alpha_limit,
      ". The accuracy of the physics simulation may be compromised.");
  }

  // 最初は変数の初期化だけして終了
  if (!is_initialized_) {
    prev_alpha_ = alpha;
    is_initialized_ = true;
    return;
  }

  // 時刻と迎角の変化率を更新
  const auto dt = ch::duration<double>(info.dt).count();
  const auto alpha_rate = (alpha - prev_alpha_) / dt;  // [rad/s]
  prev_alpha_ = alpha;

  // 無次元空力係数
  const auto force_coefs = nonDimentionalAeroCoefs_Force(ecm, alpha, beta);
  const auto moment_coefs = nonDimentionalAeroCoefs_Moment(ecm, alpha, beta, alpha_rate, V);

  // 定数部分を計算しておく
  const auto altitude = alt_0_ + P_W_B.Z();
  const auto rho = tbs::altitudeToDensity(altitude);
  const auto q_bar = tobas::dynamicPressure(rho, V);  // 動圧 (p.15) [Pa]
  const auto& S = vehicle_params_.wing_surface;       // 主翼面積 [m^2]

  // 空力中心に働く空気力 (1.8-1)
  auto force_B = q_bar * S * force_coefs;  // [N]

  // 空力中心に働く空気モーメント (1.8-7)
  const auto& C_l = moment_coefs.X();                                             // [-]
  const auto& C_m = moment_coefs.Y();                                             // [-]
  const auto& C_n = moment_coefs.Z();                                             // [-]
  const auto& b = vehicle_params_.wing_span;                                      // [m]
  const auto& c_bar = vehicle_params_.mac;                                        // [m]
  auto torque_B = q_bar * S * gz::math::Vector3d(b * C_l, c_bar * C_m, b * C_n);  // [Nm]

  // FRD coordinates -> FLU coordinates
  FRD2FLU(force_B);
  FRD2FLU(torque_B);

  // 世界座標系に変換
  const auto force_W = R_W_B.RotateVector(force_B);
  const auto torque_W = R_W_B.RotateVector(torque_B);

  // 空気力を作用させる
  gz::math::Vector3d B_Pos_BC;
  vectorKDLToGazebo(vehicle_params_.ac, B_Pos_BC);
  base_link_->AddWorldWrench(ecm, force_W, torque_W, B_Pos_BC);

  // デバッグ用メッセージを発行
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
      tobas::ControlSurface cs;

      getSdfParam(cs_elem, "jointName", cs.link_name);
      if (joint_names.contains(cs.link_name)) {
        TOBAS_EXIT("The joint names of each control surface must be unique.");
      }

      getSdfParam(cs_elem, "cLiftDelta", cs.c_lift_delta, 0.);
      getSdfParam(cs_elem, "cDragAbsDelta", cs.c_drag_abs_delta, 0.);
      getSdfParam(cs_elem, "cSideDelta", cs.c_side_delta, 0.);
      getSdfParam(cs_elem, "cRollDelta", cs.c_roll_delta, 0.);
      getSdfParam(cs_elem, "cPitchDelta", cs.c_pitch_delta, 0.);
      getSdfParam(cs_elem, "cYawDelta", cs.c_yaw_delta, 0.);

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
  // 迎角
  auto C_L = aero_coefs_.c_lift_0 + aero_coefs_.c_lift_alpha * alpha;

  // 舵面
  for (const auto& [link_name, _] : control_surfaces_) {
    C_L += control_surfaces_.at(link_name).c_lift_delta * getDeflection(ecm, link_name);
  }

  return C_L;
}

double GazeboFixedWingPlugin::dragCoefficient(const gz::sim::EntityComponentManager& ecm, double alpha) const
{
  // 迎角
  auto C_D = aero_coefs_.c_drag_0 + aero_coefs_.c_drag_alpha * alpha;

  // 舵面
  for (const auto& [link_name, _] : control_surfaces_) {
    C_D += control_surfaces_.at(link_name).c_drag_abs_delta * std::abs(getDeflection(ecm, link_name));
  }

  return C_D;
}

double GazeboFixedWingPlugin::sideCoefficient(const gz::sim::EntityComponentManager& ecm, double beta) const
{
  // 横滑り角
  auto C_S = aero_coefs_.c_side_beta * beta;

  // 舵面
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
  // 横滑り角
  auto C_l = aero_coefs_.c_roll_beta * beta;

  // 角速度
  const auto& b = vehicle_params_.wing_span;
  C_l += b / (2 * V) * (aero_coefs_.c_roll_p * p + aero_coefs_.c_roll_r * r);

  // 舵面
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
  // 迎角，横滑り角
  auto C_m = aero_coefs_.c_pitch_0 + aero_coefs_.c_pitch_alpha * alpha;
  C_m += aero_coefs_.c_pitch_abs_beta * std::abs(beta);

  // 角速度
  const auto& c = vehicle_params_.mac;
  C_m += c / (2 * V) * (aero_coefs_.c_pitch_alpha_rate * alpha_rate + aero_coefs_.c_pitch_q * q);

  // 舵面
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
  // 横滑り角
  auto C_n = aero_coefs_.c_yaw_beta * beta;

  // 角速度
  const auto& b = vehicle_params_.wing_span;
  C_n += b / (2 * V) * (aero_coefs_.c_yaw_p * p + aero_coefs_.c_yaw_r * r);

  // 舵面
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
  const auto C_L = liftCoefficient(ecm, alpha);  // 揚力係数 (1.8-3)
  const auto C_D = dragCoefficient(ecm, alpha);  // 抗力係数 (1.8-3)
  const auto C_S = sideCoefficient(ecm, beta);   // 横力係数 (1.8-5)

  const auto cos_alpha = cos(alpha);
  const auto sin_alpha = sin(alpha);

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
  // 角速度
  auto gyro_B = gyro_B_->Data();
  FLU2FRD(gyro_B);
  const auto p = gyro_B.X();
  const auto q = gyro_B.Y();
  const auto r = gyro_B.Z();

  // (1.8-9): 揚力中心に力をかけるためモーメントの補正項はなし
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

GZ_ADD_PLUGIN(
  gazebo::GazeboFixedWingPlugin,
  gz::sim::System,
  gazebo::GazeboFixedWingPlugin::ISystemConfigure,
  gazebo::GazeboFixedWingPlugin::ISystemPreUpdate)
