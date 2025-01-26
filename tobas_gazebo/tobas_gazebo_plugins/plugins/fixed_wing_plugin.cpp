#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>

#include <tobas_std_tools/range.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/fixed_wing/fixed_wing.hpp>
#include <tobas_drone_tools/utils/fixed_wing_tools.hpp>
#include <tobas_msgs_adapter/wind.hpp>
#include <tobas_msgs/msg/control_surface_deflections.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/math.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_gazebo_msgs/msg/fixed_wing_debug.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/simple_joint_model.hpp"

using namespace std;
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
  static constexpr char kDebugPubTopic[] = "gazebo/fixed_wing_debug";
  static constexpr double kAutoResetTimeout = 0.5;  // [s]

  // Default values
  static constexpr double kDefaultLowerStallAngle = -10 * tobas_std::kDeg2Rad;
  static constexpr double kDefaultUpperStallAngle = 20 * tobas_std::kDeg2Rad;

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
  string link_name_;
  double alt_0_;  // 基準点の幾何的高度
  tobas::VehicleParameters vehicle_params_;
  tobas::AerodynamicCoefficients aero_coefs_;
  vector<tobas::ControlSurface> control_surfaces_;

  shared_ptr<gz::sim::Link> link_;
  vector<shared_ptr<gz::sim::Joint>> cs_joints_;  // 制御面のジョイントへのポインタ
  vector<SimpleJointModel> cs_angle_models_;      // 制御面の角度モデル

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* vel_W_;
  const cmp::AngularVelocity* gyro_B_;

  double prev_alpha_ = 0.;
  chrono::steady_clock::duration prev_sim_time_;
  chrono::steady_clock::duration last_cmd_time_;
  bool is_initialized_ = false;
  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;                   // 風速 [m/s]
  tobas_msgs::msg::ControlSurfaceDeflections::ConstSharedPtr cs_deflections_;  // 舵角 [rad]

  // PubSub
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::FixedWingDebug> debug_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::ControlSurfaceDeflections> deflections_sub_;
  ros2::SubscriberPtr<tobas_msgs::Wind> wind_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerPubSub();

  void updateDeflections(gz::sim::EntityComponentManager& ecm, double dt);

  gz::math::Vector3d nonDimentionalAeroCoefs_Force(double alpha, double beta) const;
  gz::math::Vector3d nonDimentionalAeroCoefs_Moment(double alpha, double beta, double alpha_rate, double V) const;
  double liftCoefficient(double alpha) const;
  double dragCoefficient(double alpha) const;
  double sideCoefficient(double beta) const;
  double rollCoefficient(double beta, double p, double r, double V) const;
  double pitchCoefficient(double alpha, double beta, double alpha_rate, double q, double V) const;
  double yawCoefficient(double beta, double p, double r, double V) const;

  void deflectionsCb(const tobas_msgs::msg::ControlSurfaceDeflections::ConstSharedPtr& deflections);
  void windSpeedCb(const tobas_msgs::Wind::ConstSharedPtr& wind);

  /* ControlSurfaceをindexで並べ替えるためのキー． */
  static bool sortKey(const tobas::ControlSurface& l, const tobas::ControlSurface& r);
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

  // Get robot model
  gz::sim::Model model(model_entity);
  if (!model.Valid(ecm))
    TOBAS_EXIT("Failed to find model.");

  // Get body link
  const auto link_entity = model.LinkByName(ecm, link_name_);
  link_ = make_shared<gz::sim::Link>(link_entity);
  if (!link_->Valid(ecm))
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  // Create necessary components
  pose_W_ = getComponent<cmp::WorldPose>(link_entity, ecm);
  vel_W_ = getComponent<cmp::WorldLinearVelocity>(link_entity, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link_entity, ecm);

  // Get control surface joint models
  for (const auto& cs : control_surfaces_)
  {
    // Get control surface joint
    const auto joint_entity = findJointWithChildLink(ecm, cs.link_name);
    if (!joint_entity.has_value())
      TOBAS_EXIT("Failed to find the parent joint of control surface link \"", cs.link_name, "\".");
    const auto joint = make_shared<gz::sim::Joint>(joint_entity.value());
    if (!joint->Valid(ecm))
      TOBAS_EXIT("Failed to find control surface \"", cs.link_name, "\".");

    // Get joint name
    const auto joint_name = joint->Name(ecm).value();

    // Check joint type
    const auto joint_type = joint->Type(ecm).value();
    if (joint_type != sdf::JointType::REVOLUTE)
      TOBAS_EXIT("The type of control surface joint \"", joint_name, "\" must be revolute.");

    // Check joint limits
    const auto joint_axis = joint->Axis(ecm).value().at(0);
    if (joint_axis.Lower() >= joint_axis.Upper())
      TOBAS_EXIT("The position limit of ", joint_name, " is invalid.");
    if (joint_axis.MaxVelocity() <= 0.)
      TOBAS_EXIT("The velocity limit of ", joint_name, " must be positive.");
    if (joint_axis.Effort() <= 0.)
      TOBAS_EXIT("The effort limit of ", joint_name, " must be positive.");

    // Add joint model
    cs_joints_.push_back(joint);
    cs_angle_models_.emplace_back(joint_axis.Lower(), joint_axis.Upper(), joint_axis.MaxVelocity());
  }

  registerPubSub();
}

void GazeboFixedWingPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero, NON_NEGATIVE);

  // Vehicle
  getSdfParam(sdf, "wingSurface", vehicle_params_.wing_surface, POSITIVE);
  getSdfParam(sdf, "wingSpan", vehicle_params_.wing_span, POSITIVE);
  getSdfParam(sdf, "meanAerodynamicChord", vehicle_params_.mac, POSITIVE);

  gz::math::Vector3d ac;
  getSdfParam(sdf, "aerodynamicCenter", ac);
  vectorGazeboToKDL(ac, vehicle_params_.ac);

  getSdfParam(sdf, "lowerStallAngle", vehicle_params_.alpha_limit.lower, kDefaultLowerStallAngle);
  getSdfParam(sdf, "upperStallAngle", vehicle_params_.alpha_limit.upper, kDefaultUpperStallAngle);
  if (!vehicle_params_.alpha_limit.isValid())
    TOBAS_EXIT("Invalid stall angles");

  // Aerodynamics
  getSdfParam(sdf, "cLift0", aero_coefs_.c_lift_0, POSITIVE);
  getSdfParam(sdf, "cLiftAlpha", aero_coefs_.c_lift_alpha, POSITIVE);
  getSdfParam(sdf, "cDrag0", aero_coefs_.c_drag_0, POSITIVE);
  getSdfParam(sdf, "cDragAlpha", aero_coefs_.c_drag_alpha, POSITIVE);
  getSdfParam(sdf, "cSideBeta", aero_coefs_.c_side_beta, NEGATIVE);

  getSdfParam(sdf, "cRollBeta", aero_coefs_.c_roll_beta, NEGATIVE);
  getSdfParam(sdf, "cRollP", aero_coefs_.c_roll_p, NEGATIVE);
  getSdfParam(sdf, "cRollR", aero_coefs_.c_roll_r);

  getSdfParam(sdf, "cPitch0", aero_coefs_.c_pitch_0);
  getSdfParam(sdf, "cPitchAlpha", aero_coefs_.c_pitch_alpha, NEGATIVE);
  getSdfParam(sdf, "cPitchAbsBeta", aero_coefs_.c_pitch_abs_beta);
  getSdfParam(sdf, "cPitchAlphaRate", aero_coefs_.c_pitch_alpha_rate);
  getSdfParam(sdf, "cPitchQ", aero_coefs_.c_pitch_q, NEGATIVE);

  getSdfParam(sdf, "cYawBeta", aero_coefs_.c_yaw_beta);
  getSdfParam(sdf, "cYawP", aero_coefs_.c_yaw_p);
  getSdfParam(sdf, "cYawR", aero_coefs_.c_yaw_r, NEGATIVE);

  // ControlSurface
  if (sdf->HasElement("controlSurface"))
  {
    unordered_set<int> indexes;
    auto cs_elem = sdf->FindElement("controlSurface");

    while (cs_elem)
    {
      tobas::ControlSurface cs;

      getSdfParam(cs_elem, "channel", cs.channel, NON_NEGATIVE);
      if (indexes.contains(cs.channel))
        TOBAS_EXIT("The channel of each control surface must be unique.");

      getSdfParam(cs_elem, "linkName", cs.link_name);

      getSdfParam(cs_elem, "cLiftDelta", cs.c_lift_delta, 0.);
      getSdfParam(cs_elem, "cDragAbsDelta", cs.c_drag_abs_delta, 0.);
      getSdfParam(cs_elem, "cSideDelta", cs.c_side_delta, 0.);
      getSdfParam(cs_elem, "cRollDelta", cs.c_roll_delta, 0.);
      getSdfParam(cs_elem, "cPitchDelta", cs.c_pitch_delta, 0.);
      getSdfParam(cs_elem, "cYawDelta", cs.c_yaw_delta, 0.);

      indexes.emplace(cs.channel);
      control_surfaces_.push_back(cs);
      cs_elem = cs_elem->GetNextElement("controlSurface");
    }

    for (size_t i = 0; i < indexes.size(); ++i)
      if (!indexes.contains(static_cast<int>(i)))
        TOBAS_EXIT("controlSurface channel mismatch.");
  }

  // index順に並べ替える
  sort(control_surfaces_.begin(), control_surfaces_.end(), sortKey);
}

void GazeboFixedWingPlugin::registerPubSub()
{
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::FixedWingDebug>(kDebugPubTopic);

  deflections_sub_ = createSubscriber(tobas::kDeflectionCmdTopic, &self::deflectionsCb, this);
  wind_sub_ = createSubscriber(kWindGtTopic, &self::windSpeedCb, this);
}

void GazeboFixedWingPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  // 最新のコマンドからの経過時間を確認
  const auto secs_from_last_cmd = chrono::duration<double>(info.simTime - last_cmd_time_).count();
  if (cs_deflections_ != nullptr && secs_from_last_cmd > kAutoResetTimeout)
  {
    cs_deflections_ = nullptr;
    TOBAS_INFO(
      "Deflection angles of control surfaces are automatically reset because ", kAutoResetTimeout,
      " seconds have elapsed since the last command.");
  }

  // ベースの状態
  const auto& T_W_B = pose_W_->Data();
  const auto& P_W_B = T_W_B.Pos();
  const auto& R_W_B = T_W_B.Rot();

  // 風に対する相対的な機体速度
  const auto vel_W = vel_W_->Data() - wind_vel_W_;
  auto vel_B = R_W_B.RotateVectorReverse(vel_W);

  // NWU -> NED
  NWU2NED(vel_B);

  // 相対風速
  const auto& u = vel_B.X();
  const auto& v = vel_B.Y();
  const auto& w = vel_B.Z();
  const auto V = max(vel_B.Length(), tobas::kMinAirSpeedThresh);  // V > 0 を保証する

  // 迎角と横滑り角
  const auto alpha = tobas::angleOfAttack(u, w);      // 迎角 [rad]
  const auto beta = tobas::angleOfSideSlip(u, v, w);  // 横滑り角 [rad]

  // 迎角の範囲チェック
  if (!vehicle_params_.alpha_limit.inRange(alpha))
  {
    TOBAS_ERROR_THROTTLE(
      kErrorPeriod, "The angle of attack ", alpha, " is not within the valid range ", vehicle_params_.alpha_limit,
      ". The accuracy of the physics simulation may be compromised.");
  }

  // 最初は変数の初期化だけして終了
  if (!is_initialized_)
  {
    prev_sim_time_ = info.simTime;
    prev_alpha_ = alpha;
    is_initialized_ = true;
    return;
  }

  // 時刻と迎角の変化率を更新
  const auto dt = chrono::duration<double>(info.dt).count();
  const auto alpha_rate = (alpha - prev_alpha_) / dt;  // [rad/s]
  prev_sim_time_ = info.simTime;
  prev_alpha_ = alpha;

  // 舵角を更新
  updateDeflections(ecm, dt);

  // 無次元空力係数
  const auto force_coefs = nonDimentionalAeroCoefs_Force(alpha, beta);
  const auto moment_coefs = nonDimentionalAeroCoefs_Moment(alpha, beta, alpha_rate, V);

  // 定数部分を計算しておく
  const auto altitude = alt_0_ + P_W_B.Z();
  const auto rho = tobas_std::altitudeToDensity(altitude);
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

  // NED coordinates -> NWU coordinates
  NED2NWU(force_B);
  NED2NWU(torque_B);

  // 世界座標系に変換
  const auto force_W = R_W_B.RotateVector(force_B);
  const auto torque_W = R_W_B.RotateVector(torque_B);

  // 空気力を作用させる
  gz::math::Vector3d B_Pos_BC;
  vectorKDLToGazebo(vehicle_params_.ac, B_Pos_BC);
  link_->AddWorldWrench(ecm, force_W, torque_W, B_Pos_BC);

  // デバッグ用メッセージを発行
  auto debug_msg = make_unique<tobas_gazebo_msgs::msg::FixedWingDebug>();
  ros2::timeChronoToMsg(info.simTime, debug_msg->header.stamp);
  vectorGazeboToMsg(vel_B, debug_msg->relative_body_velocity);
  debug_msg->alpha = alpha;
  debug_msg->beta = beta;
  vectorGazeboToMsg(force_B, debug_msg->air_force);
  vectorGazeboToMsg(torque_B, debug_msg->air_moment);
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    debug_msg->deflections.push_back(cs_angle_models_[i].currentPosition());
  debug_pub_->publish(move(debug_msg));
}

void GazeboFixedWingPlugin::updateDeflections(gz::sim::EntityComponentManager& ecm, double dt)
{
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
  {
    // 角度と角速度の制限を考慮して制御面の舵角を更新
    // Transmissionに任せることもできるが，プラグイン内で完結するようにしている
    // アクティベートされていなければ0を目標値とする
    const auto& cmd_deflection = cs_deflections_ ? cs_deflections_->deflections[i] : 0.;
    cs_angle_models_[i].update(cmd_deflection, dt);

    // Gazebo内の関節角を更新
    // これは単なるアニメーションであり，制御面を動かすことによる機体への反作用は考慮しない
    cs_joints_[i]->ResetPosition(ecm, { cs_angle_models_[i].currentPosition() });
  }
}

gz::math::Vector3d GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Force(double alpha, double beta) const
{
  const auto C_L = liftCoefficient(alpha);  // 揚力係数 (1.8-3)
  const auto C_D = dragCoefficient(alpha);  // 抗力係数 (1.8-3)
  const auto C_S = sideCoefficient(beta);   // 横力係数 (1.8-5)

  const auto cos_alpha = cos(alpha);
  const auto sin_alpha = sin(alpha);

  const auto C_x = -C_D * cos_alpha + C_L * sin_alpha;  // (1.8-4)
  const auto C_z = -C_L * cos_alpha - C_D * sin_alpha;  // (1.8-4)
  const auto C_y = C_S;                                 // (1.8-5)

  return gz::math::Vector3d(C_x, C_y, C_z);
}

gz::math::Vector3d
GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Moment(double alpha, double beta, double alpha_rate, double V) const
{
  // 角速度
  auto gyro_B = gyro_B_->Data();
  NWU2NED(gyro_B);
  const auto p = gyro_B.X();
  const auto q = gyro_B.Y();
  const auto r = gyro_B.Z();

  // (1.8-9): 揚力中心に力をかけるためモーメントの補正項はなし
  const auto C_l = rollCoefficient(beta, p, r, V);
  const auto C_m = pitchCoefficient(alpha, beta, alpha_rate, q, V);
  const auto C_n = yawCoefficient(beta, p, r, V);

  return gz::math::Vector3d(C_l, C_m, C_n);
}

double GazeboFixedWingPlugin::liftCoefficient(double alpha) const
{
  // 迎角
  auto C_L = aero_coefs_.c_lift_0 + aero_coefs_.c_lift_alpha * alpha;

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_L += control_surfaces_[i].c_lift_delta * cs_angle_models_[i].currentPosition();

  return C_L;
}

double GazeboFixedWingPlugin::dragCoefficient(double alpha) const
{
  // 迎角
  auto C_D = aero_coefs_.c_drag_0 + aero_coefs_.c_drag_alpha * alpha;

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_D += control_surfaces_[i].c_drag_abs_delta * fabs(cs_angle_models_[i].currentPosition());

  return C_D;
}

double GazeboFixedWingPlugin::sideCoefficient(double beta) const
{
  // 横滑り角
  auto C_S = aero_coefs_.c_side_beta * beta;

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_S += control_surfaces_[i].c_side_delta * cs_angle_models_[i].currentPosition();

  return C_S;
}

double GazeboFixedWingPlugin::rollCoefficient(double beta, double p, double r, double V) const
{
  // 横滑り角
  auto C_l = aero_coefs_.c_roll_beta * beta;

  // 角速度
  const auto& b = vehicle_params_.wing_span;
  C_l += b / (2 * V) * (aero_coefs_.c_roll_p * p + aero_coefs_.c_roll_r * r);

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_l += control_surfaces_[i].c_roll_delta * cs_angle_models_[i].currentPosition();

  return C_l;
}

double GazeboFixedWingPlugin::pitchCoefficient(double alpha, double beta, double alpha_rate, double q, double V) const
{
  // 迎角，横滑り角
  auto C_m = aero_coefs_.c_pitch_0 + aero_coefs_.c_pitch_alpha * alpha;
  C_m += aero_coefs_.c_pitch_abs_beta * fabs(beta);

  // 角速度
  const auto& c = vehicle_params_.mac;
  C_m += c / (2 * V) * (aero_coefs_.c_pitch_alpha_rate * alpha_rate + aero_coefs_.c_pitch_q * q);

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_m += control_surfaces_[i].c_pitch_delta * cs_angle_models_[i].currentPosition();

  return C_m;
}

double GazeboFixedWingPlugin::yawCoefficient(double beta, double p, double r, double V) const
{
  // 横滑り角
  auto C_n = aero_coefs_.c_yaw_beta * beta;

  // 角速度
  const auto& b = vehicle_params_.wing_span;
  C_n += b / (2 * V) * (aero_coefs_.c_yaw_p * p + aero_coefs_.c_yaw_r * r);

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_n += control_surfaces_[i].c_yaw_delta * cs_angle_models_[i].currentPosition();

  return C_n;
}

void GazeboFixedWingPlugin::deflectionsCb(const tobas_msgs::msg::ControlSurfaceDeflections::ConstSharedPtr& deflections)
{
  // Check array size
  if (deflections->deflections.size() != control_surfaces_.size())
  {
    TOBAS_ERROR(
      "The size of the received deflections array is ", deflections->deflections.size(),
      ", which does not match numberOfControlSurfaces.");
    return;
  }

  // Update reference deflection angles
  cs_deflections_ = deflections;

  // Update last commanded time
  last_cmd_time_ = prev_sim_time_;
}

void GazeboFixedWingPlugin::windSpeedCb(const tobas_msgs::Wind::ConstSharedPtr& wind)
{
  vectorKDLToGazebo(wind->vel, wind_vel_W_);
}

bool GazeboFixedWingPlugin::sortKey(const tobas::ControlSurface& l, const tobas::ControlSurface& r)
{
  return l.channel < r.channel;
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboFixedWingPlugin,
  gz::sim::System,
  gazebo::GazeboFixedWingPlugin::ISystemConfigure,
  gazebo::GazeboFixedWingPlugin::ISystemPreUpdate)
