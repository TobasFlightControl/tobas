#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/unordered_set.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>

#include <tobas_tools/fixed_wing_tools.hpp>
#include <tobas_tools/constants.hpp>

#include "./fixed_wing_plugin.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_eigen.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/time.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboFixedWingPlugin::GazeboFixedWingPlugin() : super()
{
}

void GazeboFixedWingPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  // ボディフレームを取得
  link_ = model->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  // 制御面のジョイントと角度モデル
  for (const auto& cs : control_surfaces_)
  {
    // ジョイントを取得
    const auto joint = model->GetJoint(cs.joint_name);
    if (joint == nullptr)
      gzthrow(
        kPluginName << ": Couldn't find the control surface joint \"" << cs.joint_name << "\".");

    // ジョイントの制限をチェック
    if (joint->LowerLimit(0) >= joint->UpperLimit(0))
      gzthrow(kPluginName << ": The position limit of " << cs.joint_name << " is invalid.");
    if (joint->GetVelocityLimit(0) <= 0.)
      gzthrow(kPluginName << ": The velocity limit of " << cs.joint_name << " must be positive.");
    if (joint->GetEffortLimit(0) <= 0.)
      gzthrow(kPluginName << ": The effort limit of " << cs.joint_name << " must be positive.");

    // ジョイントモデルを追加
    cs_joints_.push_back(joint);
    cs_angle_models_.emplace_back(cs.angle_limit, cs.max_angle_rate);
  }

  cs_deflections_.deflections.resize(control_surfaces_.size());
  debug_msg_.deflections.resize(control_surfaces_.size());

  registerPubSub();

  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboFixedWingPlugin::onUpdate, this, _1));
}

void GazeboFixedWingPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero, NON_NEGATIVE);

  getSdfParam(
    sdf, "checkDelayThreshold", check_delay_threshold_, kDefaultCheckDelayThreshold, false);

  // Vehicle
  getSdfParam(sdf, "wingSurface", vehicle_params_.wing_surface, POSITIVE);
  getSdfParam(sdf, "wingSpan", vehicle_params_.wing_span, POSITIVE);
  getSdfParam(sdf, "meanAerodynamicChord", vehicle_params_.mac, POSITIVE);

  Vector3d ac;
  getSdfParam(sdf, "aerodynamicCenter", ac);
  vectorGazeboToKDL(ac, vehicle_params_.ac);

  getSdfParam(sdf, "lowerStallAngle", vehicle_params_.alpha_limit.lower, kDefaultLowerStallAngle);
  getSdfParam(sdf, "upperStallAngle", vehicle_params_.alpha_limit.upper, kDefaultUpperStallAngle);
  if (!vehicle_params_.alpha_limit.isValid())
  {
    gzthrow(kPluginName << ": Invalid stall angles");
  }

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

  // ControlSurfaces
  if (sdf->HasElement("controlSurface"))
  {
    unordered_set<int> indexes;
    sdf::ElementPtr cs_elem = sdf->GetElement("controlSurface");

    while (cs_elem)
    {
      tobas::ControlSurface cs;

      getSdfParam(cs_elem, "index", cs.index, NON_NEGATIVE);
      if (tobas_std::contains(indexes, cs.index))
        gzthrow(kPluginName << ": The index of each control surface must be unique.");

      getSdfParam(cs_elem, "jointName", cs.joint_name);

      getSdfParam(cs_elem, "minAngle", cs.angle_limit.lower);
      getSdfParam(cs_elem, "maxAngle", cs.angle_limit.upper);
      if (!cs.angle_limit.isValid() || !cs.angle_limit.inRange(0.))
        gzthrow(kPluginName << ": Invalid range of control surface angle");

      getSdfParam(cs_elem, "maxAngleRate", cs.max_angle_rate, POSITIVE);

      getSdfParam(cs_elem, "cLiftDelta", cs.c_lift_delta, 0.);
      getSdfParam(cs_elem, "cDragAbsDelta", cs.c_drag_abs_delta, 0.);
      getSdfParam(cs_elem, "cSideDelta", cs.c_side_delta, 0.);
      getSdfParam(cs_elem, "cRollDelta", cs.c_roll_delta, 0.);
      getSdfParam(cs_elem, "cPitchDelta", cs.c_pitch_delta, 0.);
      getSdfParam(cs_elem, "cYawDelta", cs.c_yaw_delta, 0.);

      indexes.emplace(cs.index);
      control_surfaces_.push_back(cs);
      cs_elem = cs_elem->GetNextElement("controlSurface");
    }

    for (size_t i = 0; i < indexes.size(); ++i)
    {
      if (!tobas_std::contains(indexes, static_cast<int>(i)))
        gzthrow(kPluginName << ": controlSurface index mismatch.");
    }
  }

  // index順に並べ替える
  sort(control_surfaces_.begin(), control_surfaces_.end(), sortKey);
}

void GazeboFixedWingPlugin::registerPubSub()
{
  debug_pub_ =
    nh_.advertise<tobas_gazebo_plugins::FixedWingDebug>("/" + ns_ + "/" + kDebugPubTopic, 1);

  deflections_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + tobas::kDeflectionCmdTopic, 1, &GazeboFixedWingPlugin::deflectionsCb, this,
    ros::TransportHints().reliable().tcpNoDelay());
  wind_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + kWindGtTopic, 1, &GazeboFixedWingPlugin::windSpeedCb, this,
    ros::TransportHints().reliable().tcpNoDelay());
}

void GazeboFixedWingPlugin::onUpdate(const common::UpdateInfo& info)
{
  // 最新のコマンドからの経過時間を確認
  const auto& cur_time = info.simTime;
  const auto time_after_last_cmd = cur_time - last_cmd_time_;
  if (cs_activated_ && time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    tobas_std::fill(cs_deflections_.deflections, 0.);
    cs_activated_ = false;
    gzmsg << kPluginName
          << ": Deflection angles of control surfaces are automatically reset because "
          << tobas::kAutoResetTimeThreshold << " seconds have elapsed since the last command."
          << endl;
  }

  // 風に対する相対的な機体速度
  const auto& W_rot_B = link_->WorldPose().Rot();
  const auto linvel_W = link_->WorldLinearVel() - wind_vel_W_;
  auto linvel_B = W_rot_B.RotateVectorReverse(linvel_W);

  // NWU -> NED
  NWU2NED(linvel_B);

  // 相対風速
  const auto& u = linvel_B.X();
  const auto& v = linvel_B.Y();
  const auto& w = linvel_B.Z();
  const auto V = max(linvel_B.Length(), tobas::kMinAirSpeedThresh);  // V > 0 を保証する

  // 迎角と横滑り角
  const auto alpha = tobas::angleOfAttack(u, w);      // 迎角 [rad]
  const auto beta = tobas::angleOfSideSlip(u, v, w);  // 横滑り角 [rad]

  // 迎角の範囲チェック
  if (!vehicle_params_.alpha_limit.inRange(alpha))
  {
    GZ_ERROR_THROTTLE(
      kErrorPeriod, kPluginName << ": The angle of attack " << alpha
                                << " is not within the valid range " << vehicle_params_.alpha_limit
                                << ". The accuracy of the physics simulation may be compromised.");
  }

  // 最初は変数の初期化だけして終了
  if (!is_initialized_)
  {
    prev_sim_time_ = info.simTime.Double();
    prev_alpha_ = alpha;
    is_initialized_ = true;
    return;
  }

  // 時刻と迎角の変化率を更新
  const auto dt = (cur_time - prev_sim_time_).Double();
  const auto alpha_rate = (alpha - prev_alpha_) / dt;  // [rad/s]
  prev_sim_time_ = cur_time;
  prev_alpha_ = alpha;

  // 舵角を更新
  updateDeflections(dt);

  // 無次元空力係数
  const auto force_coefs = nonDimentionalAeroCoefs_Force(alpha, beta);
  const auto moment_coefs = nonDimentionalAeroCoefs_Moment(alpha, beta, alpha_rate, V);

  // 定数部分を計算しておく
  const auto q_bar = dynamicPressure(V);         // 動圧 (p.15) [Pa]
  const auto& S = vehicle_params_.wing_surface;  // 主翼面積 [m^2]

  // 空気力 (1.8-1)
  auto air_force = q_bar * S * force_coefs;  // [N]

  // 空気モーメント (1.8-7)
  const auto& C_l = moment_coefs.X();                                     // [-]
  const auto& C_m = moment_coefs.Y();                                     // [-]
  const auto& C_n = moment_coefs.Z();                                     // [-]
  const auto& b = vehicle_params_.wing_span;                              // [m]
  const auto& c_bar = vehicle_params_.mac;                                // [m]
  auto air_moment = q_bar * S * Vector3d(b * C_l, c_bar * C_m, b * C_n);  // [Nm]

  // NED -> NWU
  NED2NWU(air_force);
  NED2NWU(air_moment);

  // 空気力を作用させる
  Vector3d ac;
  vectorKDLToGazebo(vehicle_params_.ac, ac);
  link_->AddLinkForce(air_force, ac);
  link_->AddRelativeTorque(air_moment);

  // デバッグ用メッセージを発行
  timeGazeboToRos(info.simTime, debug_msg_.header.stamp);
  vectorGazeboToKDL(linvel_B, debug_msg_.relative_body_velocity);
  debug_msg_.alpha = alpha;
  debug_msg_.beta = beta;
  vectorGazeboToKDL(air_force, debug_msg_.air_force);
  vectorGazeboToKDL(air_moment, debug_msg_.air_moment);
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
  {
    debug_msg_.deflections[i] = cs_angle_models_[i].currentPosition();
  }
  debug_pub_.publish(debug_msg_);
}

void GazeboFixedWingPlugin::updateDeflections(const double& dt)
{
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
  {
    // 角度と角速度の制限を考慮して制御面の舵角を更新
    // Transmissionに任せることもできるが，プラグイン内で完結するようにしている
    const auto& cmd_deflection = cs_deflections_.deflections[i];
    cs_angle_models_[i].update(cmd_deflection, dt);

    // Gazebo内の関節角を更新
    // これは単なるアニメーションであり，制御面を動かすことによる機体への反作用は考慮しない
    if (!cs_joints_[i]->SetPosition(0, cs_angle_models_[i].currentPosition(), true))
      gzerr << kPluginName << ": Failed to set control surface deflection." << endl;
  }
}

Vector3d
GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Force(const double& alpha, const double& beta)
{
  const auto C_L = liftCoefficient(alpha);  // 揚力係数 (1.8-3)
  const auto C_D = dragCoefficient(alpha);  // 抗力係数 (1.8-3)
  const auto C_S = sideCoefficient(beta);   // 横力係数 (1.8-5)

  const auto cos_alpha = cos(alpha);
  const auto sin_alpha = sin(alpha);

  const auto C_x = -C_D * cos_alpha + C_L * sin_alpha;  // (1.8-4)
  const auto C_z = -C_L * cos_alpha - C_D * sin_alpha;  // (1.8-4)
  const auto C_y = C_S;                                 // (1.8-5)

  return Vector3d(C_x, C_y, C_z);
}

Vector3d GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Moment(
  const double& alpha,
  const double& beta,
  const double& alpha_rate,
  const double& V)
{
  // 角速度
  Vector3d B_angular_velocity_W_B = link_->RelativeAngularVel();
  NWU2NED(B_angular_velocity_W_B);
  const auto p = B_angular_velocity_W_B.X();
  const auto q = B_angular_velocity_W_B.Y();
  const auto r = B_angular_velocity_W_B.Z();

  // (1.8-9): 揚力中心に力をかけるためモーメントの補正項はなし
  const auto C_l = rollCoefficient(beta, p, r, V);
  const auto C_m = pitchCoefficient(alpha, beta, alpha_rate, q, V);
  const auto C_n = yawCoefficient(beta, p, r, V);

  return Vector3d(C_l, C_m, C_n);
}

double GazeboFixedWingPlugin::liftCoefficient(const double& alpha)
{
  // 迎角
  auto C_L = aero_coefs_.c_lift_0 + aero_coefs_.c_lift_alpha * alpha;

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_L += control_surfaces_[i].c_lift_delta * cs_angle_models_[i].currentPosition();

  return C_L;
}

double GazeboFixedWingPlugin::dragCoefficient(const double& alpha)
{
  // 迎角
  auto C_D = aero_coefs_.c_drag_0 + aero_coefs_.c_drag_alpha * alpha;

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_D += control_surfaces_[i].c_drag_abs_delta * abs(cs_angle_models_[i].currentPosition());

  return C_D;
}

double GazeboFixedWingPlugin::sideCoefficient(const double& beta)
{
  // 横滑り角
  auto C_S = aero_coefs_.c_side_beta * beta;

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_S += control_surfaces_[i].c_side_delta * cs_angle_models_[i].currentPosition();

  return C_S;
}

double GazeboFixedWingPlugin::rollCoefficient(
  const double& beta,
  const double& p,
  const double& r,
  const double& V)
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

double GazeboFixedWingPlugin::pitchCoefficient(
  const double& alpha,
  const double& beta,
  const double& alpha_rate,
  const double& q,
  const double& V)
{
  // 迎角，横滑り角
  auto C_m = aero_coefs_.c_pitch_0 + aero_coefs_.c_pitch_alpha * alpha;
  C_m += aero_coefs_.c_pitch_abs_beta * abs(beta);

  // 角速度
  const auto& c = vehicle_params_.mac;
  C_m += c / (2 * V) * (aero_coefs_.c_pitch_alpha_rate * alpha_rate + aero_coefs_.c_pitch_q * q);

  // 舵面
  for (size_t i = 0; i < control_surfaces_.size(); ++i)
    C_m += control_surfaces_[i].c_pitch_delta * cs_angle_models_[i].currentPosition();

  return C_m;
}

double GazeboFixedWingPlugin::yawCoefficient(
  const double& beta,
  const double& p,
  const double& r,
  const double& V)
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

double GazeboFixedWingPlugin::dynamicPressure(const double& V)
{
  const auto altitude = alt_0_ + link_->WorldPose().Pos().Z();
  const auto rho = tobas_std::altitudeToDensity(altitude);
  return rho * tobas_std::sqr(V) / 2.;
}

void GazeboFixedWingPlugin::deflectionsCb(const CmdMsg& deflections)
{
  // Check array size
  if (deflections.deflections.size() != control_surfaces_.size())
  {
    gzerr << "The size of the received deflections array is " << deflections.deflections.size()
          << ", which does not match numberOfControlSurfaces." << endl;
    return;
  }

  // Check delay
  const auto delay = (prev_sim_time_ - deflections.header.stamp).toSec();
  if (delay > check_delay_threshold_)
  {
    GZ_WARN_THROTTLE(
      kWarnPeriod, kPluginName << ": The delay from sensors to the motor command " << delay
                               << "[s] is over " << check_delay_threshold_ << "[s].");
  }
  else if (delay < -kNegativeCmdDelayErrThreshold)
  {
    GZ_ERROR_THROTTLE(
      kErrorPeriod, kPluginName << ": Timestamp of the motor command precedes the current time.");
  }

  // Update reference deflection angles
  cs_deflections_ = deflections;

  // Update last commanded time
  last_cmd_time_ = prev_sim_time_;

  // Control surfaces are now activated
  cs_activated_ = true;
}

void GazeboFixedWingPlugin::windSpeedCb(const WindMsg& wind)
{
  vectorKDLToGazebo(wind.vel, wind_vel_W_);
}

bool GazeboFixedWingPlugin::sortKey(const tobas::ControlSurface& l, const tobas::ControlSurface& r)
{
  return l.index < r.index;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFixedWingPlugin);
}  // namespace gazebo
