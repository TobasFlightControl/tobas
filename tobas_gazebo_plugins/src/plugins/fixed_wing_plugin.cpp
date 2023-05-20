#include <dh_std_tools/math.hpp>
#include <dh_std_tools/unordered_set.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>

#include <tobas_tools/fixed_wing_tools.hpp>
#include <tobas_tools/constants.hpp>

#include "../../include/plugins/fixed_wing_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_eigen.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"
#include "../../include/tobas_gazebo_plugins/constants.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"
#include "../../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboFixedWingPlugin::GazeboFixedWingPlugin() : super()
{
}

void GazeboFixedWingPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  // 制御面の角度モデル
  for (const auto& cs : control_surfaces_)
  {
    cs_angle_models_.emplace_back(cs.angle_limit, cs.max_angle_rate);
  }

  cs_deflections_.deflections.resize(control_surfaces_.size());
  debug_msg_.deflections.resize(control_surfaces_.size());

  link_ = model->GetLink(link_name_);
  if (link_ == NULL)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  registerPubSub();

  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboFixedWingPlugin::onUpdate, this, _1));
}

void GazeboFixedWingPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "debugPubTopic", debug_pub_topic_, kDefaultDebugPubTopic);
  getSdfParam(sdf, "deflectionsSubTopic", deflections_sub_topic_, kDefaultDeflectionsSubTopic);
  getSdfParam(sdf, "windSpeedSubTopic", wind_speed_sub_topic_, kDefaultWindSubTopic);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero, NON_NEGATIVE);
  getSdfParam(
    sdf, "checkDelayThreshold", check_delay_threshold_, kDefaultCheckDelayThreshold, POSITIVE);

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
      if (dh_std::contains(indexes, cs.index))
      {
        gzthrow(kPluginName << ": The index of each control surface must be unique.");
      }

      getSdfParam(cs_elem, "minAngle", cs.angle_limit.lower);
      getSdfParam(cs_elem, "maxAngle", cs.angle_limit.upper);
      if (!cs.angle_limit.isValid() || !cs.angle_limit.inRange(0.))
      {
        gzthrow(kPluginName << ": Invalid range of control surface angle");
      }

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

    for (int i = 0; i < indexes.size(); ++i)
    {
      if (!dh_std::contains(indexes, i))
      {
        gzthrow(kPluginName << ": controlSurface index mismatch.");
      }
    }
  }

  // index順に並べ替える
  sort(control_surfaces_.begin(), control_surfaces_.end(), sortKey);
}

void GazeboFixedWingPlugin::registerPubSub()
{
  debug_pub_ = nh_.advertise<tobas_msgs::FixedWingDebug>("/" + ns_ + "/" + debug_pub_topic_, 1);

  deflections_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + deflections_sub_topic_, 1, &GazeboFixedWingPlugin::deflectionsCb, this);
  wind_speed_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + wind_speed_sub_topic_, 1, &GazeboFixedWingPlugin::windSpeedCb, this);
}

void GazeboFixedWingPlugin::onUpdate(const common::UpdateInfo& info)
{
  // 風に対する相対的な機体速度
  const auto& W_rot_B = link_->WorldPose().Rot();
  const auto linvel_W = link_->WorldLinearVel() - wind_speed_W_;
  auto linvel_B = W_rot_B.RotateVectorReverse(linvel_W);

  // NWU -> NED
  NWU2NED(linvel_B);

  // 相対風速
  const auto& u = linvel_B.X();
  const auto& v = linvel_B.Y();
  const auto& w = linvel_B.Z();
  const auto V = max(linvel_B.Length(), tobas::kMinAirSpeedThresh);  // V > 0 を保証する

  // 迎角と横滑り角
  const auto alpha = tobas::angleOfAttack(u, v, w);   // 迎角 [rad]
  const auto beta = tobas::angleOfSideSlip(u, v, w);  // 横滑り角 [rad]

  // 迎角の範囲チェック
  if (!vehicle_params_.alpha_limit.inRange(alpha))
  {
    gzwarn << kPluginName << ": The angle of attack " << alpha << " is not within the valid range "
           << vehicle_params_.alpha_limit
           << ". The accuracy of the physics simulation may be compromised." << endl;
  }

  // 最初は変数の初期化だけして終了
  if (!is_initialized_)
  {
    prev_sim_time_ = info.simTime.Double();
    prev_alpha_ = alpha;
    is_initialized_ = true;
    return;
  }

  // 迎角の変化率
  const auto cur_time = info.simTime.Double();
  const auto dt = cur_time - prev_sim_time_;
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
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    debug_msg_.deflections[i] = cs_angle_models_[i].currentPosition();
  }
  debug_pub_.publish(debug_msg_);
}

void GazeboFixedWingPlugin::updateDeflections(double dt)
{
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    const auto& cmd_deflection = cs_deflections_.deflections[i];
    cs_angle_models_[i].update(cmd_deflection, dt);
  }
}

Vector3d GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Force(double alpha, double beta)
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
  double alpha,
  double beta,
  double alpha_rate,
  double V)
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

double GazeboFixedWingPlugin::liftCoefficient(double alpha)
{
  // 迎角
  auto C_L = aero_coefs_.c_lift_0 + aero_coefs_.c_lift_alpha * alpha;

  // 舵面
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    C_L += control_surfaces_[i].c_lift_delta * cs_angle_models_[i].currentPosition();
  }

  return C_L;
}

double GazeboFixedWingPlugin::dragCoefficient(double alpha)
{
  // 迎角
  auto C_D = aero_coefs_.c_drag_0 + aero_coefs_.c_drag_alpha * alpha;

  // 舵面
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    // 舵角の正負にかかわらず抗力が発生するモデル
    C_D += control_surfaces_[i].c_drag_abs_delta * cs_angle_models_[i].currentPosition();
  }

  return C_D;
}

double GazeboFixedWingPlugin::sideCoefficient(double beta)
{
  // 横滑り角
  auto C_S = aero_coefs_.c_side_beta * beta;

  // 舵面
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    C_S += control_surfaces_[i].c_side_delta * cs_angle_models_[i].currentPosition();
  }

  return C_S;
}

double GazeboFixedWingPlugin::rollCoefficient(double beta, double p, double r, double V)
{
  // 横滑り角
  auto C_l = aero_coefs_.c_roll_beta * beta;

  // 角速度
  const auto& b = vehicle_params_.wing_span;
  C_l += b / (2 * V) * (aero_coefs_.c_roll_p * p + aero_coefs_.c_roll_r * r);

  // 舵面
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    C_l += control_surfaces_[i].c_roll_delta * cs_angle_models_[i].currentPosition();
  }

  return C_l;
}

double GazeboFixedWingPlugin::pitchCoefficient(
  double alpha,
  double beta,
  double alpha_rate,
  double q,
  double V)
{
  // 迎角，横滑り角
  auto C_m = aero_coefs_.c_pitch_0 + aero_coefs_.c_pitch_alpha * alpha;
  C_m += aero_coefs_.c_pitch_abs_beta * abs(beta);

  // 角速度
  const auto& c = vehicle_params_.mac;
  C_m += c / (2 * V) * (aero_coefs_.c_pitch_alpha_rate * alpha_rate + aero_coefs_.c_pitch_q * q);

  // 舵面
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    C_m += control_surfaces_[i].c_pitch_delta * cs_angle_models_[i].currentPosition();
  }

  return C_m;
}

double GazeboFixedWingPlugin::yawCoefficient(double beta, double p, double r, double V)
{
  // 横滑り角
  auto C_n = aero_coefs_.c_yaw_beta * beta;

  // 角速度
  const auto& b = vehicle_params_.wing_span;
  C_n += b / (2 * V) * (aero_coefs_.c_yaw_p * p + aero_coefs_.c_yaw_r * r);

  // 舵面
  for (int i = 0; i < control_surfaces_.size(); ++i)
  {
    C_n += control_surfaces_[i].c_yaw_delta * cs_angle_models_[i].currentPosition();
  }

  return C_n;
}

double GazeboFixedWingPlugin::dynamicPressure(double V)
{
  const auto altitude = alt_0_ + link_->WorldPose().Pos().Z();
  const auto rho = dh_std::altitudeToDensity(altitude);
  return rho * dh_std::sqr(V) / 2.;
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
  const auto delay = prev_sim_time_ - deflections.header.stamp.toSec();
  if (delay > check_delay_threshold_)
  {
    gzwarn << kPluginName << ": The delay from sensors to the motor command " << delay
           << "[s] is over " << check_delay_threshold_ << "[s]." << endl;
  }
  else if (delay < 0.)
  {
    gzerr << kPluginName << ": The timestamp of the motor command precedes the current time."
          << endl;
  }

  cs_deflections_ = deflections;
}

void GazeboFixedWingPlugin::windSpeedCb(const WindMsg& wind)
{
  vectorKDLToGazebo(wind.vel, wind_speed_W_);
}

bool GazeboFixedWingPlugin::sortKey(const tobas::ControlSurface& l, const tobas::ControlSurface& r)
{
  return l.index < r.index;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFixedWingPlugin);
}  // namespace gazebo
