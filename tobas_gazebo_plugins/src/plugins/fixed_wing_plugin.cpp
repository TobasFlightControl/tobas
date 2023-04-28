#include <dh_std_tools/math.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>

#include "../../include/plugins/fixed_wing_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/conversions.hpp"
#include "../../include/tobas_gazebo_plugins/constants.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)

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

  cs_deflections_.deflections.resize(num_control_surfaces_);

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
  // TODO

  // TODO: control surfaceのindexが0,1,...,N-1になっていることを確認
  // AeroCoefの正負をチェック
}

void GazeboFixedWingPlugin::registerPubSub()
{
  deflections_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + deflections_sub_topic_, 1, &GazeboFixedWingPlugin::deflectionsCb, this);
  wind_speed_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + wind_speed_sub_topic_, 1, &GazeboFixedWingPlugin::windSpeedCb, this);
}

void GazeboFixedWingPlugin::onUpdate(const common::UpdateInfo& info)
{
  // 機体に対する相対的な風速
  const Quaterniond& W_rot_B = link_->WorldPose().Rot();
  Vector3d W_air_speed_W_B = link_->WorldLinearVel() - wind_speed_W_;
  Vector3d B_air_speed_W_B = W_rot_B.RotateVectorReverse(W_air_speed_W_B);

  // NWU -> NED
  NWU2NED(B_air_speed_W_B);

  // 風速
  const double& u = B_air_speed_W_B.X();
  const double& v = B_air_speed_W_B.Y();
  const double& w = B_air_speed_W_B.Z();
  double V = B_air_speed_W_B.Length();
  if (min(u, V) < kMinAirSpeedThresh)  // 風速が閾値より小さければ空気力の計算は行わない
  {
    return;
  }

  // 迎角と横滑り角
  double alpha = atan(w / u);
  double beta = asin(v / V);

  // 迎角の範囲チェック
  if (!alpha_range_.inRange(alpha))
  {
    gzwarn << kPluginName << ": The angle of attack " << alpha << " is not within the valid range ["
           << alpha_range_.lower << ", " << alpha_range_.upper
           << "]. The accuracy of the physics simulation may be compromised." << endl;
    alpha = alpha_range_.clamp(alpha);
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
  double cur_time = info.simTime.Double();
  double dt = cur_time - prev_sim_time_;
  double alpha_rate = (alpha - prev_alpha_) / dt;
  prev_sim_time_ = cur_time;
  prev_alpha_ = alpha;

  // 無次元空力係数
  Vector3d force_coefs = nonDimentionalAeroCoefs_Force(alpha, beta);          // Cx, Cy, Cz
  Vector3d moment_coefs =
    nonDimentionalAeroCoefs_Moment(alpha, beta, alpha_rate, V, force_coefs);  // Cl, Cm, Cn

  // 定数部分を計算しておく
  double q_bar = dynamicPressure(V);               // 動圧 (p.15)
  const double& S = vehicle_params_.wing_surface;  // 主翼面積

  // 空気力 (1.8-1)
  Vector3d air_force = q_bar * S * force_coefs;

  // 空気モーメント (1.8-7)
  const double& C_l = moment_coefs.X();
  const double& C_m = moment_coefs.Y();
  const double& C_n = moment_coefs.Z();
  const double& b = vehicle_params_.wing_span;
  const double& c_bar = vehicle_params_.mac;
  Vector3d air_moment = RAD2DEG * q_bar * S * Vector3d(b * C_l, c_bar * C_m, b * C_n);

  // NED -> NWU
  NED2NWU(air_force);
  NED2NWU(air_moment);

  // 空気力を作用させる
  link_->AddRelativeForce(air_force);
  link_->AddRelativeTorque(air_moment);
}

Vector3d GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Force(double alpha, double beta)
{
  double C_L = liftCoefficient(alpha);  // 揚力係数 (1.8-3)
  double C_D = dragCoefficient(alpha);  // 抗力係数 (1.8-3)
  double C_S = sideCoefficient(beta);   // 横力係数 (1.8-5)

  double cos_alpha = cos(alpha);
  double sin_alpha = sin(alpha);

  double C_x = -C_D * cos_alpha + C_L * sin_alpha;  // (1.8-4)
  double C_z = -C_L * cos_alpha - C_D * sin_alpha;  // (1.8-4)
  double C_y = C_S;                                 // (1.8-5)

  return Vector3d(C_x, C_y, C_z);
}

Vector3d GazeboFixedWingPlugin::nonDimentionalAeroCoefs_Moment(
  double alpha,
  double beta,
  double alpha_rate,
  double V,
  const Vector3d& force_coefs)
{
  // 角速度
  Vector3d B_angular_velocity_W_B = link_->RelativeAngularVel();
  NWU2NED(B_angular_velocity_W_B);
  double p = B_angular_velocity_W_B.X();
  double q = B_angular_velocity_W_B.Y();
  double r = B_angular_velocity_W_B.Z();

  // (1.8-9)
  double C_l = rollCoefficient(beta, p, r, V);
  double C_m = pitchCoefficient(alpha, beta, alpha_rate, q, V, force_coefs.Z());
  double C_n = yawCoefficient(beta, p, r, V, force_coefs.Y());

  return Vector3d(C_l, C_m, C_n);
}

double GazeboFixedWingPlugin::liftCoefficient(double alpha)
{
  // 迎角
  double C_L = aero_coefs_.c_lift_0 + aero_coefs_.c_lift_alpha * alpha;

  // 舵面
  for (const auto& cs : aero_coefs_.control_surfaces)
  {
    const double& deflection = cs_deflections_.deflections[cs.index];
    C_L += cs.c_lift_delta * deflection;
  }

  return C_L;
}

double GazeboFixedWingPlugin::dragCoefficient(double alpha)
{
  // 迎角
  double C_D = aero_coefs_.c_drag_0 + aero_coefs_.c_drag_alpha * alpha;

  // 舵面
  for (const auto& cs : aero_coefs_.control_surfaces)
  {
    const double& deflection = cs_deflections_.deflections[cs.index];
    C_D += cs.c_drag_abs_delta * abs(deflection);  // 舵角の正負にかかわらず抗力が発生するモデル
  }

  return C_D;
}

double GazeboFixedWingPlugin::sideCoefficient(double beta)
{
  // 横滑り角
  double C_S = aero_coefs_.c_side_beta * beta;

  // 舵面
  for (const auto& cs : aero_coefs_.control_surfaces)
  {
    const double& deflection = cs_deflections_.deflections[cs.index];
    C_S += cs.c_side_delta * deflection;
  }

  return C_S;
}

double GazeboFixedWingPlugin::rollCoefficient(double beta, double p, double r, double V)
{
  // 横滑り角
  double C_l = aero_coefs_.c_roll_beta * beta;

  // 角速度
  const double& b = vehicle_params_.wing_span;
  C_l += b / (2 * V) * DEG2RAD * (aero_coefs_.c_roll_p * p + aero_coefs_.c_roll_r * r);

  // 舵面
  for (const auto& cs : aero_coefs_.control_surfaces)
  {
    const double& deflection = cs_deflections_.deflections[cs.index];
    C_l += cs.c_roll_delta * deflection;
  }

  return C_l;
}

double GazeboFixedWingPlugin::pitchCoefficient(
  double alpha,
  double beta,
  double alpha_rate,
  double q,
  double V,
  double C_z)
{
  // 迎角，横滑り角
  double C_m = aero_coefs_.c_pitch_0 + aero_coefs_.c_pitch_alpha * alpha;
  C_m += aero_coefs_.c_pitch_abs_beta * abs(beta);

  // 角速度
  C_m += vehicle_params_.mac / (2 * V) * DEG2RAD
         * (aero_coefs_.c_pitch_alpha_rate * alpha_rate + aero_coefs_.c_pitch_q * q);

  // 舵面
  for (const auto& cs : aero_coefs_.control_surfaces)
  {
    const double& deflection = cs_deflections_.deflections[cs.index];
    C_m += cs.c_pitch_delta * deflection;
  }

  return C_m;
}

double GazeboFixedWingPlugin::yawCoefficient(double beta, double p, double r, double V, double C_y)
{
  // 横滑り角
  double C_n = aero_coefs_.c_yaw_beta * beta;

  // 角速度
  const double& b = vehicle_params_.wing_span;
  C_n += b / (2 * V) * DEG2RAD * (aero_coefs_.c_yaw_p * p + aero_coefs_.c_yaw_r * r);

  // 舵面
  for (const auto& cs : aero_coefs_.control_surfaces)
  {
    const double& deflection = cs_deflections_.deflections[cs.index];
    C_n += cs.c_yaw_delta * deflection;
  }

  return C_n;
}

double GazeboFixedWingPlugin::dynamicPressure(double V)
{
  double altitude = ref_alt_ + link_->WorldPose().Pos().Z();
  double rho = dh_std::altitudeToDensity(altitude);
  return rho * dh_std::sqr(V) / 2;
}

void GazeboFixedWingPlugin::deflectionsCb(const CmdMsg& deflections)
{
  if (deflections.deflections.size() != num_control_surfaces_)
  {
    gzerr << "The size of the received deflections array is " << deflections.deflections.size()
          << ", which does not match numberOfControlSurfaces." << endl;
    return;
  }

  cs_deflections_ = deflections;
}

void GazeboFixedWingPlugin::windSpeedCb(const WindMsg& wind)
{
  vectorRosToGazebo(wind.velocity, wind_speed_W_);
}
}  // namespace gazebo
