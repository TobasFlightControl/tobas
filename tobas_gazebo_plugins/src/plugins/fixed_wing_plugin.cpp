#include "../../include/plugins/fixed_wing_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/conversions.hpp"
#include "../../include/tobas_gazebo_plugins/constants.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"

#define RAD2DEG (180. / M_PI)

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
  // 機体座標系から見た風速と角速度
  const Quaterniond& W_rot_B = link_->WorldPose().Rot();
  Vector3d W_air_speed_W_B = link_->WorldLinearVel() - wind_speed_W_;
  Vector3d B_air_speed_W_B = W_rot_B.RotateVectorReverse(W_air_speed_W_B);
  Vector3d B_angular_velocity_W_B = link_->RelativeAngularVel();

  // NWU -> NED
  NWU2NED(B_air_speed_W_B);
  NWU2NED(B_angular_velocity_W_B);

  // 迎角と横滑り角
  const double& u = B_air_speed_W_B.X();
  const double& v = B_air_speed_W_B.Y();
  const double& w = B_air_speed_W_B.Z();
  double V = B_air_speed_W_B.Length();
  double alpha = (u < kMinAirSpeedThresh) ? 0. : atan(w / u);
  double beta = (V < kMinAirSpeedThresh) ? 0. : asin(v / V);

  // 迎角の範囲チェック
  if (!alpha_range_.inRange(alpha))
  {
    gzwarn << kPluginName << ": The angle of attack " << alpha << " is not within the valid range ["
           << alpha_range_.lower << ", " << alpha_range_.upper
           << "]. The accuracy of the physics simulation may be compromised." << endl;
    alpha = alpha_range_.clamp(alpha);
  }

  // 無次元空力係数
  Vector3d force_coefs = nonDimentionalAeroCoefs_Force(alpha, beta);                 // Cx, Cy, Cz
  Vector3d moment_coefs = nonDimentionalAeroCoefs_Moment(alpha, beta, force_coefs);  // Cl, Cm, Cn

  // 定数部分を計算しておく
  double q_bar = dynamicPressure();                // 動圧
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
  const Vector3d& force_coefs)
{
  double C_l = rollCoefficient(beta);
  double C_m = pitchCoefficient(alpha, beta, force_coefs.Z());
  double C_n = yawCoefficient(beta, force_coefs.Y());

  return Vector3d(C_l, C_m, C_n);
}

double GazeboFixedWingPlugin::liftCoefficient(double alpha)
{
  // TODO
}

double GazeboFixedWingPlugin::dragCoefficient(double alpha)
{
  // TODO
}

double GazeboFixedWingPlugin::sideCoefficient(double beta)
{
  // TODO
}

double GazeboFixedWingPlugin::rollCoefficient(double beta)
{
  // TODO
}

double GazeboFixedWingPlugin::pitchCoefficient(double alpha, double beta, double C_z)
{
  // TODO
}

double GazeboFixedWingPlugin::yawCoefficient(double beta, double C_y)
{
  // TODO
}

double GazeboFixedWingPlugin::dynamicPressure()
{
  // TODO
}

void GazeboFixedWingPlugin::deflectionsCb(const CmdMsg& cmd)
{
  // TODO
}

void GazeboFixedWingPlugin::windSpeedCb(const WindMsg& wind)
{
  vectorRosToGazebo(wind.velocity, wind_speed_W_);
}
}  // namespace gazebo
