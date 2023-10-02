#include <tobas_msgs/Wind.h>

#include "./wind_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;

namespace gazebo
{
GazeboWindPlugin::GazeboWindPlugin()
  : super(), prev_sim_time_(0.), gust_u_(0.), gust_v_(0.), gust_w_(0.), rnd_gen_(rnd_dev_())
{
}

void GazeboWindPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  link_ = model->GetLink(link_name_);
  if (link_ == NULL)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  const_wind_W_.X() = mean_speed_ * cos(direction_);
  const_wind_W_.Y() = mean_speed_ * sin(direction_);
  const_wind_W_.Z() = 0.;  // 定常風の垂直成分はゼロ

  noise_ = NormalDistribution(0., 1.);

  registerPubSub();
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::onUpdate, this, _1));
}

void GazeboWindPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "windPubTopic", wind_topic_, kDefaultWindTopic);
  getSdfParam(sdf, "meanWindSpeed", mean_speed_, kDefaultMeanWindSpeed);
  getSdfParam(sdf, "constantWindDirection", direction_, kDefaultConstantWindDirection);
}

void GazeboWindPlugin::onUpdate(const common::UpdateInfo& info)
{
  // 地面からの高度を取得
  const auto h = max(link_->WorldPose().Pos().Z(), kMinimumAltitude);  // [m]
  const auto h_ft = h * kMeterToFeet;
  if (h_ft > kLowAltitudeThreshold)
  {
    gzwarn << kPluginName << ": Since the altitude from the ground exceeds "
           << kLowAltitudeThreshold << " feet, the wind simulation might be inaccurate." << endl;
  }

  // 時刻を更新
  const auto cur_time = info.simTime.Double();
  const auto dt = cur_time - prev_sim_time_;
  prev_sim_time_ = cur_time;

  // リンクに対する定常風の相対速度を計算
  const auto relative_wind_vel_W = const_wind_W_ - link_->WorldLinearVel();  // [m/s]
  const auto V = relative_wind_vel_W.Length();                               // [m/s]

  // スケール長と乱流の速度の標準偏差を計算
  const auto tmp = 0.177 + 0.000823 * h_ft;       // [-]
  const auto L_w = h;                             // [m]
  const auto L_uv = h / pow(tmp, 1.2);            // [m]
  const auto sigma_w = 0.1 * mean_speed_;         // [m/s]
  const auto sigma_uv = sigma_w / pow(tmp, 0.4);  // [m/s]

  // 乱流の成分を更新
  // 突風の波長が一定の場合，相対的な風速が大きいほど周波数が大きくなる (ドップラー効果)
  gust_u_ = (1 - V / L_uv * dt) * gust_u_ + sqrt(2 * V / L_uv * dt) * sigma_uv * noise_(rnd_gen_);
  gust_v_ = (1 - V / L_uv * dt) * gust_v_ + sqrt(2 * V / L_uv * dt) * sigma_uv * noise_(rnd_gen_);
  gust_w_ = (1 - V / L_w * dt) * gust_w_ + sqrt(2 * V / L_w * dt) * sigma_w * noise_(rnd_gen_);

  // 風速メッセージを作成
  auto wind_msg = boost::make_shared<tobas_msgs::Wind>();
  wind_msg->header.frame_id = "world";
  wind_msg->vel.x(const_wind_W_.X() + gust_u_);
  wind_msg->vel.y(const_wind_W_.Y() + gust_v_);
  wind_msg->vel.z(const_wind_W_.Z() + gust_w_);

  // 風速を発行
  wind_pub_.publish(wind_msg);
}

void GazeboWindPlugin::registerPubSub()
{
  wind_pub_ = nh_.advertise<tobas_msgs::Wind>("/" + ns_ + "/" + wind_topic_, 1);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}  // namespace gazebo
