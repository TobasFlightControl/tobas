#include <tobas_msgs/Wind.h>

#include "./wind_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboWindPlugin::GazeboWindPlugin() : super(), rnd_gen_(rnd_dev_())
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
  getSdfParam(sdf, "meanWindSpeed", mean_speed_, kDefaultMeanWindSpeed, NON_NEGATIVE);
  getSdfParam(sdf, "constantWindDirection", direction_, kDefaultConstantWindDirection);
  getSdfParam(sdf, "gustSpeedFactor", gust_speed_factor_, kDefaultGustSpeedFactor, NON_NEGATIVE);
  getSdfParam(sdf, "gustDuration", gust_duration_, kDefaultGustDuration, POSITIVE);
  getSdfParam(sdf, "gustInterval", gust_interval_, kDefaultGustInterval, NON_NEGATIVE);
}

void GazeboWindPlugin::onUpdate(const common::UpdateInfo& info)
{
  // 機体フレームの状態を取得
  const auto& T_W_B = link_->WorldPose();
  const auto& P_W_B = T_W_B.Pos();
  const auto& R_W_B = T_W_B.Rot();

  // 地面からの高度を取得
  const auto h = max(P_W_B.Z(), kMinimumAltitude);  // [m]
  const auto h_ft = h * kMeterToFeet;
  if (h_ft > kLowAltitudeThreshold)
  {
    gzwarn << kPluginName << ": Since the altitude from the ground exceeds "
           << kLowAltitudeThreshold << " feet, the wind simulation might be inaccurate." << endl;
  }

  // 時刻を更新
  const auto& cur_time = info.simTime;
  const auto dt = (cur_time - prev_sim_time_).Double();
  prev_sim_time_ = cur_time;

  // 突風
  const auto gust_time = (cur_time - gust_state_change_time_).Double();
  switch (gust_state_)
  {
    case GUST:
    {
      if (gust_time > gust_duration_)
      {
        gust_state_ = NO_GUST;
        gust_state_change_time_ = cur_time;
        break;
      }

      const auto max_gust_speed = mean_speed_ * gust_speed_factor_;
      gust_speed_ = 0.5 * max_gust_speed * (1 - cos(2 * M_PI * gust_time / gust_duration_));
      break;
    }
    case NO_GUST:
    {
      if (gust_time > gust_interval_)
      {
        gust_state_ = GUST;
        gust_state_change_time_ = cur_time;
        break;
      }

      gust_speed_ = 0.;
      break;
    }
    default:
    {
      gzthrow("Invalid gust state: " << static_cast<int>(gust_state_));
    }
  }

  // 定常風 (平均風速 + 突風)
  const auto v_steady_wind = mean_speed_ + gust_speed_;
  const Vector3d steady_W(v_steady_wind * cos(direction_), v_steady_wind * sin(direction_), 0.);

  // リンクに対する定常風の相対速度を計算
  const auto relative_wind_vel_W = steady_W - link_->WorldLinearVel();  // [m/s]
  const auto V = relative_wind_vel_W.Length();                          // [m/s]

  // スケール長と乱流の速度の標準偏差を計算
  const auto tmp = 0.177 + 0.000823 * h_ft;       // [-]
  const auto L_w = h;                             // [m]
  const auto L_uv = h / pow(tmp, 1.2);            // [m]
  const auto sigma_w = 0.1 * mean_speed_;         // [m/s]
  const auto sigma_uv = sigma_w / pow(tmp, 0.4);  // [m/s]
  const auto r_uv = V / L_uv * dt;                // [-]
  const auto r_w = V / L_w * dt;                  // [-]

  // 乱流を更新
  // 突風の波長が一定の場合，相対的な風速が大きいほど周波数が大きくなる (ドップラー効果)
  turb_B_.X() = (1 - r_uv) * turb_B_.X() + sqrt(2 * r_uv) * sigma_uv * noise_(rnd_gen_);
  turb_B_.Y() = (1 - r_uv) * turb_B_.Y() + sqrt(2 * r_uv) * sigma_uv * noise_(rnd_gen_);
  turb_B_.Z() = (1 - r_w) * turb_B_.Z() + sqrt(2 * r_w) * sigma_w * noise_(rnd_gen_);

  // 全体の風速を計算
  const auto wind_W = steady_W + R_W_B * turb_B_;

  // 風速メッセージを作成
  auto wind_msg = boost::make_shared<tobas_msgs::Wind>();
  wind_msg->header.frame_id = "world";
  vectorGazeboToKDL(wind_W, wind_msg->vel);

  // 風速を発行
  wind_pub_.publish(wind_msg);
}

void GazeboWindPlugin::registerPubSub()
{
  wind_pub_ = nh_.advertise<tobas_msgs::Wind>("/" + ns_ + "/" + wind_topic_, 1);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}  // namespace gazebo
