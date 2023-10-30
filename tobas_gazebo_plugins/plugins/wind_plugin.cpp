#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Wind.h>

#include "./wind_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboWindPlugin::GazeboWindPlugin() : super()
{
}

void GazeboWindPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  link_ = model->GetLink(link_name_);
  if (link_ == nullptr)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  dryden_.setMeanWindSpeed(mean_speed_);

  registerPubSub();
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::onUpdate, this, _1));
}

void GazeboWindPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "meanWindSpeed", mean_speed_, kDefaultMeanWindSpeed, NON_NEGATIVE);
  getSdfParam(sdf, "constantWindDirection", direction_, kDefaultConstantWindDirection);
  getSdfParam(sdf, "gustSpeedFactor", gust_speed_factor_, kDefaultGustSpeedFactor, NON_NEGATIVE);
  getSdfParam(sdf, "gustDuration", gust_duration_, kDefaultGustDuration, POSITIVE);
  getSdfParam(sdf, "gustInterval", gust_interval_, kDefaultGustInterval, NON_NEGATIVE);
}

void GazeboWindPlugin::onUpdate(const common::UpdateInfo& info)
{
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

  // 乱流成分を更新
  const auto rel_wind_speed = (steady_W - link_->WorldLinearVel()).Length();  // 定常風の相対速度
  dryden_.update(rel_wind_speed, link_->WorldPose().Pos().Z(), dt);
  const Vector3d turb_B(dryden_.u(), dryden_.v(), dryden_.w());

  // 全体の風速を計算
  const auto wind_W = steady_W + link_->WorldPose().Rot() * turb_B;

  // 風速メッセージを作成
  const auto wind_msg = boost::make_shared<tobas_msgs::Wind>();
  wind_msg->header.frame_id = "world";
  vectorGazeboToKDL(wind_W, wind_msg->vel);

  // 風速を発行
  wind_pub_.publish(wind_msg);
}

void GazeboWindPlugin::registerPubSub()
{
  wind_pub_ = nh_.advertise<tobas_msgs::Wind>("/" + ns_ + "/" + tobas::kWindGtTopic, 1);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}  // namespace gazebo
