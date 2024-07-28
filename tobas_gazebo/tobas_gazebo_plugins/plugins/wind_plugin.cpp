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

  // Initialize wind parameters
  params_.mean_speed = kDefaultMeanWindSpeed;
  params_.direction = kDefaultConstantWindDirection;
  params_.gust_speed_factor = kDefaultGustSpeedFactor;
  params_.gust_duration = kDefaultGustDuration;
  params_.gust_interval = kDefaultGustInterval;

  getSdfParams(sdf);

  link_ = model->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  wind_pub_ = nh_.advertise<tobas_msgs::Wind>("/" + ns_ + "/" + kWindGtTopic, 1);
  get_wind_params_ss_ = nh_.advertiseService("/" + ns_ + "/" + kGetWindParamsSrv, &self::getWindParamsCb, this);
  set_wind_params_ss_ = nh_.advertiseService("/" + ns_ + "/" + kSetWindParamsSrv, &self::setWindParamsCb, this);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboWindPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
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
      if (gust_time > params_.gust_duration)
      {
        gust_state_ = NO_GUST;
        gust_state_change_time_ = cur_time;
        break;
      }

      const auto max_gust_speed = params_.mean_speed * params_.gust_speed_factor;
      gust_speed_ = 0.5 * max_gust_speed * (1 - cos(2 * M_PI * gust_time / params_.gust_duration));
      break;
    }
    case NO_GUST:
    {
      if (gust_time > params_.gust_interval)
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
  const auto v_steady_wind = params_.mean_speed + gust_speed_;
  const Vector3d steady_W(v_steady_wind * cos(params_.direction), v_steady_wind * sin(params_.direction), 0.);

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

bool GazeboWindPlugin::getWindParamsCb(
  tobas_gazebo_msgs::GetWindParamsRequest& req,
  tobas_gazebo_msgs::GetWindParamsResponse& res)
{
  res.params = params_;
  return true;
}

bool GazeboWindPlugin::setWindParamsCb(
  tobas_gazebo_msgs::SetWindParamsRequest& req,
  tobas_gazebo_msgs::SetWindParamsResponse& res)
{
  res.success = false;
  res.params.mean_speed = params_.mean_speed;
  res.params.direction = params_.direction;
  res.params.gust_speed_factor = params_.gust_speed_factor;
  res.params.gust_duration = params_.gust_duration;
  res.params.gust_interval = params_.gust_interval;

  // Mean speed
  if (req.params.mean_speed < 0)
  {
    gzerr << kPluginName << ": Mean wind speed must be non-negative." << endl;
    return true;
  }
  params_.mean_speed = res.params.mean_speed = req.params.mean_speed;

  // Direction
  params_.direction = res.params.direction = req.params.direction;

  // Gust speed factor
  if (req.params.gust_speed_factor > 0)
    params_.gust_speed_factor = res.params.gust_speed_factor = req.params.gust_speed_factor;
  else
    gzwarn << kPluginName << ": Gust speed factor remains unchanged." << endl;

  // Gust duration
  if (req.params.gust_duration > 0)
    params_.gust_duration = res.params.gust_duration = req.params.gust_duration;
  else
    gzwarn << kPluginName << ": Gust duration remains unchanged." << endl;

  // Gust interval
  if (req.params.gust_interval > 0)
    params_.gust_interval = res.params.gust_interval = req.params.gust_interval;
  else
    gzwarn << kPluginName << ": Gust interval remains unchanged." << endl;

  // Update dryden wind model
  dryden_.setMeanWindSpeed(req.params.mean_speed);

  res.success = true;
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}  // namespace gazebo
