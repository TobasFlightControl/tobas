#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>

#include <tobas_tools/constants.hpp>

#include "./gps_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboGpsPlugin::GazeboGpsPlugin() : super(), is_history_filled_(false)
{
}

void GazeboGpsPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);
  fillMessageStaticParts();
  setRandomDistribuitons();

  world_ = physics::get_world(sensor->WorldName());
  link_ = dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name_));
  if (link_ == nullptr)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  registerPublishers();
  update_connection_ = sensor->ConnectUpdated(boost::bind(&self::onUpdate, this));

  t_last_ = world_->SimTime();
}

void GazeboGpsPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);

  getSdfParam(sdf, "offset", offset_, zero3);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, POSITIVE);
  getSdfParam(sdf, "delay", delay_, kDefaultDelay, NON_NEGATIVE);

  getSdfParam(sdf, "horPosStdDev", hor_pos_std_dev_, kDefaultHorPosStdDev, NON_NEGATIVE);
  getSdfParam(sdf, "verPosStdDev", ver_pos_std_dev_, kDefaultVerPosStdDev, NON_NEGATIVE);
  getSdfParam(sdf, "horVelStdDev", hor_vel_std_dev_, kDefaultHorVelStdDev, NON_NEGATIVE);
  getSdfParam(sdf, "verVelStdDev", ver_vel_std_dev_, kDefaultVerVelStdDev, NON_NEGATIVE);

  getSdfParam(sdf, "latitudeZero", lat_0_, kDefaultLatitudeZero);
  getSdfParam(sdf, "longitudeZero", lon_0_, kDefaultLongitudeZero);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero);
}

void GazeboGpsPlugin::fillMessageStaticParts()
{
  // Fill the static parts of the GPS message
  gps_msg_.header.frame_id = link_name_;

  gps_msg_.position_covariance[0] = dh_std::sqr(hor_pos_std_dev_);
  gps_msg_.position_covariance[1] = 0.;
  gps_msg_.position_covariance[2] = 0.;
  gps_msg_.position_covariance[3] = 0.;
  gps_msg_.position_covariance[4] = dh_std::sqr(hor_pos_std_dev_);
  gps_msg_.position_covariance[5] = 0.;
  gps_msg_.position_covariance[6] = 0.;
  gps_msg_.position_covariance[7] = 0.;
  gps_msg_.position_covariance[8] = dh_std::sqr(ver_pos_std_dev_);

  gps_msg_.velocity_covariance[0] = dh_std::sqr(hor_vel_std_dev_);
  gps_msg_.velocity_covariance[1] = 0.;
  gps_msg_.velocity_covariance[2] = 0.;
  gps_msg_.velocity_covariance[3] = 0.;
  gps_msg_.velocity_covariance[4] = dh_std::sqr(hor_vel_std_dev_);
  gps_msg_.velocity_covariance[5] = 0.;
  gps_msg_.velocity_covariance[6] = 0.;
  gps_msg_.velocity_covariance[7] = 0.;
  gps_msg_.velocity_covariance[8] = dh_std::sqr(ver_vel_std_dev_);
}

void GazeboGpsPlugin::setRandomDistribuitons()
{
  const Vector3d pos_stddev(hor_pos_std_dev_, hor_pos_std_dev_, ver_pos_std_dev_);
  const Vector3d vel_stddev(hor_vel_std_dev_, hor_vel_std_dev_, ver_vel_std_dev_);
  pos_noise_.reset(new NormalDistribution3d(rnd_dev_, zero3, pos_stddev));
  vel_noise_.reset(new NormalDistribution3d(rnd_dev_, zero3, vel_stddev));
}

void GazeboGpsPlugin::registerPublishers()
{
  gps_pub_ = nh_.advertise<tobas_msgs::Gps>("/" + ns_ + "/" + tobas::kGpsTopic, 1);
}

void GazeboGpsPlugin::onUpdate()
{
  // 現在の状態を履歴に追加
  const common::Time cur_time = world_->SimTime();
  history_.emplace_back(
    cur_time, link_->WorldPose(), link_->WorldLinearVel(), link_->RelativeAngularVel());

  // 古い履歴を削除
  while ((cur_time - get<0>(history_.front())).Double() > delay_)
  {
    history_.pop_front();
    if (!is_history_filled_)
    {
      is_history_filled_ = true;
    }
  }

  // 履歴が溜まっていなければ発行しない
  if (!is_history_filled_)
  {
    return;
  }

  // 更新時刻になっていなければ発行しない
  if ((cur_time - t_last_).Double() < 1 / update_rate_)
  {
    return;
  }

  // 最後の発行時間を更新
  t_last_ = cur_time;

  // 最も古い (= delay分遅れている) 状態を取得
  common::Time gps_time;
  Pose3d T_W_B;
  Vector3d W_Linvel_WB;
  Vector3d B_Angvel_WB;
  tie(gps_time, T_W_B, W_Linvel_WB, B_Angvel_WB) = history_.front();

  // 位置と速度のメッセージを更新
  timeGazeboToRos(gps_time, gps_msg_.header.stamp);
  updatePosition(T_W_B);
  updateVelocity(T_W_B.Rot(), W_Linvel_WB, B_Angvel_WB);

  // メッセージを発行
  gps_pub_.publish(gps_msg_);
}

void GazeboGpsPlugin::updatePosition(const Pose3d& T_W_B)
{
  // オフセットを考慮してGPSレシーバーの位置を計算
  const Vector3d& W_Pos_WB = T_W_B.Pos();
  const Quaterniond& W_Rot_B = T_W_B.Rot();
  Vector3d W_Pos_WS = W_Pos_WB + W_Rot_B * offset_;

  // Add the altitude of the origin to the z-coordinate
  W_Pos_WS.Z() += alt_0_;

  // Apply noise to the position
  W_Pos_WS += pos_noise_->get();

  // Fill the GPS message
  dh_std::cartToGpsRelative(
    W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_, gps_msg_.latitude, gps_msg_.longitude);
  gps_msg_.altitude = W_Pos_WS.Z();
}

void GazeboGpsPlugin::updateVelocity(
  const Quaterniond& W_Rot_B,
  const Vector3d& W_Linvel_WB,
  const Vector3d& B_Angvel_WB)
{
  // オフセットを考慮してGPSレシーバの速度を計算
  Vector3d W_Linvel_WS = W_Linvel_WB + W_Rot_B * B_Angvel_WB.Cross(offset_);

  // Apply noise to ground speed
  W_Linvel_WS += vel_noise_->get();

  // Fill the ground speed message.
  vectorGazeboToKDL(W_Linvel_WS, gps_msg_.ground_speed);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGpsPlugin);
}  // namespace gazebo
