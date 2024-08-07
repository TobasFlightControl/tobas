#include <tobas_math/core.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_constants/constants.hpp>

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
  setRandomDistribuitons();

  world_ = physics::get_world(sensor->WorldName());
  link_ = dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name_));
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  registerPublishers();
  update_connection_ = sensor->ConnectUpdated(std::bind(&self::onUpdate, this));
}

void GazeboGpsPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);

  getSdfParam(sdf, "offset", offset_, zero3);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, POSITIVE);
  getSdfParam(sdf, "delay", delay_, kDefaultDelay, NON_NEGATIVE);
  getSdfParam(sdf, "positionCorrTime", pos_corr_time_, kDefaultPosCorrTime, POSITIVE);

  getSdfParam(sdf, "horPosAccuracy", hor_pos_accuracy_, kDefaultHorPosAccuracy, NON_NEGATIVE);
  getSdfParam(sdf, "verPosAccuracy", ver_pos_accuracy_, kDefaultVerPosAccuracy, NON_NEGATIVE);
  getSdfParam(sdf, "horVelStdDev", hor_vel_stddev_, kDefaultHorVelStdDev, NON_NEGATIVE);
  getSdfParam(sdf, "verVelStdDev", ver_vel_stddev_, kDefaultVerVelStdDev, NON_NEGATIVE);

  getSdfParam(sdf, "latitudeZero", lat_0_, kDefaultLatitudeZero);
  getSdfParam(sdf, "longitudeZero", lon_0_, kDefaultLongitudeZero);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero);
}

void GazeboGpsPlugin::setRandomDistribuitons()
{
  // 位置の乱数生成器
  // バイアスの絶対値の期待値が正確度に一致するように位置のSDEの標準偏差を定める (memo: 2-57)
  const auto hor_dpos_stddev = hor_pos_accuracy_ * sqrt(M_PI / pos_corr_time_);
  const auto ver_dpos_stddev = ver_pos_accuracy_ * sqrt(M_PI / pos_corr_time_);
  const Vector3d dpos_stddev(hor_dpos_stddev, hor_dpos_stddev, ver_dpos_stddev);
  dpos_noise_.reset(new NormalDistribution3d(rnd_dev_, zero3, dpos_stddev));

  // 速度の乱数生成器
  const Vector3d vel_stddev(hor_vel_stddev_, hor_vel_stddev_, ver_vel_stddev_);
  vel_noise_.reset(new NormalDistribution3d(rnd_dev_, zero3, vel_stddev));
}

void GazeboGpsPlugin::registerPublishers()
{
  gps_pub_ = createPublisher<tobas_msgs::Gps>("/" + ns_ + "/" + tobas::kGpsTopic);
}

void GazeboGpsPlugin::onUpdate()
{
  // 現在の状態を履歴に追加
  const auto cur_time = world_->SimTime();
  history_.emplace_back(cur_time, link_->WorldPose(), link_->WorldLinearVel(), link_->RelativeAngularVel());

  // 古い履歴を削除
  while ((cur_time - get<0>(history_.front())).Double() > delay_)
  {
    history_.pop_front();
    if (!is_history_filled_)
      is_history_filled_ = true;
  }

  // ループ時刻を更新
  const auto dt = (cur_time - t_last_loop_).Double();
  t_last_loop_ = cur_time;

  // オルンシュタイン＝ウーベンレック過程に従って位置のバイアスを更新
  pos_bias_ += (dpos_noise_->get() - pos_bias_ / pos_corr_time_) * dt;

  // 履歴が溜まっていなければ発行しない
  if (!is_history_filled_)
    return;

  // 更新時刻になっていなければ発行しない
  if ((cur_time - t_last_publish_).Double() < 1 / update_rate_)
    return;

  // 最新の発行時刻を更新
  t_last_publish_ = cur_time;

  // 最も古い (= delay分遅れている) 状態を取得
  common::Time gps_time;
  Pose3d T_W_B;
  Vector3d W_Linvel_WB;
  Vector3d B_Angvel_WB;
  tie(gps_time, T_W_B, W_Linvel_WB, B_Angvel_WB) = history_.front();

  // GPSメッセージを作成
  const auto gps_msg =std::make_unique<tobas_msgs::Gps>();
  gps_msg->header.frame_id = link_name_;
  timeGazeboToRos(gps_time, gps_msg->header.stamp);
  gps_msg->fix_type = tobas_msgs::Gps::FIX_3D;
  fillCovariances(*gps_msg);
  updatePosition(*gps_msg, T_W_B);
  updateVelocity(*gps_msg, T_W_B.Rot(), W_Linvel_WB, B_Angvel_WB);

  // メッセージを発行
  gps_pub_->publish(gps_msg);
}

void GazeboGpsPlugin::fillCovariances(tobas_msgs::Gps& gps_msg)
{
  // FIXME: 正確度と共分散は異なる．しかしGNSSは白色ノイズではないし，どう共分散を計算している？
  gps_msg.position_covariance.setZero();
  gps_msg.position_covariance.diagonal()(0) = math::sqr(hor_pos_accuracy_);
  gps_msg.position_covariance.diagonal()(1) = math::sqr(hor_pos_accuracy_);
  gps_msg.position_covariance.diagonal()(2) = math::sqr(ver_pos_accuracy_);

  gps_msg.velocity_covariance.setZero();
  gps_msg.velocity_covariance.diagonal()(0) = math::sqr(hor_vel_stddev_);
  gps_msg.velocity_covariance.diagonal()(1) = math::sqr(hor_vel_stddev_);
  gps_msg.velocity_covariance.diagonal()(2) = math::sqr(ver_vel_stddev_);
}

void GazeboGpsPlugin::updatePosition(tobas_msgs::Gps& gps_msg, const Pose3d& T_W_B)
{
  // オフセットを考慮してGPSレシーバーの位置を計算
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  auto W_Pos_WS = W_Pos_WB + W_Rot_B * offset_;

  // Add the altitude of the origin to the z-coordinate
  W_Pos_WS.Z() += alt_0_;

  // 真値にバイアスを加える
  W_Pos_WS += pos_bias_;

  // Fill the GPS message
  tobas_std::cartToGpsRelative(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_, gps_msg.latitude, gps_msg.longitude);
  gps_msg.altitude = W_Pos_WS.Z();
}

void GazeboGpsPlugin::updateVelocity(
  tobas_msgs::Gps& gps_msg,
  const Quaterniond& W_Rot_B,
  const Vector3d& W_Linvel_WB,
  const Vector3d& B_Angvel_WB)
{
  // オフセットを考慮してGPSレシーバの速度を計算
  auto W_Linvel_WS = W_Linvel_WB + W_Rot_B * B_Angvel_WB.Cross(offset_);

  // Apply noise to ground speed
  W_Linvel_WS += vel_noise_->get();

  // Fill the ground speed message.
  vectorGazeboToKDL(W_Linvel_WS, gps_msg.ground_speed);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGpsPlugin);
}  // namespace gazebo
