#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

namespace dh_ros
{
/* Vector3のL2ノルムを計算する． */
double norm(const geometry_msgs::Vector3& v);

/* JointStateの各フィールドのサイズが合っているかを調べる． */
bool isFieldSizeMatch(const sensor_msgs::JointState& js);

/* JointStateを初期化． */
void clear(sensor_msgs::JointState& js);

/* JointStateをリサイズ． */
void resize(sensor_msgs::JointState& js, const size_t& size);

template <typename MsgType>
void _subscribeOnceCb(
  const boost::shared_ptr<const MsgType>& input_msg,
  MsgType& msg,
  bool& received)
{
  msg = *input_msg;
  received = true;
}

/* ROSメッセージを1度だけ取得する． */
template <typename MsgType>
bool subscribeOnce(
  MsgType& msg,
  const std::string& topic,
  ros::NodeHandle& nh,
  const double& timeout = 5.)
{
  bool received = false;
  ros::Subscriber sub = nh.subscribe<MsgType>(
    topic, 1, boost::bind(&_subscribeOnceCb<MsgType>, _1, boost::ref(msg), boost::ref(received)));
  ros::Rate rate(10);  // 10 Hz

  const ros::Time start_time = ros::Time::now();
  while (ros::ok() && !received && (ros::Time::now() - start_time).toSec() < timeout)
  {
    ros::spinOnce();
    rate.sleep();
  }

  return received;
}
}  // namespace dh_ros
