#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include <multirotor_tools/conversions/msg_msg.hpp>
#include <multirotor_tools/conversions/eigen_msg.hpp>

#include "../../include/multirotor_gazebo/state_estimator_gt.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

StateEstimatorGT::StateEstimatorGT(ros::NodeHandle& nh)
{
  const string drone_name = dh_ros::getParam<string>("/drone_name");
  const string ns = ros::this_node::getNamespace();

  bs_pub_ = nh.advertise<multirotor_msgs::PoseVel>("/" + drone_name + "/base_state", 1, false);
  odom_sub_ =
    nh.subscribe("/" + drone_name + "/ground_truth/odometry", 1, &StateEstimatorGT::odomCb, this);
}

void StateEstimatorGT::updatePosition(const geometry_msgs::Point& pos)
{
  bs_.pose.position = pos;
}

void StateEstimatorGT::updateRotation(const geometry_msgs::Quaternion& quat)
{
  auto& rpy = bs_.pose.orientation;

  // roll, pitchはクオータニオンの変換値をそのまま代入
  quaternionToEuler(quat.x, quat.y, quat.z, quat.w, rpy.roll, rpy.pitch, yaw_now_);

  // ジャンプを考慮して更新
  updateYawJumpCount();
  rpy.yaw = (2 * M_PI) * yaw_jump_count_ + yaw_now_;

  yaw_prev_ = yaw_now_;
}

void StateEstimatorGT::updateLinearVelocity(
  const geometry_msgs::Quaternion& quat,
  const geometry_msgs::Vector3& local_linvel)
{
  Quaterniond Q_world_base;
  Vector3d local_linvel_eigen;
  tf::quaternionMsgToEigen(quat, Q_world_base);
  tf::vectorMsgToEigen(local_linvel, local_linvel_eigen);

  Vector3d world_linvel = Q_world_base * local_linvel_eigen;

  tf::linVelEigenToMsg(world_linvel, bs_.twist.linear);
}

void StateEstimatorGT::updateAngularVelocity(const geometry_msgs::Vector3& local_angvel)
{
  tf::Vector3ToAngularVelocity(
    local_angvel, bs_.twist.angular);  // Odometryのtwistは元からローカル！
}

void StateEstimatorGT::updateYawJumpCount()
{
  // 前回の観測値から180deg以上ずれていたらジャンプしたとみなす
  if (yaw_now_ - yaw_prev_ > M_PI)
  {
    yaw_jump_count_--;
  }
  else if (yaw_now_ - yaw_prev_ < -M_PI)
  {
    yaw_jump_count_++;
  }
}

void StateEstimatorGT::odomCb(const nav_msgs::Odometry& msg)
{
  updatePosition(msg.pose.pose.position);     // 位置(world座標系)
  updateRotation(msg.pose.pose.orientation);  // 姿勢(world座標系)
  updateLinearVelocity(msg.pose.pose.orientation, msg.twist.twist.linear);  // 並進速度(world座標系)
  updateAngularVelocity(msg.twist.twist.angular);  // 回転速度(local座標系！！！)

  bs_pub_.publish(bs_);
}
