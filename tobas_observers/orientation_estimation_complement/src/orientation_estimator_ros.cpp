#include <tobas_ros2_tools/rosparam.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_ros2_tools/eigen_conversion.hpp>
#include <tobas_kdl_msgs/conversion/kdl_eigen.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include <tobas_kdl_msgs/QuaternionStamped.h>
#include <tobas_msgs/Gps.h>

#include "../include/orientation_estimation_complement/orientation_estimator_ros.hpp"

using namespace std;
using namespace Eigen;

namespace orientation_estimation_complement
{
OrientationEstimatorRos::OrientationEstimatorRos(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh, const string& name)
  : super(node, pnh, name),
    imu_sub_(nh_, tobas::kImuTopic, kQueueSize, tcpNoDelay()),
    mag_sub_(nh_, tobas::kMagTopic, kQueueSize, tcpNoDelay()),
    sync_(SyncPolicy(kQueueSize), imu_sub_, mag_sub_),
    check_topics_timer_(nh_, kTimerPeriod, &OrientationEstimatorRos::checkTopicsTimerCb, this)
{
  getRosParams();
  initializeFilter();

  orientation_pub_ = nh_.advertise<tobas_kdl_msgs::QuaternionStamped>("orientation", kQueueSize);
  sync_.registerCallback(&OrientationEstimatorRos::imuMagCb, this);
}

void OrientationEstimatorRos::getRosParams()
{
  ros2::getParam(pnh_, "gain_acc", gain_acc_, kDefaultGainAcc);
  ros2::getParam(pnh_, "gain_mag", gain_mag_, kDefaultGainMag);
  ros2::getParam(pnh_, "bias_alpha", bias_alpha_, kDefaultBiasAlpha);
  ros2::getParam(pnh_, "do_bias_estimation", do_bias_estimation_, kDefaultDoBiasEstimation);
  ros2::getParam(pnh_, "do_adaptive_gain", do_adaptive_gain_, kDefaultDoAdaptiveGain);
}

void OrientationEstimatorRos::initializeFilter()
{
  tobas_msgs::Gps gps;
  if (!ros2::subscribeOnce(gps, tobas::kGpsTopic, nh_))
    TOBAS_EXIT("Failed to get GPS message.");
  const auto mag = tobas::geomag(gps.latitude, gps.longitude, gps.altitude);
  filter_.setReferenceMagneticField(mag.north, mag.east);

  if (!filter_.setGravity(tobas::kGravity))
    TOBAS_EXIT("Invalid gravity");

  if (!filter_.setGainAcc(gain_acc_))
    TOBAS_EXIT("Invalid gain_acc");

  if (do_bias_estimation_ && !filter_.setBiasAlpha(bias_alpha_))
    TOBAS_EXIT("Invalid bias_alpha");

  filter_.setDoBiasEstimation(do_bias_estimation_);
  filter_.setDoAdaptiveGain(do_adaptive_gain_);
}

void OrientationEstimatorRos::imuMagCb(const ImuMsg::ConstPtr& imu, const MagMsg::ConstPtr& mag)
{
  // Initialize
  if (imu_ == nullptr)
  {
    TOBAS_INFO("The first IMU message is received.");
    check_topics_timer_.stop();
    imu_ = imu;
    return;
  }

  // Calculate dt
  const auto dt = (imu->header.stamp - imu_->header.stamp).seconds();
  imu_ = imu;

  // Update the filter
  filter_.update(imu->accel.data, imu->gyro.data, mag->magnetic_field.data, dt);

  // Get the orientation
  const auto q = filter_.getOrientation();

  // Create the orientation message
  const auto quat_msg = boost::make_shared<tobas_kdl_msgs::QuaternionStamped>();
  quat_msg->header = imu->header;
  kdl::quaternionEigenToKDL(q, quat_msg->quaternion);

  // Publish the orientation message
  orientation_pub_.publish(quat_msg);
}

void OrientationEstimatorRos::checkTopicsTimerCb(const rclcpp::TimerEvent&)
{
  TOBAS_INFO("Waiting for ", ns(), tobas::kImuTopic);
}
}  // namespace orientation_estimation_complement
