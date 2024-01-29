#include <sensor_msgs/NavSatFix.h>

#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/util.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_ros_tools/eigen_conversion.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../include/orientation_estimation_complement/orientation_estimator_ros.hpp"

using namespace std;
using namespace Eigen;

namespace orientation_estimation_complement
{
OrientationEstimatorRos::OrientationEstimatorRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    is_initialized_(false),
    imu_sub_(nh_, tobas::kImuTopic, kQueueSize, tcpNoDelay()),
    mag_sub_(nh_, tobas::kMagTopic, kQueueSize, tcpNoDelay()),
    sync_(SyncPolicy(kQueueSize), imu_sub_, mag_sub_),
    check_topics_timer_(nh_, kTimerPeriod, &OrientationEstimatorRos::checkTopicsTimerCb, this)
{
  getRosParams();
  initializeFilter();
  registerPublishers();
  registerSubscribers();
}

void OrientationEstimatorRos::getRosParams()
{
  tobas_ros::getParam(pnh_, "gain_acc", gain_acc_, kDefaultGainAcc);
  tobas_ros::getParam(pnh_, "gain_mag", gain_mag_, kDefaultGainMag);
  tobas_ros::getParam(pnh_, "bias_alpha", bias_alpha_, kDefaultBiasAlpha);
  tobas_ros::getParam(pnh_, "do_bias_estimation", do_bias_estimation_, kDefaultDoBiasEstimation);
  tobas_ros::getParam(pnh_, "do_adaptive_gain", do_adaptive_gain_, kDefaultDoAdaptiveGain);
}

void OrientationEstimatorRos::registerPublishers()
{
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("filtered_imu", kQueueSize);
}

void OrientationEstimatorRos::registerSubscribers()
{
  sync_.registerCallback(&OrientationEstimatorRos::imuMagCb, this);
}

void OrientationEstimatorRos::initializeFilter()
{
  sensor_msgs::NavSatFix gps;
  if (!tobas_ros::subscribeOnce(gps, tobas::kGpsTopic, nh_))
  {
    ROS_THROW_NAMED(name_, "Failed to get GPS message.");
  }
  const auto mag = tobas::geomag(gps.latitude, gps.longitude, gps.altitude);
  filter_.setReferenceMagneticField(mag.north, mag.east);

  if (!filter_.setGravity(tobas::kGravity))
  {
    ROS_THROW_NAMED(name_, "Invalid gravity");
  }

  if (!filter_.setGainAcc(gain_acc_))
  {
    ROS_THROW_NAMED(name_, "Invalid gain_acc");
  }

  if (do_bias_estimation_)
  {
    if (!filter_.setBiasAlpha(bias_alpha_))
    {
      ROS_THROW_NAMED(name_, "Invalid bias_alpha");
    }
  }

  filter_.setDoBiasEstimation(do_bias_estimation_);
  filter_.setDoAdaptiveGain(do_adaptive_gain_);
}

void OrientationEstimatorRos::imuMagCb(const ImuMsg::ConstPtr& imu, const MagMsg::ConstPtr& mag)
{
  const auto& cur_time = imu->header.stamp;
  tobas_ros::vectorMsgToEigen(imu->linear_acceleration, a_);
  tobas_ros::vectorMsgToEigen(imu->angular_velocity, w_);
  tobas_ros::vectorMsgToEigen(mag->magnetic_field, m_);

  // Initialize
  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    time_prev_ = cur_time;
    is_initialized_ = true;
    return;
  }

  // Calculate dt
  const auto dt = (cur_time - time_prev_).toSec();
  time_prev_ = cur_time;

  // Update the filter
  filter_.update(a_, w_, m_, dt);

  // Get the orientation
  const auto q = filter_.getOrientation();

  // Create fitlered IMU message
  const auto filtered_imu = boost::make_shared<ImuMsg>(*imu);
  filtered_imu->orientation_covariance.fill(nan(tobas::kUnknown));
  tobas_ros::quaternionEigenToMsg(q, filtered_imu->orientation);

  // Account for biases
  if (do_bias_estimation_)
  {
    w_ -= filter_.getAngularVelocityBias();
    tobas_ros::vectorEigenToMsg(w_, filtered_imu->angular_velocity);
  }

  // Publish filtered IMU message
  imu_pub_.publish(filtered_imu);
}

void OrientationEstimatorRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  rosInfo(name_, "Waiting for " << ns() << tobas::kImuTopic);
}
}  // namespace orientation_estimation_complement
