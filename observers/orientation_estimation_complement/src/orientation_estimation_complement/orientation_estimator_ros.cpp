#include <eigen_conversions/eigen_msg.h>

#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/orientation_estimation_complement/orientation_estimator_ros.hpp"

// Constants
#define TIMER_PERIOD 5.
#define QUEUE_SIZE 5

// Default parameters
#define DEFAULT_GRAVITY 9.80665
#define DEFAULT_GAIN_ACC 0.01
#define DEFAULT_GAIN_MAG 0.01
#define DEFAULT_BIAS_ALPHA 0.01
#define DEFAULT_DO_BIAS_ESTIMATION true
#define DEFAULT_DO_ADAPTIVE_GAIN false

using namespace std;
using namespace Eigen;

OrientationEstimatorRos::OrientationEstimatorRos() : is_initialized_(false)
{
  getRosParams();
  initializeFilter();
  registerPublishers();
  registerSubscribers();

  check_topics_timer_ = nh_.createTimer(
    ros::Duration(TIMER_PERIOD), &OrientationEstimatorRos::checkTopicsTimerCb, this);
}

OrientationEstimatorRos::~OrientationEstimatorRos()
{
  check_topics_timer_.stop();
}

void OrientationEstimatorRos::getRosParams()
{
  // 共通パラメータ
  drone_name_ = dh_ros::getParam<string>("/drone_name");
  gravity_ = dh_ros::getParam<double>("/gravity", DEFAULT_GRAVITY);
  ref_mag_north_ = dh_ros::getParam<double>("/geomagnetism/north");
  ref_mag_east_ = dh_ros::getParam<double>("/geomagnetism/east");
  ref_mag_down_ = dh_ros::getParam<double>("/geomagnetism/down");

  // ノード固有のパラメータ
  gain_acc_ = dh_ros::getParam<double>("~gain_acc", DEFAULT_GAIN_ACC);
  gain_mag_ = dh_ros::getParam<double>("~gain_mag", DEFAULT_GAIN_MAG);
  bias_alpha_ = dh_ros::getParam<double>("~bias_alpha", DEFAULT_BIAS_ALPHA);
  do_bias_estimation_ = dh_ros::getParam<bool>("~do_bias_estimation", DEFAULT_DO_BIAS_ESTIMATION);
  do_adaptive_gain_ = dh_ros::getParam<bool>("~do_adaptive_gain", DEFAULT_DO_ADAPTIVE_GAIN);
}

void OrientationEstimatorRos::initializeFilter()
{
  filter_.setReferenceMagneticField(ref_mag_north_, ref_mag_east_, ref_mag_down_);

  if (!filter_.setGravity(gravity_))
  {
    dh_ros::rosWarn("Invalid gravity");
  }

  if (!filter_.setGainAcc(gain_acc_))
  {
    dh_ros::rosWarn("Invalid gain_acc");
  }

  if (do_bias_estimation_)
  {
    if (!filter_.setBiasAlpha(bias_alpha_))
    {
      dh_ros::rosWarn("Invalid bias_alpha");
    }
  }

  filter_.setDoBiasEstimation(do_bias_estimation_);
  filter_.setDoAdaptiveGain(do_adaptive_gain_);
}

void OrientationEstimatorRos::registerPublishers()
{
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/" + drone_name_ + "/filtered_imu", QUEUE_SIZE);
}

void OrientationEstimatorRos::registerSubscribers()
{
  imu_sub_.reset(new ImuSubscriber(nh_, "/" + drone_name_ + "/imu", QUEUE_SIZE));
  mag_sub_.reset(new MagSubscriber(nh_, "/" + drone_name_ + "/magnetic_field", QUEUE_SIZE));
  sync_.reset(new Synchronizer(SyncPolicy(QUEUE_SIZE), *imu_sub_, *mag_sub_));
  sync_->registerCallback(&OrientationEstimatorRos::imuMagCb, this);
}

void OrientationEstimatorRos::imuMagCb(const ImuMsg& imu, const MagMsg& mag)
{
  const ros::Time& time = imu.header.stamp;
  tf::vectorMsgToEigen(imu.linear_acceleration, a_);
  tf::vectorMsgToEigen(imu.angular_velocity, w_);
  tf::vectorMsgToEigen(mag.magnetic_field, m_);

  // Initialize
  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    time_prev_ = time;
    is_initialized_ = true;
    return;
  }

  // Calculate dt
  double dt = (time - time_prev_).toSec();
  time_prev_ = time;

  // Update the filter
  filter_.update(a_, w_, m_, dt);

  // Get the orientation
  Quaterniond q = filter_.getOrientation();

  // Create fitlered IMU message
  ImuMsg filtered_imu = imu;
  filtered_imu.orientation_covariance.fill(-1);
  tf::quaternionEigenToMsg(q, filtered_imu.orientation);

  // Account for biases
  if (do_bias_estimation_)
  {
    w_ -= filter_.getAngularVelocityBias();
    tf::vectorEigenToMsg(w_, filtered_imu.angular_velocity);
  }

  // Publish filtered IMU message
  imu_pub_.publish(filtered_imu);
}

void OrientationEstimatorRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  dh_ros::rosWarn("IMU data is not received yet.");
}
