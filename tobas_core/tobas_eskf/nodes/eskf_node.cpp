// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tf2_ros/transform_broadcaster.h>

#include <tobas_algorithm/core.hpp>
#include <tobas_algorithm/kahan.hpp>
#include <tobas_constants/frame.hpp>
#include <tobas_constants/imu.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_geomag/core.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/gnss.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_time_tools/util.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tobas_debug_msgs_adapter/observer_feedback.hpp>
#include <tobas_kdl_msgs_adapter/frame_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_eskf/eskf.hpp"
#include "tobas_eskf/util.hpp"

using namespace Eigen;

namespace tobas
{
class ErrorStateKalmanFilterNode : public BaseNode
{
  using self = ErrorStateKalmanFilterNode;
  using super = BaseNode;

  using GetOriginSrv = tobas_msgs::srv::GetGnssOrigin;
  using SetOriginSrv = tobas_msgs::srv::SetGnssOrigin;

  // Initial standard deviations.
  // Covariance grows slowly but converges fairly quickly, so choosing a somewhat large value is acceptable.
  static constexpr double kInitPosStddev = 5.0;     // [m]
  static constexpr double kInitVelStddev = 1.0;     // [m/s]
  static constexpr double kInitRotStddev = M_PI_4;  // [rad]
  static constexpr double kInitMagStddev = 0.5;     // [-]

  // Other constants
  static constexpr double kAccurateAttitudeStddevThresh = 0.05;  // [rad]
  static constexpr size_t kInitMagCount = 100;
  static constexpr size_t kInitBaroCount = 100;

public:
  explicit ErrorStateKalmanFilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  eskf::ErrorStateKalmanFilter eskf_;

  Vector3d pos_meas_;
  Matrix6d gnss_cov_ = Matrix6d::Zero();
  tobas_msgs::Imu::ConstSharedPtr imu_raw_, imu_filt_;
  tobas_msgs::MagneticField::ConstSharedPtr mag_;
  tobas_msgs::Gnss::ConstSharedPtr gnss_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  bool gnss_fix_ = false;
  double gnss_anomaly_score_ = 0.0;

  // Geomagnetic reference value.
  bool mag_ref_set_ = false;
  size_t init_mag_cnt_ = 0;
  std::array<algo::Kahan<double>, 3> init_mag_sum_;

  // Barometric altitude origin.
  double baro_alt_origin_;  // Barometric altitude origin.
  bool baro_alt_origin_set_ = false;
  size_t init_pres_cnt_ = 0;
  algo::Kahan<double> init_pres_sum_;

  // GNSS coordinate origin.
  struct GeoPoint
  {
    double latitude;
    double longitude;
    double altitude;
  } gnss_origin_;  // GNSS coordinate origin.

  // Static parameters
  std::string frame_id_;
  bool use_mag_;
  bool use_baro_;
  bool use_gnss_;
  bool use_ext_pose_;
  bool adaptive_gnss_noise_;
  bool adaptive_grav_noise_;
  bool do_acc_bias_estimation_;
  bool do_gyro_bias_estimation_;
  bool do_mag_hard_bias_estimation_;
  bool do_mag_soft_bias_estimation_;
  bool do_baro_alt_bias_estimation_;
  bool do_grav_estimation_;
  Vector3d imu_offset_;   // [m] IMU position relative to the root link (Local).
  Vector3d gnss_offset_;  // [m] GNSS receiver position relative to the root link (Local).

  // Dynamic parameters
  Matrix3d fixed_acc_cov_ = Matrix3d::Zero();       // [m^2/s^4]
  Matrix3d fixed_gyro_cov_ = Matrix3d::Zero();      // [rad^2/s^2]
  Matrix3d fixed_mag_cov_ = Matrix3d::Zero();       // [-]
  double fixed_head_var_;                           // [rad^2]
  double fixed_baro_alt_var_;                       // [m^2]
  Matrix3d fixed_gnss_pos_cov_ = Matrix3d::Zero();  // [m^2]
  Matrix3d fixed_gnss_vel_cov_ = Matrix3d::Zero();  // [m^2/s^2]
  Matrix3d fixed_grav_cov_ = Matrix3d::Zero();      // [m^2/s^4]
  double grav_stddev_min_;                          // [m/s^2]
  double grav_stddev_max_;                          // [m/s^2]
  double grav_stddev_rate_;                         // [-]

  // Publishers
  ros2::PublisherPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_pub_;
  ros2::PublisherPtr<tobas_msgs::MagneticField> mag_ref_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::GeodeticCoordinates> gnss_origin_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::ObserverFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_raw_sub_;
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_filt_sub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticField> mag_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressure> baro_sub_;
  ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::FrameWithCovarianceStamped> ext_pose_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;

  // Services
  ros2::ServiceServerPtr<GetOriginSrv> get_gnss_origin_ss_;
  ros2::ServiceServerPtr<SetOriginSrv> set_gnss_origin_ss_;

  // TF
  geometry_msgs::msg::TransformStamped tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  void getStaticRosParams();
  void setupTransformMessage();
  void registerDynamicRosParams();
  void registerRosInterfaces();

  void setMagneticFieldRef(const Vector3d& mag_W);
  void fillOdometryMsg(tobas_msgs::OdometryWithCovarianceStamped& odom) const;
  void publishMagRef(const Vector3d& mag_W) const;
  void publishGnssOrigin(double lat, double lon, double alt) const;
  void publishFeedback(const std_msgs::msg::Header& header) const;
  double calcGravMeasNoiseStddev(const Vector3d& acc) const;
  Matrix3d calcGravMeasNoiseCov(const Vector3d& acc) const;

  double initAccelBiasStddev() const;
  double initGyroBiasStddev() const;
  double initMagHardBiasStddev() const;
  double initMagSoftBiasStddev() const;
  double initBaroAltBiasStddev() const;
  double initGravBiasStddev() const;

  bool fixedAccMeasNoiseStddevCb(const double& p);
  bool fixedGyroMeasNoiseStddevCb(const double& p);
  bool fixedMagMeasNoiseStddevCb(const double& p);
  bool fixedHeadMeasNoiseStddevCb(const double& p);
  bool fixedBaroAltMeasNoiseStddevCb(const double& p);
  bool fixedGnssPosMeasNoiseStddevCb(const double& p);
  bool fixedGnssVelMeasNoiseStddevCb(const double& p);
  bool fixedGravMeasNoiseStddevCb(const double& p);
  bool adaptiveGravMeasNoiseStddevMinCb(const double& p);
  bool adaptiveGravMeasNoiseStddevMaxCb(const double& p);
  bool adaptiveGravMeasNoiseStddevRateCb(const double& p);
  bool accBiasProcNoiseDensityCb(const double& p);
  bool gyroBiasProcNoiseDensityCb(const double& p);
  bool magHardBiasProcNoiseDensityCb(const double& p);
  bool magSoftBiasProcNoiseDensityCb(const double& p);
  bool baroAltBiasProcNoiseDensityCb(const double& p);
  bool gravProcNoiseDensityCb(const double& ud_ug);

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void imuFiltCb(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg);
  void baroCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& msg);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& msg);
  void externalPoseCb(const tobas_kdl_msgs::FrameWithCovarianceStamped::ConstSharedPtr& msg);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& msg);

  void getGnssOriginCb(const GetOriginSrv::Request::ConstSharedPtr& req, const GetOriginSrv::Response::SharedPtr& res);
  void setGnssOriginCb(const SetOriginSrv::Request::ConstSharedPtr& req, const SetOriginSrv::Response::SharedPtr& res);
};

ErrorStateKalmanFilterNode::ErrorStateKalmanFilterNode(const rclcpp::NodeOptions& options)
  : super(node::kObserver, nodeOptions_DParam(options)), tf_br_(this)
{
  getStaticRosParams();
  setupTransformMessage();
  registerDynamicRosParams();
  registerRosInterfaces();
}

void ErrorStateKalmanFilterNode::getStaticRosParams()
{
  frame_id_ = getStringParam("frame_id");

  use_mag_ = getBoolParam("use_magnetometer");
  use_baro_ = getBoolParam("use_barometer");
  use_gnss_ = getBoolParam("use_gnss");
  use_ext_pose_ = getBoolParam("use_external_pose");

  adaptive_gnss_noise_ = getBoolParam("adaptive_gnss_noise");
  adaptive_grav_noise_ = getBoolParam("adaptive_grav_noise");

  do_acc_bias_estimation_ = getBoolParam("do_acc_bias_estimation");
  do_gyro_bias_estimation_ = getBoolParam("do_gyro_bias_estimation");
  do_mag_hard_bias_estimation_ = getBoolParam("do_mag_hard_bias_estimation");
  do_mag_soft_bias_estimation_ = getBoolParam("do_mag_soft_bias_estimation");
  do_baro_alt_bias_estimation_ = getBoolParam("do_baro_alt_bias_estimation");
  do_grav_estimation_ = getBoolParam("do_gravity_estimation");

  const auto imu_offset = getDoubleArrayParam("imu_offset");
  const auto gnss_offset = getDoubleArrayParam("gnss_offset");
  imu_offset_ = Map<const Vector3d>(imu_offset.data());
  gnss_offset_ = Map<const Vector3d>(gnss_offset.data());
}

void ErrorStateKalmanFilterNode::setupTransformMessage()
{
  // Fill the static part of the transform message
  tf_.header.frame_id = frame::kWorld;
  tf_.child_frame_id = frame_id_;
}

void ErrorStateKalmanFilterNode::registerDynamicRosParams()
{
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_ACC_NOISE
  addDynamicDoubleParam("acc_meas_noise_stddev", &self::fixedAccMeasNoiseStddevCb, this, 0.05, 20, 1, 20, " m/s^2");

  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GYR_NOISE
  addDynamicDoubleParam("gyro_meas_noise_stddev", &self::fixedGyroMeasNoiseStddevCb, this, 0.005, 20, 1, 20, " rad/s");

  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_MAG_NOISE
  addDynamicDoubleParam("mag_meas_noise_stddev", &self::fixedMagMeasNoiseStddevCb, this, 5.0, 1, 1, 20, " uT");

  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_HEAD_NOISE
  addDynamicDoubleParam("head_meas_noise_stddev", &self::fixedHeadMeasNoiseStddevCb, this, 0.05, 6, 1, 20, " rad");

  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_BARO_NOISE
  addDynamicDoubleParam("baro_alt_meas_noise_stddev", &self::fixedBaroAltMeasNoiseStddevCb, this, 0.5, 7, 1, 30, " m");

  if (!adaptive_gnss_noise_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GPS_P_NOISE
    addDynamicDoubleParam(
      "gnss_pos_meas_noise_stddev", &self::fixedGnssPosMeasNoiseStddevCb, this, 0.1, 5, 1, 100, " m");

    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GPS_V_NOISE
    addDynamicDoubleParam(
      "gnss_vel_meas_noise_stddev", &self::fixedGnssVelMeasNoiseStddevCb, this, 0.1, 3, 1, 50, " m/s");
  }

  if (adaptive_grav_noise_) {
    addDynamicDoubleParam(
      "grav_meas_noise_stddev_min", &self::adaptiveGravMeasNoiseStddevMinCb, this, 0.01, 1, 0, 100, " m/s^2");

    addDynamicDoubleParam(
      "grav_meas_noise_stddev_max", &self::adaptiveGravMeasNoiseStddevMaxCb, this, 1.0, 20, 10, 100, " m/s^2");

    addDynamicDoubleParam(
      "grav_meas_noise_stddev_rate", &self::adaptiveGravMeasNoiseStddevRateCb, this, 5.0, 20, 0, 100);
  }
  else {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GRAV_NOISE
    addDynamicDoubleParam("grav_meas_noise_stddev", &self::fixedGravMeasNoiseStddevCb, this, 0.1, 3, 1, 20, " g");
  }
  if (do_acc_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_ACC_B_NOISE
    addDynamicDoubleParam(
      "acc_bias_proc_noise_density", &self::accBiasProcNoiseDensityCb, this, 1.0, 15, 0, 50, " ug/s/√Hz");
  }
  if (do_gyro_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GYR_B_NOISE
    addDynamicDoubleParam(
      "gyro_bias_proc_noise_density", &self::gyroBiasProcNoiseDensityCb, this, 1.0, 3, 0, 30, " mdps/s/√Hz");
  }
  if (do_mag_hard_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_MAG_B_NOISE
    addDynamicDoubleParam(
      "mag_hard_bias_proc_noise_density", &self::magHardBiasProcNoiseDensityCb, this, 0.1, 5, 0, 100, " nT/s/√Hz");
  }
  if (do_mag_soft_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_MAG_B_NOISE
    addDynamicDoubleParam(
      "mag_soft_bias_proc_noise_density", &self::magSoftBiasProcNoiseDensityCb, this, 0.1, 5, 0, 100, " nT/s/√Hz");
  }
  if (do_baro_alt_bias_estimation_) {
    // cf. https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/common.h (baro_bias_nsd)
    addDynamicDoubleParam(
      "baro_alt_bias_proc_noise_density", &self::baroAltBiasProcNoiseDensityCb, this, 0.01, 13, 0, 100, " m/s/√Hz");
  }
  if (do_grav_estimation_) {
    addDynamicDoubleParam(
      "grav_noise_proc_noise_density", &self::gravProcNoiseDensityCb, this, 1.0, 15, 0, 50, " ug/s/√Hz");
  }
}

void ErrorStateKalmanFilterNode::registerRosInterfaces()
{
  // Register publishers
  odom_pub_ = createPublisher<tobas_msgs::OdometryWithCovarianceStamped>(topic::kOdometry);
  mag_ref_pub_ = createPublisher<tobas_msgs::MagneticField>(topic::kMagRef, true, true);
  gnss_origin_pub_ = createPublisher<tobas_msgs::msg::GeodeticCoordinates>(topic::kGnssOrigin, true, true);
  feedback_pub_ = createPublisher<tobas_debug_msgs::ObserverFeedback>(topic::kObsvFeedback);

  // Register subscribers
  imu_raw_sub_ = createSubscriber(topic::kImuRaw, &self::imuRawCb, this);
  imu_filt_sub_ = createSubscriber(topic::kImuFilt, &self::imuFiltCb, this);
  if (use_mag_) {
    mag_sub_ = createSubscriber(topic::kMagneticField, &self::magCb, this);
  }
  if (use_baro_) {
    baro_sub_ = createSubscriber(topic::kAirPressure, &self::baroCb, this);
  }
  if (use_gnss_) {
    gnss_sub_ = createSubscriber(topic::kGnss, &self::gnssCb, this);
  }
  if (use_ext_pose_) {
    ext_pose_sub_ = createSubscriber(topic::kExternalPose, &self::externalPoseCb, this);
  }
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);

  // Register service servers
  get_gnss_origin_ss_ = createService<GetOriginSrv>(service::kGetGnssOrigin, &self::getGnssOriginCb, this);
  set_gnss_origin_ss_ = createService<SetOriginSrv>(service::kSetGnssOrigin, &self::setGnssOriginCb, this);
}

void ErrorStateKalmanFilterNode::setMagneticFieldRef(const Vector3d& mag_W)
{
  // Set the geomagnetic reference value.
  eskf_.setMagneticFieldRef(mag_W);

  // Initialize magnetometer bias.
  eskf_.initializeMagHardBias(Vector3d::Zero(), Vector3d::Constant(math::sqr(initMagHardBiasStddev())).asDiagonal());
  eskf_.initializeMagSoftBias(Matrix3d::Identity(), Vector6d::Constant(math::sqr(initMagSoftBiasStddev())).asDiagonal());

  // Initialize yaw if geomagnetic data has already been received.
  // Otherwise, when the yaw error is too large, even roll and pitch may be affected by the feedback.
  if (mag_) {
    // Get the current RPY.
    const auto R_W_B = eskf_.getQuaternion();
    const auto [old_roll, old_pitch, old_yaw] = st::eulerFromQuaternion(R_W_B.x(), R_W_B.y(), R_W_B.z(), R_W_B.w());

    // Move the geomagnetic field to ground coordinate system `G`,
    // whose yaw alone matches the body and whose XY axes are parallel to the ground.
    const AngleAxisd R_G_B(old_yaw, Vector3d::UnitZ());
    const auto mag_G = R_G_B.inverse() * (R_W_B * mag_->mag.data);  // Reduce computation by evaluating from the back.
    const auto& mx = mag_G.x();
    const auto& my = mag_G.y();

    // Compute yaw from the new reference.
    const auto yaw_ref = std::atan2(mag_W.y(), mag_W.x());
    const auto new_yaw = algo::wrapPi(yaw_ref - std::atan2(my, mx));

    // Compute the quaternion corrected only in yaw.
    const auto new_q = eigen::quaternionFromRPY(old_roll, old_pitch, new_yaw);

    // Correct the yaw component of attitude covariance.
    auto rot_cov = eskf_.getRotationCovariance();
    rot_cov.row(2).setZero();
    rot_cov.col(2).setZero();
    rot_cov(2, 2) = math::sqr(kInitRotStddev);

    // Initialize attitude.
    eskf_.initializeQuaternion(new_q, rot_cov);
  }

  // Publish the geomagnetic reference value.
  publishMagRef(mag_W);

  TOBAS_INFO("The reference magnetic field has been set to ", mag_W.transpose(), ".");
  mag_ref_set_ = true;
}

void ErrorStateKalmanFilterNode::fillOdometryMsg(tobas_msgs::OdometryWithCovarianceStamped& odom) const
{
  const Vector3d W_Pos_WI = eskf_.getPosition();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_grav = B_Rot_W * Vector3d(0, 0, -eskf_.getGravity());
  const Vector3d B_Acc = imu_filt_->accel.data - eskf_.getAccelBias() + B_grav;  // Acceleration excluding gravity.
  const Vector3d B_Gyro = imu_filt_->gyro.data - eskf_.getGyroBias();

  // Header
  odom.header.stamp = imu_raw_->header.stamp;
  odom.header.frame_id = frame::kWorld;

  // Position (Global): IMU frame -> Base frame
  odom.odom.odom.frame.p.data = W_Pos_WI - W_Rot_B * imu_offset_;
  odom.odom.position_covariance = eskf_.getPositionCovariance();

  // Linear velocity (Local): IMU frame -> Base frame
  odom.odom.odom.twist.vel.data = B_Rot_W * W_Vel_WI - B_Gyro.cross(imu_offset_);
  odom.odom.velocity_covariance = B_Rot_W * eskf_.getVelocityCovariance() * W_Rot_B;

  // Orientation (Global)
  odom.odom.odom.frame.M.data = W_Rot_B.toRotationMatrix();
  odom.odom.orientation_covariance = eskf_.getRotationCovariance();

  // Angular velocity (Local)
  odom.odom.odom.twist.rot.data = B_Gyro;
  odom.odom.gyro_covariance = fixed_gyro_cov_ + eskf_.getGyroBiasCovariance();

  // Linear acceleration (Local)
  odom.odom.odom.accel.linear.data = B_Acc;

  // Angular acceleration (Local)
  odom.odom.odom.accel.angular = imu_filt_->dgyro;
}

void ErrorStateKalmanFilterNode::publishMagRef(const Vector3d& mag_W) const
{
  auto msg = std::make_unique<tobas_msgs::MagneticField>();

  msg->header.stamp = now();
  msg->mag = mag_W;

  mag_ref_pub_->publish(std::move(msg));
}

void ErrorStateKalmanFilterNode::publishGnssOrigin(double lat, double lon, double alt) const
{
  auto msg = std::make_unique<tobas_msgs::msg::GeodeticCoordinates>();

  msg->header.stamp = now();
  msg->latitude = lat;
  msg->longitude = lon;
  msg->altitude = alt;

  gnss_origin_pub_->publish(std::move(msg));
}

void ErrorStateKalmanFilterNode::publishFeedback(const std_msgs::msg::Header& header) const
{
  // Create
  auto feedback = std::make_unique<tobas_debug_msgs::ObserverFeedback>();

  // Header
  feedback->header = header;

  // Data
  feedback->position = eskf_.getPosition();
  feedback->velocity = eskf_.getVelocity();
  feedback->hamilton = eskf_.getHamilton();
  feedback->accel_bias = eskf_.getAccelBias();
  feedback->gyro_bias = eskf_.getGyroBias();
  feedback->mag_hard_bias = eskf_.getMagHardBias();
  feedback->mag_soft_bias = eskf_.getMagSoftBias();
  feedback->baro_alt_bias = eskf_.getBaroAltBias();
  feedback->gravity = eskf_.getGravity();

  // Variance
  feedback->position_cov = eskf_.getPositionCovariance();
  feedback->velocity_cov = eskf_.getVelocityCovariance();
  feedback->rotation_cov = eskf_.getRotationCovariance();
  feedback->accel_bias_cov = eskf_.getAccelBiasCovariance();
  feedback->gyro_bias_cov = eskf_.getGyroBiasCovariance();
  feedback->mag_hard_bias_cov = eskf_.getMagHardBiasCovariance();
  feedback->mag_soft_bias_cov = eskf_.getMagSoftBiasCovariance();
  feedback->baro_alt_bias_var = eskf_.getBaroAltBiasVariance();
  feedback->gravity_var = eskf_.getGravityVariance();

  // Other
  feedback->gnss_anomaly_score = gnss_anomaly_score_;

  // Publish
  feedback_pub_->publish(std::move(feedback));
}

double ErrorStateKalmanFilterNode::calcGravMeasNoiseStddev(const Vector3d& acc) const
{
  // Determine the uncertainty of the gravity-direction observation from the L2 norm of acceleration.
  // It is intuitive to consider the attitude observation from acceleration less certain as the error between
  // the acceleration magnitude and gravitational acceleration grows,
  // because non-gravitational acceleration is being generated.
  // However, that error is not normally distributed and is not uniformly reliable,
  // so there is no basis for using the error directly as the standard deviation.
  // In practice, reducing the gravity-direction variance can make acceleration in the direction of travel
  // look like a larger-than-actual tilt during translation.
  // The controller then tries to recover the attitude, acceleration tracking in the translation direction lags,
  // and position control oscillates.
  // Since dynamic acceleration is not explicitly included in the model, its uncertainty must be determined heuristically.
  // In practice, it should be chosen by considering the trade-off between tracking lag during motion
  // and convergence speed while stationary.

  // TODO: Consider the modeled dynamic acceleration.
  // TODO: Consider other profiles?
  const auto acc_norm_diff = std::abs(acc.norm() - eskf_.getGravity());
  const auto grav_stddev = grav_stddev_min_ + grav_stddev_rate_ * acc_norm_diff;
  return std::min(grav_stddev, grav_stddev_max_);
}

Matrix3d ErrorStateKalmanFilterNode::calcGravMeasNoiseCov(const Vector3d& acc) const
{
  const auto grav_stddev = calcGravMeasNoiseStddev(acc);
  const auto grav_var = math::sqr(grav_stddev);
  return Vector3d::Constant(grav_var).asDiagonal();
}

double ErrorStateKalmanFilterNode::initAccelBiasStddev() const
{
  return do_acc_bias_estimation_ ? 1.0 : 0.0;
}

double ErrorStateKalmanFilterNode::initGyroBiasStddev() const
{
  return do_gyro_bias_estimation_ ? 0.1 : 0.0;
}

double ErrorStateKalmanFilterNode::initMagHardBiasStddev() const
{
  return do_mag_hard_bias_estimation_ ? 0.1 : 0.0;
}

double ErrorStateKalmanFilterNode::initMagSoftBiasStddev() const
{
  return do_mag_soft_bias_estimation_ ? 0.1 : 0.0;
}

double ErrorStateKalmanFilterNode::initBaroAltBiasStddev() const
{
  return do_baro_alt_bias_estimation_ ? 0.1 : 0.0;
}

double ErrorStateKalmanFilterNode::initGravBiasStddev() const
{
  return do_grav_estimation_ ? 0.1 : 0.0;
}

bool ErrorStateKalmanFilterNode::fixedAccMeasNoiseStddevCb(const double& p)
{
  const auto acc_stddev = p;  // [m/s^2]
  const auto acc_var = math::sqr(acc_stddev);
  fixed_acc_cov_.diagonal().fill(acc_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGyroMeasNoiseStddevCb(const double& p)
{
  const auto gyro_stddev = p;  // [rad/s]
  const auto gyro_var = math::sqr(gyro_stddev);
  fixed_gyro_cov_.diagonal().fill(gyro_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedMagMeasNoiseStddevCb(const double& p)
{
  const auto mag_stddev = p * 1e-2 / st::kGeomagScale;  // [-]
  const auto mag_var = math::sqr(mag_stddev);
  fixed_mag_cov_.diagonal().fill(mag_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedHeadMeasNoiseStddevCb(const double& p)
{
  const auto head_stddev = p;  // [rad]
  fixed_head_var_ = math::sqr(head_stddev);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedBaroAltMeasNoiseStddevCb(const double& p)
{
  const auto baro_alt_stddev = p;  // [m]
  fixed_baro_alt_var_ = math::sqr(baro_alt_stddev);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGnssPosMeasNoiseStddevCb(const double& p)
{
  assert(!adaptive_gnss_noise_);

  const auto gnss_pos_stddev = p;  // [m]
  const auto gnss_pos_var = math::sqr(gnss_pos_stddev);
  fixed_gnss_pos_cov_.diagonal().fill(gnss_pos_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGnssVelMeasNoiseStddevCb(const double& p)
{
  assert(!adaptive_gnss_noise_);

  const auto gnss_vel_stddev = p;  // [m/s]
  const auto gnss_vel_var = math::sqr(gnss_vel_stddev);
  fixed_gnss_vel_cov_.diagonal().fill(gnss_vel_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGravMeasNoiseStddevCb(const double& p)
{
  assert(!adaptive_grav_noise_);

  const auto grav_stddev = p * st::kGravity;  // [m/s^2]
  const auto grav_var = math::sqr(grav_stddev);
  fixed_grav_cov_.diagonal().fill(grav_var);

  return true;
}

bool ErrorStateKalmanFilterNode::adaptiveGravMeasNoiseStddevMinCb(const double& p)
{
  assert(adaptive_grav_noise_);
  grav_stddev_min_ = p;
  return true;
}

bool ErrorStateKalmanFilterNode::adaptiveGravMeasNoiseStddevMaxCb(const double& p)
{
  assert(adaptive_grav_noise_);
  grav_stddev_max_ = p;
  return true;
}

bool ErrorStateKalmanFilterNode::adaptiveGravMeasNoiseStddevRateCb(const double& p)
{
  assert(adaptive_grav_noise_);
  grav_stddev_rate_ = p;
  return true;
}

bool ErrorStateKalmanFilterNode::accBiasProcNoiseDensityCb(const double& p)
{
  assert(do_acc_bias_estimation_);

  const auto nd = p * 1e-6 * st::kGravity;  // ug/s/√Hz -> m/s^3/√Hz
  return eskf_.setAccBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::gyroBiasProcNoiseDensityCb(const double& p)
{
  assert(do_gyro_bias_estimation_);

  const auto nd = p * 1e-3 * st::kDeg2Rad;  // mdps/s/√Hz -> rad/s^2/√Hz
  return eskf_.setGyroBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::magHardBiasProcNoiseDensityCb(const double& p)
{
  assert(do_mag_hard_bias_estimation_);

  const auto nd = p * 1e-5 / st::kGeomagScale;  // nT/s/√Hz -> /s/√Hz
  return eskf_.setMagHardBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::magSoftBiasProcNoiseDensityCb(const double& p)
{
  assert(do_mag_soft_bias_estimation_);

  const auto nd = p * 1e-5 / st::kGeomagScale;  // nT/s/√Hz -> /s/√Hz
  return eskf_.setMagSoftBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::baroAltBiasProcNoiseDensityCb(const double& p)
{
  assert(do_baro_alt_bias_estimation_);

  return eskf_.setBaroAltBiasProcNoiseDensity(p);
}

bool ErrorStateKalmanFilterNode::gravProcNoiseDensityCb(const double& p)
{
  assert(do_grav_estimation_);

  const auto nd = p * 1e-6 * st::kGravity;  // ug/s/√Hz -> m/s^3/√Hz
  return eskf_.setGravProcNoiseDensity(nd);
}

void ErrorStateKalmanFilterNode::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& msg)
{
  // Compute IMU time
  const auto cur_time = ros2::chronoFromRosTime(msg->header.stamp);

  // Initialization
  if (!imu_raw_) {
    eskf_.initialize(
      Vector3d::Zero(),                                                     // Init position
      Vector3d::Constant(math::sqr(kInitPosStddev)).asDiagonal(),           // Init position covariance
      Vector3d::Zero(),                                                     // Init velocity
      Vector3d::Constant(math::sqr(kInitVelStddev)).asDiagonal(),           // Init velocity covariance
      Quaterniond::Identity(),                                              // Init quaternion
      Vector3d::Constant(math::sqr(kInitRotStddev)).asDiagonal(),           // Init rotation covariance
      Vector3d::Zero(),                                                     // Init accel bias
      Vector3d::Constant(math::sqr(initAccelBiasStddev())).asDiagonal(),    // Init accel bias covariance
      Vector3d::Zero(),                                                     // Init gyro bias
      Vector3d::Constant(math::sqr(initGyroBiasStddev())).asDiagonal(),     // Init gyro bias covariance
      Vector3d::Zero(),                                                     // Init mag hard bias
      Vector3d::Constant(math::sqr(initMagHardBiasStddev())).asDiagonal(),  // Init mag hard bias covariance
      Matrix3d::Identity(),                                                 // Init mag soft bias
      Vector6d::Constant(math::sqr(initMagSoftBiasStddev())).asDiagonal(),  // Init mag soft bias covariance
      0.0,                                                                  // Init barometer altitude bias
      math::sqr(initBaroAltBiasStddev()),                                   // Init barometer altitude bias variance
      st::kGravity,                                                         // Init gravity
      math::sqr(initGravBiasStddev()),                                      // Init gravity variance
      cur_time);
    imu_raw_ = msg;
    return;
  }

  // Update IMU message
  imu_raw_ = msg;

  // Measure IMU
  const auto& acc_meas = msg->accel.data;
  const auto& gyro_meas = msg->gyro.data;
  const auto grav_cov = adaptive_grav_noise_ ? calcGravMeasNoiseCov(acc_meas) : fixed_grav_cov_;
  eskf_.measureIMU(acc_meas, gyro_meas, fixed_acc_cov_, fixed_gyro_cov_, grav_cov, cur_time);

  // Do not publish any messages if filtered IMU message is not ready
  if (!imu_filt_) {
    return;
  }

  // Create odometry message
  auto odom = std::make_unique<tobas_msgs::OdometryWithCovarianceStamped>();
  fillOdometryMsg(*odom);

  // Create TF message
  tf_.header.stamp = odom->header.stamp;
  kdl::transformKDLToMsg(odom->odom.odom.frame, tf_.transform);

  // Publish odometry and TF
  odom_pub_->publish(std::move(odom));
  tf_br_.sendTransform(tf_);

  // Publish feedback
  publishFeedback(msg->header);
}

void ErrorStateKalmanFilterNode::imuFiltCb(const tobas_msgs::Imu::ConstSharedPtr& msg)
{
  imu_filt_ = msg;
}

void ErrorStateKalmanFilterNode::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg)
{
  if (!imu_raw_) {
    return;
  }

  mag_ = msg;

  // If GNSS has not been received when the first geomagnetic data arrives,
  // temporarily use the first geomagnetic vector as the reference.
  if (!mag_ref_set_) {
    // Wait until the attitude stabilizes.
    const auto rot_cov = eskf_.getRotationCovariance();
    const auto atti_var = (rot_cov(0, 0) + rot_cov(1, 1)) / 2;
    const auto atti_stddev = std::sqrt(atti_var);  // [rad]
    if (atti_stddev > kAccurateAttitudeStddevThresh) {
      TOBAS_INFO_THROTTLE(kTypicalInfoPeriod, "Waiting for attitude estimation to converge.");
      return;
    }

    // Wait until filtered IMU data is available.
    if (!imu_filt_) {
      TOBAS_INFO_THROTTLE(kTypicalInfoPeriod, "Waiting for the filtered IMU messages.");
      return;
    }

    // Restart from the beginning if motion is detected.
    if (imu_filt_->gyro.norm() > kStaticGyroThresh) {
      TOBAS_WARN_THROTTLE(
        kTypicalWarnPeriod, "Motion was detected while measuring the reference magnetic field. Retrying...");
      init_mag_cnt_ = 0;
      for (auto& sum : init_mag_sum_) {
        sum.reset();
      }
      return;
    }

    // Correct the attitude and compute the geomagnetic vector as seen from the ground coordinate system.
    const auto R_W_B = eskf_.getQuaternion();
    const auto [roll, pitch, _] = st::eulerFromQuaternion(R_W_B.x(), R_W_B.y(), R_W_B.z(), R_W_B.w());
    const auto R_G_B = kdl::Rotation::RPY(roll, pitch, 0.0);
    const auto mag_G = R_G_B * msg->mag;

    // Accumulate the geomagnetic vector.
    for (size_t i = 0; i < 3; ++i) {
      init_mag_sum_[i].add(mag_G(i));
    }

    // Set the mean value as the reference vector after enough geomagnetic data has been accumulated.
    if (++init_mag_cnt_ >= kInitMagCount) {
      Eigen::Vector3d init_mag_mean;
      for (size_t i = 0; i < 3; ++i) {
        init_mag_mean(i) = init_mag_sum_[i].get() / init_mag_cnt_;
      }
      setMagneticFieldRef(init_mag_mean);
    }

    return;
  }

  const auto& mag_meas = msg->mag.data;
  const auto stamp = ros2::chronoFromRosTime(msg->header.stamp);

  // Update all three axes when estimating bias; otherwise update yaw only.
  if (do_mag_hard_bias_estimation_ || do_mag_soft_bias_estimation_) {
    eskf_.measureMagneticField3d(mag_meas, fixed_mag_cov_, stamp);
  }
  else {
    eskf_.measureMagneticFieldHead(mag_meas, fixed_head_var_, stamp);
  }
}

void ErrorStateKalmanFilterNode::baroCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& msg)
{
  if (!imu_raw_) {
    return;
  }

  // Set the initial barometric altitude value.
  if (!baro_alt_origin_set_) {
    init_pres_sum_.add(msg->pressure);
    if (++init_pres_cnt_ >= kInitBaroCount) {
      const auto init_pres_mean = init_pres_sum_.get() / init_pres_cnt_;
      eskf_.setAirPressureOrigin(init_pres_mean);
      baro_alt_origin_set_ = true;
      TOBAS_INFO("The pressure at zero altitude has been set to ", init_pres_mean * 1e-2, " hPa.");
    }
    return;
  }

  const auto stamp = ros2::chronoFromRosTime(msg->header.stamp);
  eskf_.measureAirPressure(msg->pressure, fixed_baro_alt_var_, stamp);
}

void ErrorStateKalmanFilterNode::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& msg)
{
  if (!imu_raw_ || !imu_filt_) {
    return;
  }

  gnss_fix_ = (msg->fix_type == tobas_msgs::msg::Gnss::FIX_3D);
  if (!gnss_fix_) {
    return;
  }

  const auto& vel_meas = msg->ground_speed.data;
  const auto& pos_cov = adaptive_gnss_noise_ ? msg->position_covariance : fixed_gnss_pos_cov_;
  const auto& vel_cov = adaptive_gnss_noise_ ? msg->velocity_covariance : fixed_gnss_vel_cov_;

  if (!gnss_) {
    // Skip initialization because initializing the GNSS position during flight is dangerous.
    if (arming_ && arming_->data) {
      TOBAS_WARN_THROTTLE(
        kTypicalWarnPeriod,
        "The first GNSS coordinates have been obtained, "
        "but the position and heading cannot be initialized because the vehicle is armed.");
      return;
    }

    // Initial GNSS position.
    // TODO: Convert to the body frame.
    gnss_origin_.latitude = msg->latitude;
    gnss_origin_.longitude = msg->longitude;
    gnss_origin_.altitude = msg->altitude;

    // Publish the initial GNSS position.
    publishGnssOrigin(gnss_origin_.latitude, gnss_origin_.longitude, gnss_origin_.altitude);

    // Compute the geomagnetic reference value from the initial GNSS value.
    // TODO: Compute the reference value online according to position changes.
    const auto mag = geomag::elementsFromGeodetic(
      gnss_origin_.latitude, gnss_origin_.longitude, gnss_origin_.altitude, tim::yearFraction());
    const Vector3d mag_W(mag.east, mag.north, -mag.down);  // ENU coordinates
    setMagneticFieldRef(mag_W);

    // Initialize with the position and velocity from the first received GNSS data.
    // Otherwise, excessive feedback may be applied to attitude.
    eskf_.initializePosition(Vector3d::Zero(), pos_cov);
    eskf_.initializeVelocity(vel_meas, vel_cov);
  }

  gnss_ = msg;

  // Position observation.
  std::tie(pos_meas_.x(), pos_meas_.y()) =
    st::gnssToCartRelative(msg->latitude, msg->longitude, gnss_origin_.latitude, gnss_origin_.longitude);
  pos_meas_.z() = msg->altitude - gnss_origin_.altitude;

  // Covariance.
  gnss_cov_.topLeftCorner<3, 3>() = pos_cov;
  gnss_cov_.bottomRightCorner<3, 3>() = vel_cov;

  // Update ESKF.
  const Vector3d imu2gnss = gnss_offset_ - imu_offset_;
  const auto& gyro_meas = imu_filt_->gyro.data;
  const auto stamp = ros2::chronoFromRosTime(msg->header.stamp);
  gnss_anomaly_score_ = eskf_.measurePosVel(pos_meas_, vel_meas, gnss_cov_, imu2gnss, gyro_meas, stamp);
}

void ErrorStateKalmanFilterNode::externalPoseCb(const tobas_kdl_msgs::FrameWithCovarianceStamped::ConstSharedPtr& msg)
{
  if (!imu_raw_ || !imu_filt_) {
    return;
  }

  const auto& pose = msg->frame.frame;

  Quaterniond quat;
  pose.M.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());

  const auto stamp = ros2::chronoFromRosTime(msg->header.stamp);
  eskf_.measurePose(pose.p.data, quat, msg->frame.covariance, Vector3d::Zero(), stamp);  // TODO: Specify offset.
}

void ErrorStateKalmanFilterNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& msg)
{
  arming_ = msg;
}

void ErrorStateKalmanFilterNode::getGnssOriginCb(
  const GetOriginSrv::Request::ConstSharedPtr&,
  const GetOriginSrv::Response::SharedPtr& res)
{
  if (!gnss_fix_) {
    res->success = false;
    res->message = "GNSS position is not fixed.";
    return;
  }

  res->latitude = gnss_origin_.latitude;
  res->longitude = gnss_origin_.longitude;
  res->altitude = gnss_origin_.altitude;

  res->success = true;
  res->message.clear();
}

void ErrorStateKalmanFilterNode::setGnssOriginCb(
  const SetOriginSrv::Request::ConstSharedPtr& req,
  const SetOriginSrv::Response::SharedPtr& res)
{
  if (!gnss_fix_) {
    res->success = false;
    res->message = "GNSS position is not fixed.";
    return;
  }

  gnss_origin_.latitude = req->latitude;
  gnss_origin_.longitude = req->longitude;

  publishGnssOrigin(gnss_origin_.latitude, gnss_origin_.longitude, gnss_origin_.altitude);

  res->success = true;
  res->message.clear();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::ErrorStateKalmanFilterNode)
