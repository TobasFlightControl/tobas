#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_std_tools/exception.hpp>
#include <dh_eigen_tools/geometry.hpp>
#include <dh_eigen_tools/iostream.hpp>
#include <dh_eigen_tools/conversion/eigen_boost.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../include/state_estimation_eskf/eskf_ros.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace state_estimation_eskf
{
ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos(ros::NodeHandle nh, ros::NodeHandle pnh)
  : super(nh, pnh),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &ErrorStateKalmanFilterRos::checkTopicsTimerCb,
      this)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  imu2gps_ = drone_.gpsOffset() - drone_.imuOffset();

  rot_acc_cov_.setZero();
  rot_mag_cov_.setZero();

  dynamicReconfigureCb(cfg_, 0);

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f =
    boost::bind(&ErrorStateKalmanFilterRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ErrorStateKalmanFilterRos::getRosParams()
{
  dh_ros::getParam(pnh_, "gyro_noise_density", gyro_noise_density_, dh_ros::POSITIVE);
  dh_ros::getParam(pnh_, "gyro_random_walk", gyro_random_walk_, dh_ros::POSITIVE);
  dh_ros::getParam(pnh_, "acc_noise_density", acc_noise_density_, dh_ros::POSITIVE);
  dh_ros::getParam(pnh_, "acc_random_walk", acc_random_walk_, dh_ros::POSITIVE);

  dh_ros::getParam(pnh_, "use_barometer", use_bar_, kDefaultUseBarometer);
  dh_ros::getParam(pnh_, "use_gps", use_gps_, kDefaultUseGps);
  dh_ros::getParam(
    pnh_, "gps_horizontal_position_stddev_threshold", gps_hor_pos_stddev_thr_,
    kDefaultGpsHorPosStddevThreshold, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh_, "gps_vertical_position_stddev_threshold", gps_ver_pos_stddev_thr_,
    kDefaultGpsVerPosStddevThreshold, dh_ros::POSITIVE);

  string geomag_observe_method;
  dh_ros::getParam(
    pnh_, "geomag_observe_method", geomag_observe_method, kDefaultGeomagObserveMethod);
  if (geomag_observe_method == "rpy")
  {
    geomag_observe_method_ = GeomagObserveMethod::RPY;
  }
  else if (geomag_observe_method == "yaw_only")
  {
    geomag_observe_method_ = GeomagObserveMethod::YAW_ONLY;
  }
  else
  {
    rosthrow("Invalid geomagnetism observation method: " << geomag_observe_method);
  }

  // Dynamic parameters
  dh_ros::getParam(pnh_, "rotation_variance_grav", cfg_.rotation_variance_grav, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh_, "rotation_variance_geomag", cfg_.rotation_variance_geomag, dh_ros::POSITIVE);
}

void ErrorStateKalmanFilterRos::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>("event", 1);
  posevel_pub_ = nh_.advertise<StateMsg>("base_state", 1);
}

void ErrorStateKalmanFilterRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &ErrorStateKalmanFilterRos::eventCb, this);
  imu_sub_ = nh_.subscribe("imu", 1, &ErrorStateKalmanFilterRos::imuCb, this);
  mag_sub_ = nh_.subscribe("magnetic_field", 1, &ErrorStateKalmanFilterRos::magCb, this);

  if (use_bar_)
  {
    bar_sub_ = nh_.subscribe("air_pressure", 1, &ErrorStateKalmanFilterRos::barCb, this);
  }

  if (use_gps_)
  {
    gps_sub_ = nh_.subscribe("gps", 1, &ErrorStateKalmanFilterRos::gpsCb, this);
    vel_sub_ = nh_.subscribe("ground_speed", 1, &ErrorStateKalmanFilterRos::velCb, this);
  }
}

bool ErrorStateKalmanFilterRos::isReady()
{
  bool ok = true;

  ok &= imu_received_;
  ok &= mag_received_;

  if (use_bar_)
  {
    ok &= bar_received_;
  }

  if (use_gps_)
  {
    ok &= gps_received_;
    ok &= vel_received_;
  }

  return ok;
}

bool ErrorStateKalmanFilterRos::isValidDeltaTime(double dt)
{
  if (dt == 0.)
  {
    rosError("The time gap between 2 IMU messages is 0.");
    return false;
  }

  if (dt < 0.)
  {
    rosError("The time gap between 2 IMU messages is negative: " << dt << " [s]");
    return false;
  }

  if (dt > kImuTimeGapThreshold)
  {
    rosError("The time gap between 2 IMU messages is too large: " << dt << " [s]");
    return false;
  }

  return true;
}

void ErrorStateKalmanFilterRos::initialize()
{
  // 静止状態でのセンサデータを平均してゼロ点を決める
  const auto result = setZeroPositions();

  // GPSの初期値から地磁気の参照値を求める
  // TODO: 位置の変化に合わせてオンラインで参照値を求める
  const auto mag = tobas::geomag(lat_0_, lon_0_, alt_0_gps_);
  cout << "The magnetic field of the initial point:" << endl;
  cout << "North: " << mag.north << ", East: " << mag.east << ", Down: " << mag.down << endl;

  // ISKFを初期化
  // TODO: IMUのバイアスの共分散の初期値をちゃんと設定
  eskf_.initialize(
    acc_noise_density_,                                           // Accelerometer noise density
    gyro_noise_density_,                                          // Gyrometer noise density
    acc_random_walk_,                                             // Accelerometer random walk
    gyro_random_walk_,                                            // Gyrometer random walk
    Vector3d(0., 0., -tobas::kGravity),                           // Gravity vector
    Vector3d(mag.north, -mag.east, -mag.down),                    // Magnetic field (NWU)
    Vector3d::Zero(),                                             // Init position
    Vector3d::Zero(),                                             // Init velocity
    q_0_,                                                         // Init quaternion
    Map<const Matrix3d>(result->gps.position_covariance.data()),  // Init position cov
    Map<const Matrix3d>(result->ground_speed.covariance.data()),  // Init velocity cov
    Vector3d::Constant(kInitRotStddev).asDiagonal(),              // Init quaternion cov
    Matrix3d::Zero(),                                             // Init accel bias cov
    Matrix3d::Zero()                                              // Init gyro bias cov
  );

  // ヨー角の初期値
  double roll, pitch;
  quaternionToEuler(q_0_.x(), q_0_.y(), q_0_.z(), q_0_.w(), roll, pitch, yaw_prev_);

  yaw_jump_count_ = 0;
}

tobas_msgs::StaticStateDeterminationResultConstPtr ErrorStateKalmanFilterRos::setZeroPositions()
{
  constexpr char action_name[] = "static_state_determination";
  actionlib::SimpleActionClient<tobas_msgs::StaticStateDeterminationAction> ac(action_name);
  rosInfo("Waiting for action server '" << action_name << "' to start.");
  ac.waitForServer();

  rosInfo("Action server '" << action_name << "' started, sending goal.");
  tobas_msgs::StaticStateDeterminationGoal goal;
  goal.gps_horizontal_position_stddev_threshold = gps_hor_pos_stddev_thr_;
  goal.gps_vertical_position_stddev_threshold = gps_ver_pos_stddev_thr_;
  ac.sendGoal(goal);

  const bool finished_before_timeout = ac.waitForResult();
  if (!finished_before_timeout)
  {
    rosError("Action did not finish before timeout. Shutting down the system.");
    requestShutdown();
  }

  const auto result = ac.getResult();
  const auto state = ac.getState();
  if (result->error_code != tobas_msgs::StaticStateDeterminationResult::NO_ERROR)
  {
    rosError(
      "'" << action_name << "' finished with error: " << state.getText()
          << " Shutting down the system.");
    requestShutdown();
  }

  rosInfo(
    "The result of " << action_name << ":\n"
                     << "IMU count: " << result->imu_count << endl
                     << "Magnetometer count: " << result->mag_count << endl
                     << "Barometer count: " << result->bar_count << endl
                     << "GPS position count: " << result->gps_count << endl
                     << "GPS velocity count: " << result->vel_count << endl
                     << "IMU:\n"
                     << result->imu << endl
                     << "Magnetic Field:\n"
                     << result->magnetic_field << endl
                     << "Air Pressure:\n"
                     << result->air_pressure << endl
                     << "GPS:\n"
                     << result->gps << endl
                     << "Ground Speed:\n"
                     << result->ground_speed);

  // GPS
  // TODO: IMUフレームに変換
  lat_0_ = result->gps.latitude;
  lon_0_ = result->gps.longitude;
  alt_0_gps_ = result->gps.altitude;

  // Barometer
  // TODO: IMUフレームに変換
  alt_0_bar_ = pressureToAltitude(result->air_pressure.fluid_pressure);

  // 初期姿勢
  tf::vectorMsgToEigen(result->imu.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(result->magnetic_field.magnetic_field, mag_m_);
  const auto mag = tobas::geomag(result->gps.latitude, result->gps.longitude, result->gps.altitude);
  const Vector3d m0(mag.north, -mag.east, -mag.down);  // NED -> NWU
  eigen_tools::imuToQuaternion(a_m_, mag_m_, m0, q_0_);

  return result;
}

void ErrorStateKalmanFilterRos::updatePoseVelMsg(const ImuMsg& imu, StateMsg& state)
{
  const Vector3d W_Pos_WI = eskf_.getXYZ();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_omega = w_m_ - eskf_.getGyroBias();
  const Vector3d B_Pos_BI = drone_.imuOffset();

  // Time stamp
  state.header.stamp = imu.header.stamp;

  // Position (Global): IMU frame -> Base frame
  const Vector3d W_Pos_WB = W_Pos_WI - W_Rot_B * B_Pos_BI;
  tf::vectorEigenToKDL(W_Pos_WB, state.pose.pos);

  // Linear velocity (Local): IMU frame -> Base frame
  const Vector3d B_Vel_WB = B_Rot_W * W_Vel_WI - B_omega.cross(B_Pos_BI);
  tf::vectorEigenToKDL(B_Vel_WB, state.twist.vel);

  // Roll, Pitch
  auto& rpy = state.pose.euler;
  quaternionToEuler(
    W_Rot_B.x(), W_Rot_B.y(), W_Rot_B.z(), W_Rot_B.w(), rpy.roll, rpy.pitch, yaw_now_);

  // Yaw
  if (yaw_now_ - yaw_prev_ > M_PI)  // 負方向のジャンプを検出
  {
    --yaw_jump_count_;
  }
  else if (yaw_now_ - yaw_prev_ < -M_PI)  // 正方向のジャンプを検出
  {
    ++yaw_jump_count_;
  }
  yaw_prev_ = yaw_now_;
  rpy.yaw = (2 * M_PI) * yaw_jump_count_ + yaw_now_;

  // Angular velocity (Local)
  tf::vectorEigenToKDL(B_omega, state.twist.rot);

  // Covariances
  eigen_tools::matrix3EigenToBoost(eskf_.getPositionCovariance(), state.position_covariance);
  eigen_tools::matrix3EigenToBoost(eskf_.getOrientationCovariance(), state.orientation_covariance);
  eigen_tools::matrix3EigenToBoost(eskf_.getVelocityCovariance(), state.linear_velocity_covariance);
  state.angular_velocity_covariance = imu.angular_velocity_covariance;  // ジャイロはそのまま

  // For debug
  // cout << "Estiamted Quaternion:" << endl << W_Rot_B << endl;
  // cout << "Estimated accelerometer bias:" << endl << eskf_.getAccelBias() << endl;
  // cout << "Estimated gyroscope bias:" << endl << eskf_.getGyroBias() << endl;
}

void ErrorStateKalmanFilterRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void ErrorStateKalmanFilterRos::imuCb(const ImuMsg::ConstPtr& imu)
{
  switch (stage_)
  {
    case FIRST_IMU:
    {
      imu_received_ = true;
      stage_ = WAIT_TOPICS;
      break;
    }
    case WAIT_TOPICS:
    {
      if (isReady())
      {
        check_topics_timer_.stop();
        initialize();
        rosInfo("All messages are received. Wait to publish for " << kWaitToPublish << " seconds.");
        stage_ = SET_FIRST_TIME;
      }
      break;
    }
    case SET_FIRST_TIME:
    {
      t_ready_ = t_last_ = imu->header.stamp;
      stage_ = RUNNING;
      break;
    }
    case RUNNING:
    {
      const double dt = (imu->header.stamp - t_last_).toSec();
      t_last_ = imu->header.stamp;
      if (!isValidDeltaTime(dt))
      {
        return;
      }

      tf::vectorMsgToEigen(imu->linear_acceleration, a_m_);
      tf::vectorMsgToEigen(imu->angular_velocity, w_m_);

      // 事前予測
      eskf_.predictIMU(a_m_, w_m_, dt);

      // 重力方向の観測
      eskf_.measureAcceleration(a_m_, rot_acc_cov_);

      // 推定状態を発行
      if ((ros::Time::now() - t_ready_).toSec() > kWaitToPublish)
      {
        const auto state = boost::make_shared<StateMsg>();
        updatePoseVelMsg(*imu, *state);
        posevel_pub_.publish(state);
      }

      break;
    }
  }
}

void ErrorStateKalmanFilterRos::magCb(const MagMsg::ConstPtr& mag)
{
  if (!mag_received_)
  {
    mag_received_ = true;
  }

  if (stage_ < RUNNING)
  {
    return;
  }

  tf::vectorMsgToEigen(mag->magnetic_field, mag_m_);

  switch (geomag_observe_method_)
  {
    case GeomagObserveMethod::RPY:
      eskf_.measureMagneticFieldRPY(mag_m_, rot_mag_cov_);
      break;
    case GeomagObserveMethod::YAW_ONLY:
      eskf_.measureMagneticFieldYaw(mag_m_.x(), mag_m_.y(), rot_mag_cov_(0, 0));
      break;
    default:
      throw NotImplementedError();
  }
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg::ConstPtr& bar)
{
  if (!bar_received_)
  {
    bar_received_ = true;
  }

  if (stage_ < RUNNING)
  {
    return;
  }

  double z_abs, z_var;
  pressureToAltitude(bar->fluid_pressure, bar->variance, z_abs, z_var);

  const double z_m = z_abs - alt_0_bar_;
  eskf_.measureAltitude(z_m, z_var);
}

void ErrorStateKalmanFilterRos::gpsCb(const GpsMsg::ConstPtr& gps)
{
  if (!gps_received_)
  {
    gps_received_ = true;
  }

  if (stage_ < RUNNING)
  {
    return;
  }

  gpsToCartRelative(gps->latitude, gps->longitude, lat_0_, lon_0_, pos_m_.x(), pos_m_.y());
  pos_m_.z() = gps->altitude - alt_0_gps_;
  const Matrix3d cov = Map<const Matrix3d>(gps->position_covariance.data());

  eskf_.measureXYZ(pos_m_, cov, imu2gps_);

  // トピック通信の遅延チェック
  // const auto delay = (ros::Time::now() - gps->header.stamp).toSec();
  // rosInfo("NavSatFix communication delay: " << delay << "[s]");
}

void ErrorStateKalmanFilterRos::velCb(const VelMsg::ConstPtr& vel)
{
  if (!vel_received_)
  {
    vel_received_ = true;
  }

  if (stage_ < RUNNING)
  {
    return;
  }

  tf::vectorKDLToEigen(vel->vel, vel_m_);
  const Matrix3d cov = Map<const Matrix3d>(vel->covariance.data());

  eskf_.measureVelocity(vel_m_, cov, w_m_, imu2gps_);
}

void ErrorStateKalmanFilterRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!imu_received_)
  {
    rosWarn("IMU data is not received yet.");
  }

  if (!mag_received_)
  {
    rosWarn("Magnetometer data is not received yet.");
  }

  if (use_bar_ && !bar_received_)
  {
    rosWarn("Barometer data is not received yet.");
  }

  if (use_gps_)
  {
    if (!gps_received_)
    {
      rosWarn("GPS position data is not received yet.");
    }

    if (!vel_received_)
    {
      rosWarn("GPS velocity data is not received yet.");
    }
  }
}

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  rot_acc_cov_.diagonal().fill(cfg.rotation_variance_grav);
  rot_mag_cov_.diagonal().fill(cfg.rotation_variance_geomag);

  rosInfo("New dynamic parameters are set.");
}
}  // namespace state_estimation_eskf
