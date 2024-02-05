#include <actionlib/client/simple_action_client.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/exception.hpp>
#include <tobas_std_tools/debug.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/iostream.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_ros_tools/eigen_conversion.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/PreArmCheckAction.h>
#include <tobas_msgs/conversions/msg_msg.hpp>

#include "../include/state_estimation_eskf/eskf_ros.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace et = eigen_tools;

namespace state_estimation_eskf
{
ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    check_topics_timer_(nh_, tobas::kCheckTopicsTimerPeriod, &self::checkTopicsTimerCb, this),
    server_(pnh_)  // NodeletのときはPrivate NodeHandleを明示的に渡す必要がある
{
  TOBAS_DEBUG("ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos");

  getRosParams();
  drone_.loadFromParam(nh_);

  // Fill the static part of the transform message
  tf_.header.frame_id = tobas::kWorldFrame;
  tf_.child_frame_id = drone_.tree().getRootName();

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigureの設定．この時点で1度コールバックが呼ばれる．
  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ErrorStateKalmanFilterRos::getRosParams()
{
  tobas_ros::getParam(pnh_, "use_barometer", use_bar_, kDefaultUseBarometer);
  tobas_ros::getParam(pnh_, "use_gps", use_gps_, kDefaultUseGps);
  tobas_ros::getParam(
    pnh_, "do_acc_bias_estimation", do_acc_bias_estimation_, kDefaultDoAccBiasEstimation);
  tobas_ros::getParam(
    pnh_, "do_gyro_bias_estimation", do_gyro_bias_estimation_, kDefaultDoGyroBiasEstimation);
  tobas_ros::getParam(pnh_, "do_gravity_estimation", do_grav_estimation_, kDefaultDoGravEstimation);
  tobas_ros::getParam(
    pnh_, "check_covariance_convergence", check_covariance_convergence_,
    kDefaultCheckCovarianceConvergence);
  tobas_ros::getParam(pnh_, "imu_offset", imu_offset_, Vector3d::Zero());
  tobas_ros::getParam(pnh_, "barometer_offset", bar_offset_, Vector3d::Zero());
  tobas_ros::getParam(pnh_, "gps_offset", gps_offset_, Vector3d::Zero());

  // 加速度バイアスのZ成分と重力加速度の分離は困難だと思われるため，どちらか一方のみを許容
  if (do_acc_bias_estimation_ && do_grav_estimation_)
    ROS_THROW_NAMED(
      name_, "You cannot enable both accelerometer bias estimation and gravity estimation.");
}

void ErrorStateKalmanFilterRos::registerPublishers()
{
  TOBAS_DEBUG("ErrorStateKalmanFilterRos::registerPublishers");

  odom_pub_ = nh_.advertise<OdomMsg>(tobas::kOdometryTopic, 1);
  feedback_pub_ = nh_.advertise<FeedbackMsg>("eskf_feedback", 1);
}

void ErrorStateKalmanFilterRos::registerSubscribers()
{
  TOBAS_DEBUG("ErrorStateKalmanFilterRos::registerSubscribers");

  imu_sub_ = nh_.subscribe(tobas::kImuTopic, 1, &self::imuCb, this, tcpNoDelay());
  mag_sub_ = nh_.subscribe(tobas::kMagTopic, 1, &self::magCb, this, tcpNoDelay());

  if (use_bar_)
    bar_sub_ = nh_.subscribe(tobas::kAirPressureTopic, 1, &self::barCb, this, tcpNoDelay());

  if (use_gps_)
    gps_sub_ = nh_.subscribe(tobas::kGpsTopic, 1, &self::gpsCb, this, tcpNoDelay());
}

bool ErrorStateKalmanFilterRos::isReady() const
{
  if (!imu_received_)
    return false;
  if (!mag_received_)
    return false;
  if (use_bar_ && !bar_received_)
    return false;
  if (use_gps_ && !gps_received_)
    return false;

  return true;
}

void ErrorStateKalmanFilterRos::initialize()
{
  TOBAS_DEBUG("ErrorStateKalmanFilterRos::initialize");

  // 静止状態でのセンサデータを平均してゼロ点を決める
  setZeroPositions();

  // GPSの初期値から地磁気の参照値を求める
  // TODO: 位置の変化に合わせてオンラインで参照値を求める
  const auto mag = tobas::geomag(lat_0_, lon_0_, alt_0_gps_);
  cout << "The magnetic field of the initial point:" << endl;
  cout << "North: " << mag.north << ", East: " << mag.east << ", Down: " << mag.down << endl;

  // ESKFを初期化
  // TODO: IMUのバイアスの共分散の初期値をちゃんと設定
  const double init_acc_bias_stddev = do_acc_bias_estimation_ ? kInitAccBiasStddev : 0;
  const double init_gyro_bias_stddev = do_gyro_bias_estimation_ ? kInitGyroBiasStddev : 0;
  const double init_grav_stddev = do_grav_estimation_ ? kInitGravStddev : 0;
  eskf_.initialize(
    Vector3d(mag.north, -mag.east, -mag.down),                    // Magnetic field (NWU)
    Vector3d::Zero(),                                             // Init position
    Vector3d::Zero(),                                             // Init velocity
    q_0_,                                                         // Init quaternion
    Vector3d::Constant(sqr(kInitPosStddev)).asDiagonal(),         // Init position cov
    Vector3d::Constant(sqr(kInitVelStddev)).asDiagonal(),         // Init velocity cov
    Vector3d::Constant(sqr(kInitRotStddev)).asDiagonal(),         // Init rotation cov
    Vector3d::Constant(sqr(init_acc_bias_stddev)).asDiagonal(),   // Init accel bias cov
    Vector3d::Constant(sqr(init_gyro_bias_stddev)).asDiagonal(),  // Init gyro bias cov
    sqr(init_grav_stddev)                                         // Init gravity var
  );

  // ヨー角の初期値
  double roll, pitch;
  quaternionToEuler(q_0_.x(), q_0_.y(), q_0_.z(), q_0_.w(), roll, pitch, yaw_prev_);

  yaw_jump_count_ = 0;
}

void ErrorStateKalmanFilterRos::setZeroPositions()
{
  TOBAS_DEBUG("ErrorStateKalmanFilterRos::setZeroPositions");

  actionlib::SimpleActionClient<tobas_msgs::PreArmCheckAction> ac(tobas::kPreArmCheckAction);
  rosInfo(name_, "Waiting for action server '" << tobas::kPreArmCheckAction << "' to start.");
  ac.waitForServer();

  rosInfo(name_, "Action server '" << tobas::kPreArmCheckAction << "' started, sending goal.");
  tobas_msgs::PreArmCheckGoal goal;
  ac.sendGoal(goal);

  const bool finished_before_timeout = ac.waitForResult();
  if (!finished_before_timeout)
  {
    rosFatal(name_, "'" << tobas::kPreArmCheckAction << "' did not finish before timeout.");
    nh_.shutdown();
  }

  const auto result = ac.getResult();
  const auto state = ac.getState();
  if (result->error_code != tobas_msgs::PreArmCheckResult::NO_ERROR)
  {
    rosFatal(
      name_, "'" << tobas::kPreArmCheckAction << "' finished with error: " << state.getText());
    nh_.shutdown();
  }

  // GPS
  // TODO: IMUフレームに変換
  lat_0_ = result->gps.latitude;
  lon_0_ = result->gps.longitude;
  alt_0_gps_ = result->gps.altitude;

  // Barometer
  // TODO: IMUフレームに変換
  alt_0_bar_ = pressureToAltitude(result->air_pressure.fluid_pressure);

  // 初期姿勢
  tobas_ros::vectorMsgToEigen(result->imu.linear_acceleration, acc_meas_);
  tobas_ros::vectorMsgToEigen(result->magnetic_field.magnetic_field, mag_meas_);
  const auto mag = tobas::geomag(result->gps.latitude, result->gps.longitude, result->gps.altitude);
  const Vector3d m0(mag.north, -mag.east, -mag.down);  // NED -> NWU
  et::imuToQuaternion(acc_meas_, mag_meas_, m0, q_0_);
}

ErrorStateKalmanFilterRos::OdomMsg::ConstPtr
ErrorStateKalmanFilterRos::makeOdometryMsg(const ImuMsg& imu)
{
  const Vector3d W_Pos_WI = eskf_.getPosition();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_grav = B_Rot_W * Vector3d(0, 0, -tobas::kGravity);
  const Vector3d B_Acc = acc_meas_ - eskf_.getAccelBias() + B_grav;  // 重力を除いた加速度
  const Vector3d B_Gyro = gyro_meas_ - eskf_.getGyroBias();

  const auto odom = boost::make_shared<OdomMsg>();

  // Header
  odom->header.stamp = imu.header.stamp;
  odom->header.frame_id = tobas::kWorldFrame;

  // Position (Global): IMU frame -> Base frame
  odom->pose.pos.data = W_Pos_WI - W_Rot_B * imu_offset_;
  tobas_ros::matrix3EigenToMsg(eskf_.getPositionCovariance(), odom->position_covariance);

  // Linear velocity (Local): IMU frame -> Base frame
  odom->twist.vel.data = B_Rot_W * W_Vel_WI - B_Gyro.cross(imu_offset_);
  const Matrix3d vel_cov_B = B_Rot_W * eskf_.getVelocityCovariance() * W_Rot_B;
  tobas_ros::matrix3EigenToMsg(vel_cov_B, odom->linear_velocity_covariance);

  // Roll, Pitch
  auto& rpy = odom->pose.euler;
  quaternionToEuler(
    W_Rot_B.x(), W_Rot_B.y(), W_Rot_B.z(), W_Rot_B.w(), rpy.roll, rpy.pitch, yaw_now_);

  // Yaw
  if (yaw_now_ - yaw_prev_ > M_PI)  // 負方向のジャンプを検出
    --yaw_jump_count_;
  else if (yaw_now_ - yaw_prev_ < -M_PI)  // 正方向のジャンプを検出
    ++yaw_jump_count_;
  yaw_prev_ = yaw_now_;
  rpy.yaw = (2 * M_PI) * yaw_jump_count_ + yaw_now_;

  tobas_ros::matrix3EigenToMsg(eskf_.getOrientationCovariance(), odom->orientation_covariance);

  // Angular velocity (Local)
  odom->twist.rot.data = B_Gyro;
  odom->angular_velocity_covariance = imu.angular_velocity_covariance;

  // Linear acceleration (Local)
  odom->accel.linear.data = B_Acc;
  odom->linear_acceleration_covariance = imu.linear_acceleration_covariance;

  // Angular acceleration (Local)
  odom->accel.angular.fill(nan(tobas::kUnknown));
  odom->angular_acceleration_covariance.fill(nan(tobas::kUnknown));

  return odom;
}

void ErrorStateKalmanFilterRos::imuCb(const ImuMsg::ConstPtr& imu)
{
  // 加速度とジャイロを更新
  // 他のコールバックで使用する場合があるので，この更新だけは先にやっておく
  tobas_ros::vectorMsgToEigen(imu->linear_acceleration, acc_meas_);
  tobas_ros::vectorMsgToEigen(imu->angular_velocity, gyro_meas_);

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
        rosInfo(name_, "All messages are received. Initializing kalman filter.");
        stage_ = SET_FIRST_TIME;
      }
      break;
    }
    case SET_FIRST_TIME:
    {
      t_last_ = imu->header.stamp;
      stage_ = RUNNING;
      break;
    }
    case RUNNING:
    {
      // Compute IMU time gap
      const double dt = (imu->header.stamp - t_last_).toSec();
      // cout << "dt[s] " << dt << endl;
      t_last_ = imu->header.stamp;

      // Check IMU time gap
      if (dt == 0.)
      {
        rosError(name_, "The time gap between 2 IMU messages is 0.");
        break;
      }
      if (dt < 0.)
      {
        rosError(name_, "The time gap between 2 IMU messages is negative: " << dt << " [s]");
        break;
      }
      if (dt > kImuTimeGapThreshold)
      {
        rosWarn(name_, "The time gap between 2 IMU messages is too large: " << dt << " [s]");
      }

      // 観測ノイズの分散を計算
      const auto acc_noise_var = trace(imu->linear_acceleration_covariance) / 3;
      const auto gyro_noise_var = trace(imu->angular_velocity_covariance) / 3;

      // 事前予測
      eskf_.predictIMU(
        acc_meas_, gyro_meas_, acc_noise_var, gyro_noise_var, acc_bias_noise_var_,
        gyro_bias_noise_var_, grav_noise_var_, dt);

      // 重力方向の観測
      eskf_.measureGravity(acc_meas_, grav_cov_);

      // 共分散の収束を確認
      if (check_covariance_convergence_ && !cov_converged_)
      {
        const auto pos_cov = eskf_.getPositionCovariance();
        const auto vel_cov = eskf_.getVelocityCovariance();
        const auto rot_cov = eskf_.getOrientationCovariance();

        const auto hor_pos_var = max(pos_cov(0, 0), pos_cov(1, 1));
        const auto ver_pos_var = pos_cov(2, 2);
        const auto vel_var = vel_cov.diagonal().maxCoeff();
        const auto rot_var = rot_cov.diagonal().maxCoeff();

        bool cov_ok = true;
        if (hor_pos_var > sqr(kHorPosStddevThreshold))
        {
          ROS_INFO_STREAM_THROTTLE(
            kPrintStddevPeriod, "Horizontal Position std. dev [m]: " << sqrt(hor_pos_var) << " > "
                                                                     << kHorPosStddevThreshold);
          cov_ok = false;
        }
        if (ver_pos_var > sqr(kVerPosStddevThreshold))
        {
          ROS_INFO_STREAM_THROTTLE(
            kPrintStddevPeriod, "Vertical Position std. dev [m]: " << sqrt(ver_pos_var) << " > "
                                                                   << kVerPosStddevThreshold);
          cov_ok = false;
        }
        if (vel_var > sqr(kVelStddevThreshold))
        {
          ROS_INFO_STREAM_THROTTLE(
            kPrintStddevPeriod,
            "Velocity std. dev [m/s]: " << sqrt(vel_var) << " > " << kVelStddevThreshold);
          cov_ok = false;
        }
        if (rot_var > sqr(kRotStddevThreshold))
        {
          ROS_INFO_STREAM_THROTTLE(
            kPrintStddevPeriod,
            "Rotation std. dev [rad]: " << sqrt(rot_var) << " > " << kRotStddevThreshold);
          cov_ok = false;
        }

        if (cov_ok)
        {
          cov_converged_ = true;
          rosInfo(name_, "Kalman filter is initialized. Start to publish pose & twist.");
        }

        return;
      }

      // 推定状態を発行
      const auto odom = makeOdometryMsg(*imu);
      odom_pub_.publish(odom);

      // TFを発行
      tf_.header.stamp = odom->header.stamp;
      tobas::transformTobasToMsg(odom->pose, tf_.transform);
      tf_br_.sendTransform(tf_);

      // フィードバックを発行
      const auto feedback = boost::make_shared<FeedbackMsg>();
      feedback->header = imu->header;
      feedback->acc_bias.data = eskf_.getAccelBias();
      feedback->gyro_bias.data = eskf_.getGyroBias();
      feedback->gravity = eskf_.getGravity();
      tobas_ros::matrix3EigenToMsg(eskf_.getAccelBiasCovariance(), feedback->acc_bias_covariance);
      tobas_ros::matrix3EigenToMsg(eskf_.getGyroBiasCovariance(), feedback->gyro_bias_covariance);
      feedback->gravity_variance = eskf_.getGravityVariance();
      feedback->gps_anormaly_score = gps_anormaly_score_;
      feedback_pub_.publish(feedback);

      break;
    }
  }
}

void ErrorStateKalmanFilterRos::magCb(const MagMsg::ConstPtr& mag)
{
  if (!mag_received_)
    mag_received_ = true;

  if (stage_ < RUNNING)
    return;

  eskf_.measureMagneticField(mag->magnetic_field.x, mag->magnetic_field.y, yaw_var_);
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg::ConstPtr& bar)
{
  if (!bar_received_)
    bar_received_ = true;

  if (stage_ < RUNNING)
    return;

  double z_abs, z_var;
  pressureToAltitude(bar->fluid_pressure, bar->variance, z_abs, z_var);

  // TODO: bar_offsetを考慮
  const double z_m = z_abs - alt_0_bar_;
  eskf_.measureAltitude(z_m, z_var);
}

void ErrorStateKalmanFilterRos::gpsCb(const GpsMsg::ConstPtr& gps)
{
  if (!gps_received_)
    gps_received_ = true;

  if (stage_ < RUNNING)
    return;

  // 位置の観測値
  gpsToCartRelative(gps->latitude, gps->longitude, lat_0_, lon_0_, pos_meas_.x(), pos_meas_.y());
  pos_meas_.z() = gps->altitude - alt_0_gps_;

  // 共分散
  const Matrix3d pos_cov = Map<const Matrix3d>(gps->position_covariance.data());
  const Matrix3d vel_cov = Map<const Matrix3d>(gps->velocity_covariance.data());

  // ESKFを更新
  const auto imu2gps = gps_offset_ - imu_offset_;
  gps_anormaly_score_ =
    eskf_.measurePosVel(pos_meas_, pos_cov, gps->ground_speed.data, vel_cov, imu2gps, gyro_meas_);

  // 異常度が高すぎる場合は警告
  if (gps_anormaly_score_ > kAnormalyScoreThreshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The kalman filter is in an abnormal state. There is a very large error between the GPS "
      "position and velocity information and the estimated values from the Kalman filter.");
  }
}

void ErrorStateKalmanFilterRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!imu_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kImuTopic);

  if (!mag_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kMagTopic);

  if (use_bar_ && !bar_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kAirPressureTopic);

  if (use_gps_ && !gps_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kGpsTopic);
}

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  grav_cov_.diagonal().fill(cfg.gravity_variance);
  yaw_var_ = cfg.yaw_variance;
  acc_bias_noise_var_ = do_acc_bias_estimation_ ? exp10(cfg.acc_bias_noise_var_log10) : 0;
  gyro_bias_noise_var_ = do_gyro_bias_estimation_ ? exp10(cfg.gyro_bias_noise_var_log10) : 0;
  grav_noise_var_ = do_grav_estimation_ ? exp10(cfg.gravity_noise_var_log10) : 0;

  rosInfo(name_, "New dynamic parameters are set.");
}
}  // namespace state_estimation_eskf
