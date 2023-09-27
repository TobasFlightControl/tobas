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
#include <tobas_msgs/StaticStateDeterminationAction.h>
#include <tobas_msgs/conversions/msg_msg.hpp>

#include "../include/state_estimation_eskf/eskf_ros.hpp"

#define INF numeric_limits<double>::max()

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace et = eigen_tools;

namespace state_estimation_eskf
{
ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos(
  ros::NodeHandle nh,
  ros::NodeHandle pnh,
  string name)
  : super(nh, pnh, name),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &ErrorStateKalmanFilterRos::checkTopicsTimerCb,
      this),
    server_(pnh_)  // NodeletのときはPrivate NodeHandleを明示的に渡す必要がある
{
  getRosParams();
  drone_.loadFromParam(nh_);

  imu2gps_ = drone_.gpsOffset() - drone_.imuOffset();

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigureの設定．この時点で1度コールバックが呼ばれる．
  ConfigServer::CallbackType f =
    boost::bind(&ErrorStateKalmanFilterRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ErrorStateKalmanFilterRos::getRosParams()
{
  dh_ros::getParam(pnh_, "use_barometer", use_bar_, kDefaultUseBarometer);
  dh_ros::getParam(pnh_, "use_gps", use_gps_, kDefaultUseGps);
  dh_ros::getParam(
    pnh_, "gps_horizontal_position_stddev_threshold", gps_hor_pos_stddev_thr_,
    kDefaultGpsHorPosStddevThreshold, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh_, "gps_vertical_position_stddev_threshold", gps_ver_pos_stddev_thr_,
    kDefaultGpsVerPosStddevThreshold, dh_ros::POSITIVE);
}

void ErrorStateKalmanFilterRos::registerPublishers()
{
  pt_pub_ = nh_.advertise<StateMsg>("pose_twist", 1);
  odom_pub_ = nh_.advertise<OdomMsg>("odom", 1);
  feedback_pub_ = nh_.advertise<FeedbackMsg>("eskf_feedback", 1);
}

void ErrorStateKalmanFilterRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &ErrorStateKalmanFilterRos::eventCb, this, tcpNoDelay());
  imu_sub_ = nh_.subscribe("imu", 1, &ErrorStateKalmanFilterRos::imuCb, this, tcpNoDelay());
  mag_sub_ =
    nh_.subscribe("magnetic_field", 1, &ErrorStateKalmanFilterRos::magCb, this, tcpNoDelay());

  if (use_bar_)
  {
    bar_sub_ =
      nh_.subscribe("air_pressure", 1, &ErrorStateKalmanFilterRos::barCb, this, tcpNoDelay());
  }

  if (use_gps_)
  {
    gps_sub_ = nh_.subscribe("gps", 1, &ErrorStateKalmanFilterRos::gpsCb, this, tcpNoDelay());
    vel_sub_ =
      nh_.subscribe("ground_speed", 1, &ErrorStateKalmanFilterRos::velCb, this, tcpNoDelay());
  }
}

bool ErrorStateKalmanFilterRos::isReady() const
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

void ErrorStateKalmanFilterRos::initialize()
{
  // 静止状態でのセンサデータを平均してゼロ点を決める
  setZeroPositions();

  // GPSの初期値から地磁気の参照値を求める
  // TODO: 位置の変化に合わせてオンラインで参照値を求める
  const auto mag = tobas::geomag(lat_0_, lon_0_, alt_0_gps_);
  cout << "The magnetic field of the initial point:" << endl;
  cout << "North: " << mag.north << ", East: " << mag.east << ", Down: " << mag.down << endl;

  // ESKFを初期化
  // TODO: IMUのバイアスの共分散の初期値をちゃんと設定
  eskf_.initialize(
    Vector3d(0., 0., -tobas::kGravity),                        // Gravity vector
    Vector3d(mag.north, -mag.east, -mag.down),                 // Magnetic field (NWU)
    Vector3d::Zero(),                                          // Init position
    Vector3d::Zero(),                                          // Init velocity
    q_0_,                                                      // Init quaternion
    Vector3d::Constant(sqr(kInitPosStddev)).asDiagonal(),      // Init position cov
    Vector3d::Constant(sqr(kInitVelStddev)).asDiagonal(),      // Init velocity cov
    Vector3d::Constant(sqr(kInitRotStddev)).asDiagonal(),      // Init rotation cov
    Vector3d::Constant(sqr(kInitAccBiasStddev)).asDiagonal(),  // Init accel bias cov
    Vector3d::Constant(sqr(kInitGyroBiasStddev)).asDiagonal()  // Init gyro bias cov
  );

  // ヨー角の初期値
  double roll, pitch;
  quaternionToEuler(q_0_.x(), q_0_.y(), q_0_.z(), q_0_.w(), roll, pitch, yaw_prev_);

  yaw_jump_count_ = 0;
}

void ErrorStateKalmanFilterRos::setZeroPositions()
{
  constexpr char action_name[] = "static_state_determination";
  actionlib::SimpleActionClient<tobas_msgs::StaticStateDeterminationAction> ac(action_name);
  rosInfo(name_, "Waiting for action server '" << action_name << "' to start.");
  ac.waitForServer();

  rosInfo(name_, "Action server '" << action_name << "' started, sending goal.");
  tobas_msgs::StaticStateDeterminationGoal goal;
  goal.gps_horizontal_position_stddev_threshold = gps_hor_pos_stddev_thr_;
  goal.gps_vertical_position_stddev_threshold = gps_ver_pos_stddev_thr_;
  ac.sendGoal(goal);

  const bool finished_before_timeout = ac.waitForResult();
  if (!finished_before_timeout)
  {
    rosthrow(name_, "'" << action_name << "' did not finish before timeout.");
  }

  const auto result = ac.getResult();
  const auto state = ac.getState();
  if (result->error_code != tobas_msgs::StaticStateDeterminationResult::NO_ERROR)
  {
    rosthrow(name_, "'" << action_name << "' finished with error: " << state.getText());
  }

  // rosInfo(
  //   name_, "The result of " << action_name << ":\n"
  //                           << "IMU count: " << result->imu_count << endl
  //                           << "Magnetometer count: " << result->mag_count << endl
  //                           << "Barometer count: " << result->bar_count << endl
  //                           << "GPS position count: " << result->gps_count << endl
  //                           << "GPS velocity count: " << result->vel_count << endl
  //                           << "IMU:\n"
  //                           << result->imu << endl
  //                           << "Magnetic Field:\n"
  //                           << result->magnetic_field << endl
  //                           << "Air Pressure:\n"
  //                           << result->air_pressure << endl
  //                           << "GPS:\n"
  //                           << result->gps << endl
  //                           << "Ground Speed:\n"
  //                           << result->ground_speed);

  // GPS
  // TODO: IMUフレームに変換
  lat_0_ = result->gps.latitude;
  lon_0_ = result->gps.longitude;
  alt_0_gps_ = result->gps.altitude;

  // Barometer
  // TODO: IMUフレームに変換
  alt_0_bar_ = pressureToAltitude(result->air_pressure.fluid_pressure);

  // 初期姿勢
  tf::vectorMsgToEigen(result->imu.linear_acceleration, acc_meas_);
  tf::vectorMsgToEigen(result->magnetic_field.magnetic_field, mag_meas_);
  const auto mag = tobas::geomag(result->gps.latitude, result->gps.longitude, result->gps.altitude);
  const Vector3d m0(mag.north, -mag.east, -mag.down);  // NED -> NWU
  et::imuToQuaternion(acc_meas_, mag_meas_, m0, q_0_);
}

ErrorStateKalmanFilterRos::StateMsg::ConstPtr
ErrorStateKalmanFilterRos::makePoseVelMsg(const ImuMsg& imu)
{
  const Vector3d W_Pos_WI = eskf_.getXYZ();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_grav = B_Rot_W * Vector3d(0, 0, -tobas::kGravity);
  const Vector3d B_Acc = acc_meas_ - eskf_.getAccelBias() + B_grav;  // 重力を除いた加速度
  const Vector3d B_Gyro = gyro_meas_ - eskf_.getGyroBias();
  const Vector3d B_Pos_BI = drone_.imuOffset();

  auto state = boost::make_shared<StateMsg>();

  // Time stamp
  state->header.stamp = imu.header.stamp;

  // Position (Global): IMU frame -> Base frame
  const Vector3d W_Pos_WB = W_Pos_WI - W_Rot_B * B_Pos_BI;
  tf::vectorEigenToKDL(W_Pos_WB, state->pose.pos);

  // Linear velocity (Local): IMU frame -> Base frame
  const Vector3d B_Vel_WB = B_Rot_W * W_Vel_WI - B_Gyro.cross(B_Pos_BI);
  tf::vectorEigenToKDL(B_Vel_WB, state->twist.vel);

  // Roll, Pitch
  auto& rpy = state->pose.euler;
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
  tf::vectorEigenToKDL(B_Gyro, state->twist.rot);

  // Linear acceleration (Local)
  tf::vectorEigenToKDL(B_Acc, state->accel.linear);

  // Angular acceleration (Local)
  state->accel.angular = KDL::Vector(INF, INF, INF);  // Unknown

  // Covariances
  et::matrix3EigenToBoost(eskf_.getPositionCovariance(), state->position_covariance);
  et::matrix3EigenToBoost(eskf_.getOrientationCovariance(), state->orientation_covariance);
  et::matrix3EigenToBoost(eskf_.getVelocityCovariance(), state->linear_velocity_covariance);
  state->angular_velocity_covariance = imu.angular_velocity_covariance;
  state->linear_acceleration_covariance = imu.linear_acceleration_covariance;
  state->angular_acceleration_covariance.fill(-1);  // Unknown

  return state;
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

      // Convert ROS messages to Eigen vectors
      tf::vectorMsgToEigen(imu->linear_acceleration, acc_meas_);
      tf::vectorMsgToEigen(imu->angular_velocity, gyro_meas_);

      // 観測ノイズの分散を計算
      const auto acc_noise_var = trace(imu->linear_acceleration_covariance) / 3;
      const auto gyro_noise_var = trace(imu->angular_velocity_covariance) / 3;

      // 事前予測 (KFにはIMUの生データを渡す)
      eskf_.predictIMU(
        acc_meas_, gyro_meas_, acc_noise_var, gyro_noise_var, acc_bias_noise_var_,
        gyro_bias_noise_var_, dt);

      // 重力方向の観測
      eskf_.measureAcceleration(acc_meas_, grav_cov_);

      // 共分散の収束を確認
      if (!is_initialized_)
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
          is_initialized_ = true;
          rosInfo(name_, "Kalman filter is initialized. Start to publish pose & twist.");
        }

        return;
      }

      // 推定状態を発行
      const auto state = makePoseVelMsg(*imu);
      pt_pub_.publish(state);

      // 外部用にオドメトリを発行
      const auto odom = boost::make_shared<OdomMsg>();
      tobas::odometryTobasToMsg(*state, *odom);
      odom_pub_.publish(odom);

      // フィードバックを発行
      const auto feedback = boost::make_shared<FeedbackMsg>();
      feedback->header = imu->header;
      tf::vectorEigenToKDL(eskf_.getAccelBias(), feedback->acc_bias);
      tf::vectorEigenToKDL(eskf_.getGyroBias(), feedback->gyro_bias);
      et::matrix3EigenToBoost(eskf_.getAccelBiasCovariance(), feedback->acc_bias_covariance);
      et::matrix3EigenToBoost(eskf_.getGyroBiasCovariance(), feedback->gyro_bias_covariance);
      feedback_pub_.publish(feedback);

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

  eskf_.measureMagneticField(mag->magnetic_field.x, mag->magnetic_field.y, yaw_var_);
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

  gpsToCartRelative(gps->latitude, gps->longitude, lat_0_, lon_0_, pos_meas_.x(), pos_meas_.y());
  pos_meas_.z() = gps->altitude - alt_0_gps_;
  const Matrix3d cov = Map<const Matrix3d>(gps->position_covariance.data());

  eskf_.measurePosition(pos_meas_, cov, imu2gps_);

  // トピック通信の遅延チェック
  // const auto delay = (ros::Time::now() - gps->header.stamp).toSec();
  // rosInfo(name_, "NavSatFix communication delay: " << delay << "[s]");
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

  tf::vectorKDLToEigen(vel->vel, vel_meas_);
  const Matrix3d cov = Map<const Matrix3d>(vel->covariance.data());

  eskf_.measureVelocity(vel_meas_, cov, gyro_meas_, imu2gps_);
}

void ErrorStateKalmanFilterRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!imu_received_)
  {
    rosWarn(name_, "IMU data is not received yet.");
  }

  if (!mag_received_)
  {
    rosWarn(name_, "Magnetometer data is not received yet.");
  }

  if (use_bar_ && !bar_received_)
  {
    rosWarn(name_, "Barometer data is not received yet.");
  }

  if (use_gps_)
  {
    if (!gps_received_)
    {
      rosWarn(name_, "GPS position data is not received yet.");
    }

    if (!vel_received_)
    {
      rosWarn(name_, "GPS velocity data is not received yet.");
    }
  }
}

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  grav_cov_.diagonal().fill(cfg.gravity_variance);
  yaw_var_ = cfg.yaw_variance;
  acc_bias_noise_var_ = exp10(cfg.acc_bias_noise_var_log10);
  gyro_bias_noise_var_ = exp10(cfg.gyro_bias_noise_var_log10);

  rosInfo(name_, "New dynamic parameters are set.");
}
}  // namespace state_estimation_eskf
