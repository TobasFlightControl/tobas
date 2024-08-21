#include <eigen3/Eigen/LU>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_linear_control/kalman_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_tools/mr_dynamics.hpp>
#include <tobas_wind_model/dryden.hpp>

#include <tobas_msgs_adapter/Wind.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_kdl_msgs_adapter/Tree.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

#define E_XY DiagonalMatrix<double, 3>(1, 1, 0)
#define GRAV_W Vector3d(0, 0, tobas_std::kGravity)

using namespace std;
using namespace Eigen;

class WindEstimatorNode : public tobas::BaseNode
{
  static constexpr size_t kStateSize = 2;
  static constexpr double kInitWindStddev = 10.;  // [m/s]

  using self = WindEstimatorNode;
  using super = tobas::BaseNode;

public:
  explicit WindEstimatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  kdl::Tree tree_;

  tobas::MultirotorDynamicsComponents dynamics_;

  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  rclcpp::Time t_last_;
  ctrl::IdentityKalmanFilter kf_;
  tobas::DrydenComponents dryden_;
  tobas_msgs::msg::RotorSpeeds::ConstSharedPtr rotor_speeds_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::Wind> wind_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeeds> rotor_speeds_sub_;

  void updateInternalDataStructures();
  Matrix3d velCoef(const kdl::Rotation& R_W_B);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void rotorSpeedsCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& rotor_speeds);
};

WindEstimatorNode::WindEstimatorNode(const rclcpp::NodeOptions& options)
  : super("wind_estimator", options), dynamics_(drone_, tree_), kf_(kStateSize)
{
  kf_.initialize(Vector2d::Zero(), Vector2d::Constant(math::sqr(kInitWindStddev)).asDiagonal());
  kf_.setZero();

  // Register publishers
  wind_pub_ = createPublisher<tobas_msgs::Wind>(tobas::kWindTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  rotor_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsTopic, &self::rotorSpeedsCb, this);
}

void WindEstimatorNode::updateInternalDataStructures()
{
  dynamics_.updateInternalDataStructures();
}

Matrix3d WindEstimatorNode::velCoef(const kdl::Rotation& R_W_B)
{
  const auto drag_rotor_sum = dynamics_.dragRotorSum(rotor_speeds_->speeds);
  const auto& mass = dynamics_.mass();
  const auto& R_B_W = R_W_B.inverse().data;
  return (drag_rotor_sum / mass) * E_XY * R_B_W;
}

void WindEstimatorNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (odom->status != tobas_msgs::msg::Odometry::NO_ERROR)
    return;

  if (!is_initialized_)
  {
    if (
      drone_received_ && tree_received_ && rotor_speeds_ != nullptr
      && odom->frame.p.z() > tobas::kTakeoffAltitudeThreshold)
    {
      t_last_ = odom->header.stamp;
      is_initialized_ = true;
      TOBAS_INFO("Start to estimate wind speed.");
    }

    // 風速推定器は制御器と相互依存しているため，準備ができるまでは風速0を発行する．
    auto wind_msg = std::make_unique<tobas_msgs::Wind>();
    wind_msg->header.frame_id = tobas::kWorldFrame;
    wind_msg->header.stamp = odom->header.stamp;
    wind_msg->vel.data.setZero();
    wind_pub_->publish(move(wind_msg));

    return;
  }

  // 時刻を更新
  const auto dt = (odom->header.stamp - t_last_).seconds();
  t_last_ = odom->header.stamp;

  const Matrix3d& R_W_B = odom->frame.M.data;
  const Vector3d vel_W = R_W_B * odom->twist.vel.data;
  const Vector3d& acc_B = odom->accel.linear.data;
  const Vector3d grav_B = R_W_B.transpose() * GRAV_W;

  // 速度から加速度への係数行列を計算
  // Cvのランクは2だから，
  // 1. 最小二乗法で3軸とも推定
  // 2. 風速の水平成分のみを推定
  // の2つの選択肢がある．
  // 1の場合は水平成分の大きな誤差を垂直成分で説明しようとしてしまい精度が落ちるため，2を採用している．
  const Matrix3d Cv = velCoef(odom->frame.M);
  const Matrix2d Cv_hor_inv = Cv.topLeftCorner(kStateSize, kStateSize).inverse();  // 水平成分のみ

  // 風速の観測値
  const Vector2d wind_W_meas = Cv_hor_inv * (acc_B + grav_B + Cv * vel_W).head(kStateSize);
  kf_.y = wind_W_meas;

  // プロセスノイズの共分散
  const Vector2d relative_wind_vel = kf_.state() - odom->twist.vel.data.head(kStateSize);  // 相対風速
  dryden_.update(relative_wind_vel.norm(), odom->frame.p.z(), dt);
  kf_.Q(0, 0) = math::sqr(dryden_.noiseStddevLon());
  kf_.Q(1, 1) = math::sqr(dryden_.noiseStddevLat());

  // 観測ノイズの共分散
  const auto vel_cov_W = R_W_B * odom->velocity_covariance * R_W_B.transpose();
  const auto hor_vel_cov_W = vel_cov_W.topLeftCorner(kStateSize, kStateSize);
  const auto hor_acc_cov_B = odom->accel_covariance.topLeftCorner(kStateSize, kStateSize);
  kf_.R = hor_vel_cov_W + Cv_hor_inv * hor_acc_cov_B * Cv_hor_inv.transpose();

  // カルマンフィルタを更新
  kf_.update();

  // Publish wind message
  auto wind_msg = std::make_unique<tobas_msgs::Wind>();
  wind_msg->header.frame_id = tobas::kWorldFrame;
  wind_msg->header.stamp = odom->header.stamp;
  wind_msg->vel.data.head(kStateSize) = kf_.state();
  wind_pub_->publish(move(wind_msg));
}

void WindEstimatorNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;
  drone_received_ = true;

  if (tree_received_)
    updateInternalDataStructures();
}

void WindEstimatorNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  tree_received_ = true;

  if (drone_received_)
    updateInternalDataStructures();
}

void WindEstimatorNode::rotorSpeedsCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
}

RCLCPP_COMPONENTS_REGISTER_NODE(WindEstimatorNode)
