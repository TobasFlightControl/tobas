#include <tobas_algorithm/kahan.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs_adapter/Imu.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/action/accel_calibration.hpp>

#include "./util.hpp"

using namespace std;
using namespace Eigen;

class AccelCalibrationNode : public tobas::BaseNode
{
  static constexpr size_t kDataCount = 1000;
  static constexpr double kTimeout = 5.;  // [s]

  using self = AccelCalibrationNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_calibration_msgs::action::AccelCalibration;

public:
  explicit AccelCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t cnt_;
  array<algo::Kahan<double>, 3> acc_sum_;
  Eigen::Vector3d acc_top_;

  ros2::ActionServerPtr<ActionType> as_;

  bool getAccelMean(Eigen::Vector3d& des);

  void imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

AccelCalibrationNode::AccelCalibrationNode(const rclcpp::NodeOptions& options) : super("accel_calibration", options)
{
  as_ =
    createAction<ActionType>(tobas::kAccelCalibAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool AccelCalibrationNode::getAccelMean(Eigen::Vector3d& des)
{
  // 初期化
  cnt_ = 0;
  for (auto& sum : acc_sum_)
    sum.reset();

  // 一時的にIMUの購読を開始
  auto imu_sub = createSubscriber(hal::kImuTopic, &AccelCalibrationNode::imuCb, this);

  // データが溜まるまで待機
  if (!sleepUntil(shared_from_this(), [this]() { return cnt_ >= kDataCount; }, kTimeout))
    return false;

  // IMUの購読を終了
  imu_sub.reset();

  // 平均を計算
  for (size_t i = 0; i < 3; ++i)
    des(i) = acc_sum_.at(i).get() / cnt_;

  return true;
}

void AccelCalibrationNode::imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw)
{
  ++cnt_;
  for (size_t i = 0; i < 3; ++i)
    acc_sum_.at(i).add(imu_raw->accel(i));
}

rclcpp_action::GoalResponse
AccelCalibrationNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AccelCalibrationNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AccelCalibrationNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
  TOBAS_INFO("Accel calibration is requested.");

  // Create result
  const auto result = std::make_shared<ActionType::Result>();

  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  if (!getAccelMean(acc_top_))
  {
    result->message = "Timeout before accel data collection is completed.";
    goal_handle->abort(result);
    return;
  }
  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Vector3d acc_offset = acc_top_ - Vector3d(0, 0, tobas_std::kGravity);

  // Configに保存
  ptree::PropertyClient property_client(shared_from_this(), real::kPropertyServerFC);
  if (property_client.set(real::kConfigKey_AccOffsetX, acc_offset.x()) < 0)
  {
    result->message = property_client.errorMessage();
    goal_handle->abort(result);
    return;
  }
  if (property_client.set(real::kConfigKey_AccOffsetY, acc_offset.y()) < 0)
  {
    result->message = property_client.errorMessage();
    goal_handle->abort(result);
    return;
  }
  if (property_client.set(real::kConfigKey_AccOffsetZ, acc_offset.z()) < 0)
  {
    result->message = property_client.errorMessage();
    goal_handle->abort(result);
    return;
  }
  if (property_client.save() < 0)
  {
    result->message = property_client.errorMessage();
    goal_handle->abort(result);
    return;
  }

  result->message.clear();
  goal_handle->succeed(result);
}

RCLCPP_COMPONENTS_REGISTER_NODE(AccelCalibrationNode)
