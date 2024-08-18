#include <tobas_algorithm/kahan.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Imu.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/srv/accel_calibration.hpp>

using namespace std;
using namespace Eigen;

class AccelCalibrationNode : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "accel_calibration";

  static constexpr size_t kDataCount = 1000;
  static constexpr double kTimeout = 5.;  // [s]

  using self = AccelCalibrationNode;
  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::srv::AccelCalibration;

public:
  explicit AccelCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t cnt_;
  array<algo::Kahan<double>, 3> acc_sum_;
  Eigen::Vector3d acc_top_;

  ServicePtr<SrvType> ss_;

  bool getAccelMean(Eigen::Vector3d& des);

  void imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw);
  void executeCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);
};

AccelCalibrationNode::AccelCalibrationNode(const rclcpp::NodeOptions& options) : super("accel_calibration", options)
{
  ss_ = createService<SrvType>(kServiceName, &self::executeCb, this);
}

bool AccelCalibrationNode::getAccelMean(Eigen::Vector3d& des)
{
  // 初期化
  cnt_ = 0;
  for (auto& sum : acc_sum_)
    sum.reset();

  // 一時的にIMUの購読を開始
  const auto imu_sub = createSubscriber(hal::kImuTopic, &AccelCalibrationNode::imuCb, this);

  // データが溜まるまで待機
  if (!ros2::spinUntil(shared_from_this(), [this]() { return cnt_ == kDataCount; }, kTimeout))
    return false;

  // 平均を計算
  for (size_t i = 0; i < 3; ++i)
    des(i) = acc_sum_.at(i).get() / kDataCount;

  return true;
}

void AccelCalibrationNode::imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw)
{
  ++cnt_;
  for (size_t i = 0; i < 3; ++i)
    acc_sum_.at(i).add(imu_raw->accel(i));
}

void AccelCalibrationNode::executeCb(const SrvType::Request::ConstSharedPtr&, const SrvType::Response::SharedPtr& res)
{
  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  if (!getAccelMean(acc_top_))
  {
    res->success = false;
    res->message = "Timeout before accel data collection is completed.";
    return;
  }
  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Vector3d acc_offset = acc_top_ - Vector3d(0, 0, tobas_std::kGravity);

  // Configに保存
  ptree::PropertyClient property_client(shared_from_this(), real::kPropertyServerFC);
  if (property_client.set(real::kConfigKey_AccOffsetX, acc_offset.x()) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_AccOffsetY, acc_offset.y()) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_AccOffsetZ, acc_offset.z()) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.save() < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(AccelCalibrationNode)
