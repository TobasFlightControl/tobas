#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_real_ros/common.hpp>

#include "../include/tobas_calibration_ros/accel_calibration.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_calibration
{
AccelCalibrationRos::AccelCalibrationRos(const rclcpp::NodeOptions& options)
  : super(node, pnh, name), property_client_(node_, tobas_real_ros::kPropertyServerFC)
{
  ss_ = createService(kServiceName, &AccelCalibrationRos::executeCb, this);
}

bool AccelCalibrationRos::getAccelMean(Eigen::Vector3d& des)
{
  // 初期化
  cnt_ = 0;
  for (auto& sum : acc_sum_)
    sum.reset();

  // 一時的にIMUの購読を開始
  const auto imu_sub = createSubscriber(hal::kImuTopic, &AccelCalibrationRos::imuCb, this);

  // データが溜まるまで待機
  if (!ros2::spinUntil([this]() { return cnt_ == kDataCount; }, kTimeout))
    return false;

  // 平均を計算
  for (size_t i = 0; i < 3; ++i)
    des(i) = acc_sum_.at(i).get() / kDataCount;

  return true;
}

void AccelCalibrationRos::imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw)
{
  ++cnt_;
  for (size_t i = 0; i < 3; ++i)
    acc_sum_.at(i).add(imu_raw->accel(i));
}

bool AccelCalibrationRos::executeCb(SrvType::Request&, SrvType::Response& res)
{
  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  if (!getAccelMean(acc_top_))
  {
    res.success = false;
    res.message = "Timeout before accel data collection is completed.";
    return true;
  }
  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Vector3d acc_offset = acc_top_ - Vector3d(0, 0, tobas_std::kGravity);

  // Configに保存
  if (property_client_.set(tobas_real_ros::kConfigKey_AccOffsetX, acc_offset.x()) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_AccOffsetY, acc_offset.y()) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_AccOffsetZ, acc_offset.z()) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.save() < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }

  res.success = true;
  res.message = "";
  return true;
}
}  // namespace tobas_calibration
