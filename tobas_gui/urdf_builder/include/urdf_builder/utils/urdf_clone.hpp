#pragma once

#include <urdf_model/link.h>
#include <urdf_model/joint.h>

namespace urdf_builder
{
namespace utils
{
/**
 * @brief デフォルトのクローン関数．
 * インスタンス変数にポインタが含まれる場合，make_sharedではその実態はコピーされないため特殊化が必要．
 */
template <typename T>
std::shared_ptr<T> clone(const std::shared_ptr<T>& ptr)
{
  if (!ptr)
    return nullptr;

  return std::make_shared<T>(*ptr);
}

template <typename T>
std::vector<std::shared_ptr<T>> clone(const std::vector<std::shared_ptr<T>>& ptr_array)
{
  std::vector<std::shared_ptr<T>> res;
  for (const auto& ptr : ptr_array)
    res.push_back(clone(ptr));
  return res;
}

urdf::VisualSharedPtr clone(const urdf::VisualSharedPtr& visual);
urdf::CollisionSharedPtr clone(const urdf::CollisionSharedPtr& collision);
urdf::JointCalibrationSharedPtr clone(const urdf::JointCalibrationSharedPtr& calibration);
urdf::JointSharedPtr clone(const urdf::JointSharedPtr& joint);
urdf::LinkSharedPtr clone(const urdf::LinkSharedPtr& link);
}  // namespace utils
}  // namespace urdf_builder
