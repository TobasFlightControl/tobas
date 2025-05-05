#include "../include/tobas_rviz_plugin/transforms.hpp"

#include <geometric_shapes/check_isometry.h>
#include <boost/algorithm/string/trim.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "../include/tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.transforms");
}
}  // namespace

Transforms::Transforms(const std::string& target_frame) : target_frame_(target_frame)
{
  boost::trim(target_frame_);
  if (target_frame_.empty()) {
    RCLCPP_ERROR(getLogger(), "The target frame for Tobas Transforms cannot be empty.");
  }
  else {
    transforms_map_[target_frame_] = Eigen::Isometry3d::Identity();
  }
}

bool Transforms::sameFrame(const std::string& frame1, const std::string& frame2)
{
  if (frame1.empty() || frame2.empty()) {
    return false;
  }
  return frame1 == frame2;
}

Transforms::~Transforms() = default;

const std::string& Transforms::getTargetFrame() const
{
  return target_frame_;
}

const FixedTransformsMap& Transforms::getAllTransforms() const
{
  return transforms_map_;
}

void Transforms::setAllTransforms(const FixedTransformsMap& transforms)
{
  for (const auto& t : transforms) {
    ASSERT_ISOMETRY(t.second)  // unsanitized input, could contain a non-isometry
  }
  transforms_map_ = transforms;
}

bool Transforms::isFixedFrame(const std::string& frame) const
{
  if (frame.empty()) {
    return false;
  }
  else {
    return transforms_map_.find(frame) != transforms_map_.end();
  }
}

const Eigen::Isometry3d& Transforms::getTransform(const std::string& from_frame) const
{
  if (!from_frame.empty()) {
    FixedTransformsMap::const_iterator it = transforms_map_.find(from_frame);
    if (it != transforms_map_.end()) {
      return it->second;
    }
    // If no transform found in map, return identity
  }

  RCLCPP_ERROR(
    getLogger(),
    "Unable to transform from frame '%s' to frame '%s'. Returning identity.",
    from_frame.c_str(),
    target_frame_.c_str());

  // return identity
  static const Eigen::Isometry3d IDENTITY = Eigen::Isometry3d::Identity();
  return IDENTITY;
}

bool Transforms::canTransform(const std::string& from_frame) const
{
  if (from_frame.empty()) {
    return false;
  }
  else {
    return transforms_map_.find(from_frame) != transforms_map_.end();
  }
}

void Transforms::setTransform(const Eigen::Isometry3d& t, const std::string& from_frame)
{
  ASSERT_ISOMETRY(t)  // unsanitized input, could contain a non-isometry
  if (from_frame.empty()) {
    RCLCPP_ERROR(getLogger(), "Cannot record transform with empty name");
  }
  else {
    transforms_map_[from_frame] = t;
  }
}

void Transforms::setTransform(const geometry_msgs::msg::TransformStamped& transform)
{
  if (sameFrame(transform.child_frame_id, target_frame_)) {
    // convert message manually to ensure correct normalization for double (error < 1e-12)
    // tf2 only enforces float normalization (error < 1e-5)
    const auto& trans = transform.transform.translation;
    const auto& rot = transform.transform.rotation;
    Eigen::Translation3d translation(trans.x, trans.y, trans.z);
    Eigen::Quaterniond rotation(rot.w, rot.x, rot.y, rot.z);
    rotation.normalize();

    setTransform(translation * rotation, transform.header.frame_id);
  }
  else {
    RCLCPP_ERROR(
      getLogger(),
      "Given transform is to frame '%s', but frame '%s' was expected.",
      transform.child_frame_id.c_str(),
      target_frame_.c_str());
  }
}

void Transforms::setTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
{
  for (const geometry_msgs::msg::TransformStamped& transform : transforms) {
    setTransform(transform);
  }
}

void Transforms::copyTransforms(std::vector<geometry_msgs::msg::TransformStamped>& transforms) const
{
  transforms.resize(transforms_map_.size());
  std::size_t i = 0;
  for (FixedTransformsMap::const_iterator it = transforms_map_.begin(); it != transforms_map_.end(); ++it, ++i) {
    transforms[i] = tf2::eigenToTransform(it->second);
    transforms[i].child_frame_id = target_frame_;
    transforms[i].header.frame_id = it->first;
  }
}
}  // namespace tobas
