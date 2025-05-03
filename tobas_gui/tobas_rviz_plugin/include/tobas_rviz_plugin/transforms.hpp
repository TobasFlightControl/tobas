#pragma once

#include <map>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "./class_forward.hpp"

namespace tobas
{
TOBAS_CLASS_FORWARD(Transforms);  // Defines TransformsPtr, ConstPtr, WeakPtr... etc

/* Map frame names to the transformation matrix that can transform objects from the frame name to the planning */
using FixedTransformsMap = std::map<
  std::string,
  Eigen::Isometry3d,
  std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>>;

/**
 * @brief Provides an implementation of a snapshot of a transform tree
 * that can be easily queried for transforming different quantities.
 * Transforms are maintained as a list of transforms to a particular frame.
 * All stored transforms are considered fixed.
 */
class Transforms
{
public:
  /**
   * @brief Transforms cannot be copy-constructed
   */
  Transforms(const Transforms&) = delete;

  /**
   * @brief Transforms cannot be copy-assigned
   */
  Transforms& operator=(const Transforms&) = delete;

  /**
   * @brief Construct a transform list
   */
  Transforms(const std::string& target_frame);

  /**
   * @brief Destructor
   */
  virtual ~Transforms();

  /* Check if two frames end up being the same once the missing / are added as prefix (if they are missing) */
  static bool sameFrame(const std::string& frame1, const std::string& frame2);

  /**
   * @brief Get the planning frame corresponding to this set of transforms
   * @return The planning frame
   */
  const std::string& getTargetFrame() const;

  /**
   * @brief Return all the transforms
   * @return A map from string names of frames to corresponding Eigen::Isometry3d (w.r.t the planning frame). The
   * transforms are guaranteed to be valid isometries.
   */
  const FixedTransformsMap& getAllTransforms() const;

  /**
   * @brief Get a vector of all the transforms as ROS messages
   * @param transforms The output transforms. They are guaranteed to be valid isometries.
   */
  void copyTransforms(std::vector<geometry_msgs::msg::TransformStamped>& transforms) const;

  /**
   * @brief Set a transform in the transform tree (adding it if necessary)
   * @param t The input transform (w.r.t the target frame)
   * @param from_frame The frame for which the input transform is specified
   */
  void setTransform(const Eigen::Isometry3d& t, const std::string& from_frame);

  /**
   * @brief Set a transform in the transform tree (adding it if necessary)
   * @param transform The input transform (the frame_id must match the target frame)
   */
  void setTransform(const geometry_msgs::msg::TransformStamped& transform);

  /**
   * @brief Set a transform in the transform tree (adding it if necessary)
   * @param transform The input transforms (the frame_id must match the target frame)
   */
  void setTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms);

  /**
   * @brief Set all the transforms: a map from string names of frames to corresponding Eigen::Isometry3d
   * (w.r.t the planning frame)
   */
  void setAllTransforms(const FixedTransformsMap& transforms);

  /**
   * @brief Transform a vector in from_frame to the target_frame
   *
   * @param from_frame The frame from which the transform is computed
   * @param v_in The input vector (in from_frame)
   * @param v_out The resultant (transformed) vector
   *
   * @note assumes that v_in and v_out are "free" vectors, not points
   */
  void transformVector3(const std::string& from_frame, const Eigen::Vector3d& v_in, Eigen::Vector3d& v_out) const
  {
    // getTransform() returns a valid isometry by contract
    v_out = getTransform(from_frame).linear() * v_in;
  }

  /**
   * @brief Transform a quaternion in from_frame to the target_frame
   * @param from_frame The frame in which the input quaternion is specified
   * @param v_in The input quaternion (in from_frame). Make sure the quaternion is normalized.
   * @param v_out The resultant (transformed) quaternion. It is guaranteed to be a valid and normalized quaternion.
   */
  void
  transformQuaternion(const std::string& from_frame, const Eigen::Quaterniond& q_in, Eigen::Quaterniond& q_out) const
  {
    // getTransform() returns a valid isometry by contract
    q_out = getTransform(from_frame).linear() * q_in;
  }

  /**
   * @brief Transform a rotation matrix in from_frame to the target_frame
   * @param from_frame The frame in which the input rotation matrix is specified
   * @param m_in The input rotation matrix (in from_frame). Make sure the matrix is a valid rotation matrix.
   * @param m_out The resultant (transformed) rotation matrix. It is guaranteed to be a valid rotation matrix.
   */
  void transformRotationMatrix(const std::string& from_frame, const Eigen::Matrix3d& m_in, Eigen::Matrix3d& m_out) const
  {
    // getTransform() returns a valid isometry by contract
    m_out = getTransform(from_frame).linear() * m_in;
  }

  /**
   * @brief Transform a pose in from_frame to the target_frame
   * @param from_frame The frame in which the input pose is specified
   * @param t_in The input pose (in from_frame). Make sure the pose is a valid isometry.
   * @param t_out The resultant (transformed) pose. It is guaranteed to be a valid isometry.
   */
  void transformPose(const std::string& from_frame, const Eigen::Isometry3d& t_in, Eigen::Isometry3d& t_out) const
  {
    // getTransform() returns a valid isometry by contract
    t_out = getTransform(from_frame) * t_in;
  }

  /**
   * @brief Check whether data can be transformed from a particular frame
   */
  virtual bool canTransform(const std::string& from_frame) const;

  /**
   * @brief Check whether a frame stays constant as the state of the robot model changes.
   * This is true for any transform mainatined by this object.
   */
  virtual bool isFixedFrame(const std::string& frame) const;

  /**
   * @brief Get transform for from_frame (w.r.t target frame)
   * @param from_frame The string id of the frame for which the transform is being computed
   * @return The required transform. It is guaranteed to be a valid isometry.
   */
  virtual const Eigen::Isometry3d& getTransform(const std::string& from_frame) const;

protected:
  std::string target_frame_;
  FixedTransformsMap transforms_map_;
};
}  // namespace tobas
