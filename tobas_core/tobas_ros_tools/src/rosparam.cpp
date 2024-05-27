#include <tobas_std_tools/string.hpp>

#include "../include/tobas_ros_tools/rosparam.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_ros
{
void getParam(ros::NodeHandle& nh, const string& key, size_t& param)
{
  int tmp;
  getParam(nh, key, tmp, NON_NEGATIVE);
  param = static_cast<size_t>(tmp);
}

void getParam(ros::NodeHandle& nh, const string& key, size_t& param, const size_t& _default)
{
  int tmp;
  getParam(nh, key, tmp, static_cast<int>(_default));

  if (tmp < 0)
  {
    ROS_ERROR_STREAM("Negative value is specified for '" << key << "'. The default value is used.");
    param = _default;
    return;
  }

  param = static_cast<size_t>(tmp);
}

void getParam(ros::NodeHandle& nh, const string& key, uint8_t& param)
{
  int tmp;
  getParam(nh, key, tmp);
  ROS_CHECK(
    nh, numeric_limits<uint8_t>::lowest() <= tmp && tmp <= numeric_limits<uint8_t>::max(),
    "The specified value for '" << key << "' is out of range of unsigned char.");
  param = static_cast<uint8_t>(tmp);
}

void getParam(ros::NodeHandle& nh, const string& key, uint8_t& param, const uint8_t& _default)
{
  int tmp;
  getParam(nh, key, tmp, static_cast<int>(_default));

  if (tmp < numeric_limits<uint8_t>::lowest() || numeric_limits<uint8_t>::max() < tmp)
  {
    ROS_ERROR_STREAM(
      "The specified value for '" << key << "' is out of range of unsigned char. The default value "
                                  << static_cast<int>(_default) << " is used.");
    param = _default;
    return;
  }

  param = static_cast<uint8_t>(tmp);
}

void getParam(ros::NodeHandle& nh, const string& key, Vector3d& param)
{
  vector<double> tmp;
  getParam(nh, key, tmp);
  ROS_CHECK(nh, tmp.size() == 3, "The size of '" << key << "' must be 3.");
  param = Map<Vector3d>(tmp.data());
}

void getParam(ros::NodeHandle& nh, const string& key, Vector3d& param, const Vector3d& _default)
{
  vector<double> param_vec;
  const vector<double> default_vec(_default.data(), _default.data() + _default.size());
  getParam(nh, key, param_vec, default_vec);

  if (param_vec.size() != 3)
  {
    ROS_ERROR_STREAM("The size of specified vector for '" << key << "' is not 3. The default vector is used.");
    param = _default;
    return;
  }

  param = Map<Vector3d>(param_vec.data());
}

void getParam(ros::NodeHandle& nh, const string& key, VectorXd& param)
{
  vector<double> tmp;
  getParam(nh, key, tmp);
  param = Map<VectorXd>(tmp.data(), tmp.size());
}

bool match(ros::NodeHandle& nh, const string& key)
{
  if (tobas_std::contains(key, '/'))
  {
    ROS_ERROR("The key cannot contain '/'.");
    return false;
  }

  string found;
  const auto res = nh.searchParam(key, found);
  // ROS_INFO_STREAM("Key: " << key << ", Found: " << found);
  return res;
}
}  // namespace tobas_ros
