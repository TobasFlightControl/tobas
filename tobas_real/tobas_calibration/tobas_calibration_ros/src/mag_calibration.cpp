#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_real_ros/common.hpp>

#include "../include/tobas_calibration_ros/mag_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
MagCalibrationRos::MagCalibrationRos(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, tobas_real_ros::kPropertyServerFC)
{
  ss_ = nh_.advertiseService(kServiceName, &self::executeCb, this);
}

bool MagCalibrationRos::isValidEllipse(const SrvType::Request& req)
{
  math::EllipseTransformer mag_trans;

  mag_trans.a_xx = req.a_xx;
  mag_trans.a_yy = req.a_yy;
  mag_trans.a_zz = req.a_zz;
  mag_trans.a_xy = req.a_xy;
  mag_trans.a_yz = req.a_yz;
  mag_trans.a_zx = req.a_zx;
  mag_trans.b_x = req.b_x;
  mag_trans.b_y = req.b_y;
  mag_trans.b_z = req.b_z;
  mag_trans.c = req.c;

  return mag_trans.initialize();
}

bool MagCalibrationRos::executeCb(SrvType::Request& req, SrvType::Response& res)
{
  // 楕円体であることを確認
  if (!isValidEllipse(req))
  {
    res.success = false;
    res.message = "The estimated coefficients do not satisfy the conditions necessary for forming an ellipsoid.";
    return true;
  }

  // Configに保存
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseAxx, req.a_xx) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseAyy, req.a_yy) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseAzz, req.a_zz) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseAxy, req.a_xy) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseAyz, req.a_yz) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseAzx, req.a_zx) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseBx, req.b_x) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseBy, req.b_y) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseBz, req.b_z) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_MagEllipseC, req.c) < 0)
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
