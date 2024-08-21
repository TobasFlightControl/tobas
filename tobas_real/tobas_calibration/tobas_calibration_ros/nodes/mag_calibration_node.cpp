#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tools/property_client.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/srv/mag_calibration.hpp>

using namespace std;

class MagCalibrationNode : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "mag_calibration";

  using self = MagCalibrationNode;
  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::srv::MagCalibration;

public:
  explicit MagCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::ServicePtr<SrvType> ss_;

  bool isValidEllipse(const SrvType::Request::ConstSharedPtr& req);
  void executeCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);
};

MagCalibrationNode::MagCalibrationNode(const rclcpp::NodeOptions& options) : super("mag_calibration", options)
{
  ss_ = createService<SrvType>(kServiceName, &self::executeCb, this);
}

bool MagCalibrationNode::isValidEllipse(const SrvType::Request::ConstSharedPtr& req)
{
  math::EllipseTransformer mag_trans;

  mag_trans.a_xx = req->a_xx;
  mag_trans.a_yy = req->a_yy;
  mag_trans.a_zz = req->a_zz;
  mag_trans.a_xy = req->a_xy;
  mag_trans.a_yz = req->a_yz;
  mag_trans.a_zx = req->a_zx;
  mag_trans.b_x = req->b_x;
  mag_trans.b_y = req->b_y;
  mag_trans.b_z = req->b_z;
  mag_trans.c = req->c;

  return mag_trans.initialize();
}

void MagCalibrationNode::executeCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res)
{
  // 楕円体であることを確認
  if (!isValidEllipse(req))
  {
    res->success = false;
    res->message = "The estimated coefficients do not satisfy the conditions necessary for forming an ellipsoid.";
    return;
  }

  // Configに保存
  ptree::PropertyClient property_client(shared_from_this(), real::kPropertyServerFC);
  if (property_client.set(real::kConfigKey_MagEllipseAxx, req->a_xx) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseAyy, req->a_yy) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseAzz, req->a_zz) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseAxy, req->a_xy) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseAyz, req->a_yz) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseAzx, req->a_zx) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseBx, req->b_x) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseBy, req->b_y) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseBz, req->b_z) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_MagEllipseC, req->c) < 0)
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

RCLCPP_COMPONENTS_REGISTER_NODE(MagCalibrationNode)
