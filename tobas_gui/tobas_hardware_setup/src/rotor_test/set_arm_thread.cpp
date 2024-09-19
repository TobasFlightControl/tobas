#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include "tobas_hardware_setup/rotor_test/set_arm_thread.hpp"

namespace gui
{
namespace hardware_setup
{
SetArmThread::SetArmThread(rclcpp::Node::SharedPtr node, bool arming) : node_(node), arming_(arming)
{
}

void SetArmThread::run()
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(node_, tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming_;
  if (!sc.call(req))
  {
    Q_EMIT finished(false, "Arming service is not available.");
    return;
  }

  const auto& res = sc.getResponse();
  if (!res->success)
  {
    Q_EMIT finished(false, "Arming service failed: " + QString::fromStdString(res->message));
    return;
  }

  Q_EMIT finished(true, "");
}

void SetArmThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}

bool SetArmThread::setArming(bool arming)
{
  arming_ = arming;
}
}  // namespace hardware_setup
}  // namespace gui
