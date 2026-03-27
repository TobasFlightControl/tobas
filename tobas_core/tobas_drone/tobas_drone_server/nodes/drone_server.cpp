#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>

namespace tobas
{
class DroneServerNode : public BaseNode
{
  using self = DroneServerNode;
  using super = BaseNode;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone drone_;

  ros2::PublisherPtr<Drone> drone_pub_;

  void publishDrone();

  bool fileParamCb(const std::string& p);
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options)
  : super("drone_server", nodeOptions_DParam(options))
{
  addDynamicStringParam("tbsdrn_path", &self::fileParamCb, this);

  drone_pub_ = createPublisher<Drone>(topic::kDrone, true, true);
}

void DroneServerNode::publishDrone()
{
  auto drone_msg = std::make_unique<Drone>(drone_);
  drone_pub_->publish(std::move(drone_msg));
}

bool DroneServerNode::fileParamCb(const std::string& p)
{
  // Load drone configuration
  if (!drone_.load(p)) {
    TOBAS_ERROR("Failed to load drone configuration from \"", p, "\".");
    return false;
  }

  // Check drone configuration validity
  if (!drone_.isValid()) {
    TOBAS_ERROR("Drone configuration is invalid.");
    return false;
  }

  // Publish drone configuration
  publishDrone();

  TOBAS_INFO("New drone configuration message is published.");
  return true;
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::DroneServerNode)
