#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

class DroneServerNode : public tobas::BaseNode
{
  using self = DroneServerNode;
  using super = tobas::BaseNode;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;

  ros2::PublisherPtr<tobas::Drone> drone_pub_;

  void publishDrone();

  bool fileParamCb(const std::string& p);
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options) : super("drone_server", options)
{
  addDynamicStringParam("tbsdrn_path", &self::fileParamCb, this);

  drone_pub_ = createPublisher<tobas::Drone>(tobas::kDroneTopic, true, true);
}

void DroneServerNode::publishDrone()
{
  auto drone_msg = std::make_unique<tobas::Drone>(drone_);
  drone_pub_->publish(move(drone_msg));
}

bool DroneServerNode::fileParamCb(const std::string& p)
{
  // Load drone configuration
  if (!drone_.load(p)) {
    TOBAS_ERROR("Failed to load drone configurations from \"", p, "\".");
    return false;
  }

  // Check drone configuration validity
  if (!drone_.isValid()) {
    TOBAS_ERROR("Drone configurations are invalid.");
    return false;
  }

  // Publish drone configuration
  publishDrone();

  TOBAS_INFO("New drone configuration message is published.");
  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DroneServerNode)
