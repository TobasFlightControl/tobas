#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_msgs/Drone.hpp>

using namespace std;

class DroneServerNode : public tobas::BaseNode
{
  static constexpr char kFilePath[] = "tbsdrn_path";

  using self = DroneServerNode;
  using super = tobas::BaseNode;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PublisherPtr<tobas::Drone> drone_pub_;

  bool fileParamCb(const string& p);
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options) : super("drone_server", options)
{
  addDynamicStringParam(kFilePath, &self::fileParamCb, this);
  publishDynamicParameterDescriptions();

  drone_pub_ = createPublisher<tobas::Drone>(tobas::kDroneTopic, true);
}

bool DroneServerNode::fileParamCb(const string& p)
{
  auto drone = std::make_unique<tobas::Drone>();

  if (!drone->load(p))
  {
    TOBAS_ERROR("Failed to load drone configurations from \"", p, "\".");
    return false;
  }

  drone_pub_->publish(move(drone));

  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DroneServerNode)
