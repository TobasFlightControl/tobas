#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_msgs/Drone.hpp>

using namespace std;

namespace tobas
{
class DroneServerNode : public BaseNode
{
  static constexpr char kFilePath[] = "tbsdrn_path";

  using self = DroneServerNode;
  using super = BaseNode;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PublisherPtr<Drone> drone_pub_;

  bool publishDrone(const string& file_path) const;
  bool fileParamCb(const string& p);
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options) : super("drone_server", options)
{
  addDynamicStringParam(kFilePath, &self::fileParamCb, this);
  publishDynamicParameterDescriptions();

  drone_pub_ = createPublisher<Drone>(tobas::kDroneTopic, true);
}

bool DroneServerNode::publishDrone(const string& file_path) const
{
  auto drone = std::make_unique<Drone>();

  if (!drone->load(file_path))
  {
    TOBAS_ERROR("Failed to load drone configurations from \"", file_path, "\".");
    return false;
  }

  drone_pub_->publish(move(drone));

  return true;
}

bool DroneServerNode::fileParamCb(const string& p)
{
  return publishDrone(p);
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::DroneServerNode)
