#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/srv/remove_rotor.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

class DroneServerNode : public tobas::BaseNode
{
  using self = DroneServerNode;
  using super = tobas::BaseNode;

  using RemoveRotor = tobas_msgs::srv::RemoveRotor;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;

  ros2::PublisherPtr<tobas::Drone> drone_pub_;
  ros2::ServiceServerPtr<RemoveRotor> remove_rotor_ss_;

  void publishDrone();

  bool fileParamCb(const std::string& p);
  void removeRotorCb(const RemoveRotor::Request::ConstSharedPtr& req, const RemoveRotor::Response::SharedPtr& res);
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options) : super("drone_server", options)
{
  addDynamicStringParam("tbsdrn_path", &self::fileParamCb, this);

  drone_pub_ = createPublisher<tobas::Drone>(tobas::kDroneTopic, true, true);
  remove_rotor_ss_ = createService<RemoveRotor>(tobas::kRemoveRotorSrv, &self::removeRotorCb, this);
}

void DroneServerNode::publishDrone()
{
  auto drone_msg = std::make_unique<tobas::Drone>(drone_);
  drone_pub_->publish(move(drone_msg));
}

bool DroneServerNode::fileParamCb(const std::string& p)
{
  // Load drone configuration
  if (!drone_.load(p))
  {
    TOBAS_ERROR("Failed to load drone configurations from \"", p, "\".");
    return false;
  }

  // Check drone configuration validity
  if (!drone_.isValid())
  {
    TOBAS_ERROR("Drone configurations are invalid.");
    return false;
  }

  // Publish drone configuration
  publishDrone();

  TOBAS_INFO("New drone configuration message is published.");
  return true;
}

void DroneServerNode::removeRotorCb(
  const RemoveRotor::Request::ConstSharedPtr& req,
  const RemoveRotor::Response::SharedPtr& res)
{
  for (size_t i = 0; i < drone_.numRotors(); ++i)
  {
    const auto& rotor = drone_.rotors.at(i);
    if (rotor.channel == req->channel)
    {
      drone_.rotors.erase(drone_.rotors.begin() + i);
      publishDrone();
      res->success = true;
      res->message.clear();
      return;
    }
  }

  res->success = false;
  res->message = "Rotor channel " + std::to_string(req->channel) + " does not exist.";
}

RCLCPP_COMPONENTS_REGISTER_NODE(DroneServerNode)
