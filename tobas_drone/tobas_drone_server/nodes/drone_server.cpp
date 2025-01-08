#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/srv/enable_rotor.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

class DroneServerNode : public tobas::BaseNode
{
  using self = DroneServerNode;
  using super = tobas::BaseNode;

  using EnableRotor = tobas_msgs::srv::EnableRotor;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  std::map<size_t, double> max_rot_speeds_;

  ros2::PublisherPtr<tobas::Drone> drone_pub_;
  ros2::ServiceServerPtr<EnableRotor> enable_rotor_ss_;

  void publishDrone();

  bool fileParamCb(const std::string& p);
  void enableRotorCb(const EnableRotor::Request::ConstSharedPtr& req, const EnableRotor::Response::SharedPtr& res);
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options) : super("drone_server", options)
{
  addDynamicStringParam("tbsdrn_path", &self::fileParamCb, this);

  drone_pub_ = createPublisher<tobas::Drone>(tobas::kDroneTopic, true, true);
  enable_rotor_ss_ = createService<EnableRotor>(tobas::kEnableRotorSrv, &self::enableRotorCb, this);
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

  // Save max rotation speeds
  max_rot_speeds_.clear();
  for (const auto& [_, rotor] : drone_.rotors)
    max_rot_speeds_[rotor.channel] = rotor.max_rot_speed;

  TOBAS_INFO("New drone configuration message is published.");
  return true;
}

void DroneServerNode::enableRotorCb(
  const EnableRotor::Request::ConstSharedPtr& req,
  const EnableRotor::Response::SharedPtr& res)
{
  if (!drone_.rotors.contains(req->channel))
  {
    res->success = false;
    res->message = "Rotor channel " + std::to_string(req->channel) + " does not exist.";
    return;
  }

  // 動作中にモータの個数が変わるとGCS等に悪影響が出る恐れがあるため，モータの生死を最大回転速度で表現．
  auto& rotor = drone_.rotors.at(req->channel);
  if (req->enable)
    rotor.max_rot_speed = max_rot_speeds_.at(req->channel);  // 保存しておいた最大速度を回復
  else
    rotor.max_rot_speed = 0.;  // 最大速度を0にすることでモータをアクチュエータとして使用できないようにする

  publishDrone();

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(DroneServerNode)
