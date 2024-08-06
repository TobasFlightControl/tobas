#include <fstream>
#include <yaml-cpp/yaml.h>

#include <tobas_ros2_tools/node.hpp>

#include <tobas_drone_core/joint.hpp>
#include <tobas_drone_msgs/Drone.hpp>

using namespace std;

namespace tobas
{
class DroneServer : public ros2::Node
{
  static constexpr char kFilePath[] = "tbsdrn_path";

  using self = DroneServer;
  using super = ros2::Node;

public:
  explicit DroneServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ParamHandlePtr file_handle_;
  PublisherPtr<Drone> drone_pub_;

  bool publishDrone(const string& file_path) const;
  void fileParamCb(const rclcpp::Parameter& p);
};

DroneServer::DroneServer(const rclcpp::NodeOptions& options) : super("drone_server", options)
{
  declare_parameter<string>(kFilePath);

  file_handle_ = addParamCallback(kFilePath, &self::fileParamCb, this);  // コールバックを定義した時点で一度呼ばれる

  // TODO: tobas_constantsにトピックをまとめて使う
  drone_pub_ = createPublisher<Drone>("drone", true);
}

bool DroneServer::publishDrone(const string& file_path) const
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

void DroneServer::fileParamCb(const rclcpp::Parameter& p)
{
  if (!publishDrone(p.as_string()))
    return;

  TOBAS_INFO("Drone structure is updated.");
  return;
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::DroneServer)
