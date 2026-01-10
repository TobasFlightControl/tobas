#include <tobas_node/node.hpp>
#include <tobas_ntrip_client/ntrip_client.hpp>

class NtripClientNode : public tobas::BaseNode
{
  using self = NtripClientNode;
  using super = tobas::BaseNode;

public:
  explicit NtripClientNode(const rclcpp::NodeOptions& options);
private:
  static constexpr char kDefaultServerIp[] = "3.143.243.81"; // RTK2GO http://rtk2go.com/
  static constexpr int kDefaultServerPort = 2101;
  static constexpr char kDefaultPassword[] = "none";

  ntrip::NtripClient ntrip_client_;
};

NtripClientNode::NtripClientNode(const rclcpp::NodeOptions& options)
: super("ntrip_client", options)
{
  auto server_ip = getStringParam("server_ip", kDefaultServerIp);
  auto server_port = getIntParam("server_port", kDefaultServerPort);
  auto mount_point = getStringParam("mount_point");
  auto user_name = getStringParam("user_name");
  auto password = getStringParam("password", kDefaultPassword);
  auto latitude = getDoubleParam("latitude"); // 最も近くのmount pointを探すために用いる 大体でok degree
  auto longitude = getDoubleParam("longitude");

  if (!ntrip_client_.initialize(server_ip.c_str(), server_port, mount_point.c_str(), user_name.c_str(), password.c_str(), latitude, longitude)) {
    TOBAS_ERROR("Failed to initialize NTRIP client.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(NtripClientNode)
