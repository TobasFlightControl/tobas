#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_dparam_common/constants.hpp>
#include <tobas_dparam_msgs/srv/set_bool.hpp>
#include <tobas_dparam_msgs/srv/set_int.hpp>
#include <tobas_dparam_msgs/srv/set_double.hpp>
#include <tobas_dparam_msgs/srv/set_string.hpp>

using namespace std;
using namespace tobas_dparam_msgs::srv;

namespace dparam
{
class DynamicParamServer : public tobas::BaseNode
{
  using self = DynamicParamServer;
  using super = tobas::BaseNode;

public:
  explicit DynamicParamServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  vector<rclcpp::ServiceBase::SharedPtr> services_;

  template <typename SrvType, typename ValueType>
  void callback(const typename SrvType::Request::ConstSharedPtr& req, const typename SrvType::Response::SharedPtr& res);
};

DynamicParamServer::DynamicParamServer(const rclcpp::NodeOptions& options) : super("dynamic_parameter_server", options)
{
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  services_.push_back(createService<SetBool>(kSetBoolSrv, &self::callback<SetBool, bool>, this, cb_group_));
  services_.push_back(createService<SetInt>(kSetIntSrv, &self::callback<SetInt, long>, this, cb_group_));
  services_.push_back(createService<SetDouble>(kSetDoubleSrv, &self::callback<SetDouble, double>, this, cb_group_));
  services_.push_back(createService<SetString>(kSetStringSrv, &self::callback<SetString, string>, this, cb_group_));
}

template <typename SrvType, typename ValueType>
void DynamicParamServer::callback(
  const typename SrvType::Request::ConstSharedPtr& req,
  const typename SrvType::Response::SharedPtr& res)
{
  ros2::SyncParamClient client(shared_from_this(), req->node_name);
  if (!client.setParam<ValueType>(req->param_name, req->value))
  {
    res->success = false;
    return;
  }

  res->success = true;
}
}  // namespace dparam

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto node = make_shared<dparam::DynamicParamServer>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
}
