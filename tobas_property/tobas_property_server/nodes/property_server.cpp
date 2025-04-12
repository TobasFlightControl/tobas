#include <std_srvs/srv/trigger.hpp>

#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>

#include <tobas_property_tree/property_tree.hpp>
#include <tobas_property_common/constants.hpp>
#include <tobas_property_msgs/srv/get_bool.hpp>
#include <tobas_property_msgs/srv/get_int.hpp>
#include <tobas_property_msgs/srv/get_double.hpp>
#include <tobas_property_msgs/srv/get_string.hpp>
#include <tobas_property_msgs/srv/set_bool.hpp>
#include <tobas_property_msgs/srv/set_int.hpp>
#include <tobas_property_msgs/srv/set_double.hpp>
#include <tobas_property_msgs/srv/set_string.hpp>

using namespace std;
using namespace std_srvs::srv;
using namespace tobas_property_msgs::srv;

namespace ptree
{
class PropertyServer : public tobas::BaseNode
{
  using self = PropertyServer;
  using super = tobas::BaseNode;

public:
  explicit PropertyServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Property tree
  PropertyTree pt_;

  // Service servers
  rclcpp::Service<GetBool>::SharedPtr get_bool_ss_;
  rclcpp::Service<GetInt>::SharedPtr get_int_ss_;
  rclcpp::Service<GetDouble>::SharedPtr get_double_ss_;
  rclcpp::Service<GetString>::SharedPtr get_string_ss_;
  rclcpp::Service<SetBool>::SharedPtr set_bool_ss_;
  rclcpp::Service<SetInt>::SharedPtr set_int_ss_;
  rclcpp::Service<SetDouble>::SharedPtr set_double_ss_;
  rclcpp::Service<SetString>::SharedPtr set_string_ss_;
  rclcpp::Service<Trigger>::SharedPtr save_file_ss_;

  template <typename SrvType>
  void getCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);
  template <typename SrvType>
  void setCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);

  void saveFileCb(const Trigger::Request::ConstSharedPtr& req, const Trigger::Response::SharedPtr& res);

  string keyWithSection(const string& section, const string& key);
};

PropertyServer::PropertyServer(const rclcpp::NodeOptions& options) : super("property_server", options)
{
  const auto file_path = getStringParam("file_path", "~/.config/tobas/config.json");
  if (!pt_.initialize(ros2::expandUser(file_path.c_str())))
  {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  // Advertise service servers
  const auto prefix = string(get_name()) + "/";
  get_bool_ss_ = createService<GetBool>(prefix + kGetBoolSrv, &self::getCb<GetBool>, this);
  get_int_ss_ = createService<GetInt>(prefix + kGetIntSrv, &self::getCb<GetInt>, this);
  get_double_ss_ = createService<GetDouble>(prefix + kGetDoubleSrv, &self::getCb<GetDouble>, this);
  get_string_ss_ = createService<GetString>(prefix + kGetStringSrv, &self::getCb<GetString>, this);
  set_bool_ss_ = createService<SetBool>(prefix + kSetBoolSrv, &self::setCb<SetBool>, this);
  set_int_ss_ = createService<SetInt>(prefix + kSetIntSrv, &self::setCb<SetInt>, this);
  set_double_ss_ = createService<SetDouble>(prefix + kSetDoubleSrv, &self::setCb<SetDouble>, this);
  set_string_ss_ = createService<SetString>(prefix + kSetStringSrv, &self::setCb<SetString>, this);
  save_file_ss_ = createService<Trigger>(prefix + kSaveFileSrv, &self::saveFileCb, this);
}

template <typename SrvType>
void PropertyServer::getCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res)
{
  if (pt_.get(keyWithSection(req->key, req->section), res->value))
  {
    res->success = true;
    res->message = "";
  }
  else
  {
    res->success = false;
    res->message = "Failed to get " + req->key + " in section " + req->section + ".";
  }
}

template <typename SrvType>
void PropertyServer::setCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res)
{
  pt_.set(keyWithSection(req->key, req->section), req->value);

  res->success = true;
  res->message = "";
}

void PropertyServer::saveFileCb(const Trigger::Request::ConstSharedPtr&, const Trigger::Response::SharedPtr& res)
{
  if (!pt_.save())
  {
    res->success = false;
    res->message = "Failed to save properties to \"" + pt_.filePath().string() + "\".";
  }

  res->success = true;
  res->message = "";
}

inline string PropertyServer::keyWithSection(const string& section, const string& key)
{
  return section + "." + key;
}
}  // namespace ptree

RCLCPP_COMPONENTS_REGISTER_NODE(ptree::PropertyServer)
