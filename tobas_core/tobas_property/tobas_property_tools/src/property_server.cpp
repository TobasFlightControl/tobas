#include <filesystem>

#include <tobas_linux/core.hpp>
#include <tobas_path_tools/core.hpp>

#include "../include/tobas_property_tools/property_server.hpp"
#include "../include/tobas_property_tools/constants.hpp"

using namespace std;
using namespace std_srvs::srv;
using namespace tobas_property_msgs::srv;

namespace ptree
{
PropertyServer::PropertyServer() : super("property_server")
{
  ini_path_ = linux::expandUser(getStringParam(kIniPathParam, "hoge"));

  if (filesystem::exists(ini_path_))
  {
    // If configuration file exists, try to load it.
    try
    {
      boost::property_tree::ini_parser::read_ini(ini_path_, pt_);
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(get_logger(), ini_path_ << " exists, but failed to load it.");
      rclcpp::shutdown();
      return;
    }
  }
  else
  {
    // If configuration file does not exist, create a new one.
    RCLCPP_INFO_STREAM(get_logger(), ini_path_ << " does not exist. Creating...");
    if (!path::createFilePath(ini_path_))
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to create " << ini_path_ << ".");
      rclcpp::shutdown();
      return;
    }
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

void PropertyServer::saveFileCb(const Trigger::Request::ConstSharedPtr&, const Trigger::Response::SharedPtr& res)
{
  try
  {
    boost::property_tree::ini_parser::write_ini(ini_path_, pt_);
  }
  catch (...)
  {
    res->success = false;
    res->message = "Failed to load " + ini_path_ + ".";
    return;
  }

  res->success = true;
  res->message = "";
}
}  // namespace ptree
