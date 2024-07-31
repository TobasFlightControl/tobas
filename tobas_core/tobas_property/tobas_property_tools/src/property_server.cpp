#include <filesystem>

#include <tobas_linux/core.hpp>
#include <tobas_path_tools/core.hpp>

#include "../include/tobas_property_tools/property_server.hpp"
#include "../include/tobas_property_tools/constants.hpp"

using namespace std;
using namespace tobas_property_msgs;

namespace ptree
{
PropertyServer::PropertyServer(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh)
{
  // Get configuration file path
  if (!pnh.getParam(kIniPathParam, ini_path_))
  {
    RCLCPP_FATAL_STREAM("\"" << kIniPathParam << "\" is not specified. Exiting...");
    node.shutdown();
    return;
  }
  ini_path_ = linux::expandUser(ini_path_);

  if (filesystem::exists(ini_path_))
  {
    // If configuration file exists, try to load it.
    try
    {
      boost::property_tree::ini_parser::read_ini(ini_path_, pt_);
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(ini_path_ << " exists, but failed to load it.");
      node.shutdown();
      return;
    }
  }
  else
  {
    // If configuration file does not exist, create a new one.
    RCLCPP_INFO_STREAM(ini_path_ << " does not exist. Creating...");
    if (!path::createFilePath(ini_path_))
    {
      RCLCPP_FATAL_STREAM("Failed to create " << ini_path_ << ".");
      node.shutdown();
      return;
    }
  }

  // Advertise service servers
  const auto prefix = rclcpp::this_node::getName() + "/";
  get_bool_ss_ = node.advertiseService(prefix + kGetBoolSrv, &self::getCb<GetBoolRequest, GetBoolResponse>, this);
  get_int_ss_ = node.advertiseService(prefix + kGetIntSrv, &self::getCb<GetIntRequest, GetIntResponse>, this);
  get_double_ss_ = node.advertiseService(prefix + kGetDoubleSrv, &self::getCb<GetDoubleRequest, GetDoubleResponse>, this);
  get_string_ss_ = node.advertiseService(prefix + kGetStringSrv, &self::getCb<GetStringRequest, GetStringResponse>, this);
  set_bool_ss_ = node.advertiseService(prefix + kSetBoolSrv, &self::setCb<SetBoolRequest, SetBoolResponse>, this);
  set_int_ss_ = node.advertiseService(prefix + kSetIntSrv, &self::setCb<SetIntRequest, SetIntResponse>, this);
  set_double_ss_ = node.advertiseService(prefix + kSetDoubleSrv, &self::setCb<SetDoubleRequest, SetDoubleResponse>, this);
  set_string_ss_ = node.advertiseService(prefix + kSetStringSrv, &self::setCb<SetStringRequest, SetStringResponse>, this);
  save_file_ss_ = node.advertiseService(prefix + kSaveFileSrv, &self::saveFileCb, this);
}

bool PropertyServer::saveFileCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  try
  {
    boost::property_tree::ini_parser::write_ini(ini_path_, pt_);
  }
  catch (...)
  {
    res.success = false;
    res.message = "Failed to load " + ini_path_ + ".";
    return true;
  }

  res.success = true;
  res.message = "";

  return true;
}
}  // namespace ptree
