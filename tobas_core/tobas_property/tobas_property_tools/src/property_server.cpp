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
  declare_parameter<std::string>(kIniPathParam, "");
  if (!get_parameter(kIniPathParam, ini_path_) || ini_path_.empty())
  {
    RCLCPP_FATAL_STREAM(get_logger(), "\"" << kIniPathParam << "\" is not specified. Exiting...");
    rclcpp::shutdown();
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
  get_bool_ss_ = create_service<GetBool>(
    prefix + kGetBoolSrv, bind(&self::getCb<GetBool>, this, placeholders::_1, placeholders::_2));
  get_int_ss_ =
    create_service<GetInt>(prefix + kGetIntSrv, bind(&self::getCb<GetInt>, this, placeholders::_1, placeholders::_2));
  get_double_ss_ = create_service<GetDouble>(
    prefix + kGetDoubleSrv, bind(&self::getCb<GetDouble>, this, placeholders::_1, placeholders::_2));
  get_string_ss_ = create_service<GetString>(
    prefix + kGetStringSrv, bind(&self::getCb<GetString>, this, placeholders::_1, placeholders::_2));
  set_bool_ss_ = create_service<SetBool>(
    prefix + kSetBoolSrv, bind(&self::setCb<SetBool>, this, placeholders::_1, placeholders::_2));
  set_int_ss_ =
    create_service<SetInt>(prefix + kSetIntSrv, bind(&self::setCb<SetInt>, this, placeholders::_1, placeholders::_2));
  set_double_ss_ = create_service<SetDouble>(
    prefix + kSetDoubleSrv, bind(&self::setCb<SetDouble>, this, placeholders::_1, placeholders::_2));
  set_string_ss_ = create_service<SetString>(
    prefix + kSetStringSrv, bind(&self::setCb<SetString>, this, placeholders::_1, placeholders::_2));
  save_file_ss_ =
    create_service<Trigger>(prefix + kSaveFileSrv, bind(&self::saveFileCb, this, placeholders::_1, placeholders::_2));
}

bool PropertyServer::saveFileCb(const Trigger::Request::ConstSharedPtr&, const Trigger::Response::SharedPtr& res)
{
  try
  {
    boost::property_tree::ini_parser::write_ini(ini_path_, pt_);
  }
  catch (...)
  {
    res->success = false;
    res->message = "Failed to load " + ini_path_ + ".";
    return true;
  }

  res->success = true;
  res->message = "";

  return true;
}
}  // namespace ptree
