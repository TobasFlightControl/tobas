#include <filesystem>

#include <tobas_path_tools/core.hpp>
#include <tobas_ros_tools/rosparam.hpp>

#include "../include/tobas_property_tools/property_server.hpp"
#include "../include/tobas_property_tools/constants.hpp"

using namespace std;

namespace ptree
{
PropertyServer::PropertyServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  // Get ROS parameters
  tobas_ros::getParam(pnh, "ini_path", ini_path_);

  if (filesystem::exists(ini_path_))
  {
    // If configuration file exists, try to load it.
    try
    {
      boost::property_tree::ini_parser::read_ini(ini_path_, pt_);
    }
    catch (...)
    {
      ROS_FATAL_STREAM(ini_path_ << " exists, but failed to load it.");
      nh.shutdown();
      return;
    }
  }
  else
  {
    // If configuration file does not exist, create a new one.
    ROS_INFO_STREAM(ini_path_ << " does not exist. Creating...");
    if (!path::createFilePath(ini_path_))
    {
      ROS_FATAL_STREAM("Failed to create " << ini_path_ << ".");
      nh.shutdown();
      return;
    }
  }

  // Advertise service servers
  get_bool_ss_ = nh.advertiseService(kGetBoolSrv, &self::getCb<tobas_property_msgs::GetBool>, this);
  get_int_ss_ = nh.advertiseService(kGetIntSrv, &self::getCb<tobas_property_msgs::GetInt>, this);
  get_double_ss_ = nh.advertiseService(kGetDoubleSrv, &self::getCb<tobas_property_msgs::GetDouble>, this);
  get_string_ss_ = nh.advertiseService(kGetStringSrv, &self::getCb<tobas_property_msgs::GetString>, this);
  set_bool_ss_ = nh.advertiseService(kSetBoolSrv, &self::setCb<tobas_property_msgs::SetBool>, this);
  set_int_ss_ = nh.advertiseService(kSetIntSrv, &self::setCb<tobas_property_msgs::SetInt>, this);
  set_double_ss_ = nh.advertiseService(kSetDoubleSrv, &self::setCb<tobas_property_msgs::SetDouble>, this);
  set_string_ss_ = nh.advertiseService(kSetStringSrv, &self::setCb<tobas_property_msgs::SetString>, this);
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
