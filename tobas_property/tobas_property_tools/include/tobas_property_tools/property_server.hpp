#pragma once

#include <boost/property_tree/ini_parser.hpp>
#include <std_srvs/Trigger.h>

#include <tobas_tools/node.hpp>

#include <tobas_property_msgs/GetBool.h>
#include <tobas_property_msgs/GetInt.h>
#include <tobas_property_msgs/GetDouble.h>
#include <tobas_property_msgs/GetString.h>
#include <tobas_property_msgs/SetBool.h>
#include <tobas_property_msgs/SetInt.h>
#include <tobas_property_msgs/SetDouble.h>
#include <tobas_property_msgs/SetString.h>

namespace ptree
{
class PropertyServer : public tobas::BaseNode
{
  using self = PropertyServer;
  using super = tobas::BaseNode;

public:
  explicit PropertyServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // ROS parameters
  std::string ini_path_;

  // Property tree
  boost::property_tree::ptree pt_;

  // Service servers
  ros::ServiceServer get_bool_ss_;
  ros::ServiceServer get_int_ss_;
  ros::ServiceServer get_double_ss_;
  ros::ServiceServer get_string_ss_;
  ros::ServiceServer set_bool_ss_;
  ros::ServiceServer set_int_ss_;
  ros::ServiceServer set_double_ss_;
  ros::ServiceServer set_string_ss_;
  ros::ServiceServer save_file_ss_;

  inline std::string keyWithSection(const std::string& section, const std::string& key) const;

  template <typename T>
  bool get(const std::string& section, const std::string& key, T& value) const;
  template <typename T>
  void put(const std::string& section, const std::string& key, const T& value);

  template <typename SrvType>
  bool getCb(SrvType::Request& req, SrvType::Response& res);
  template <typename SrvType>
  bool putCb(SrvType::Request& req, SrvType::Response& res);

  bool saveFileCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
};

inline std::string PropertyServer::keyWithSection(const std::string& section, const std::string& key) const
{
  return section + "." + key;
}

template <typename T>
bool PropertyServer::get(const std::string& section, const std::string& key, T& value) const
{
  const auto optional = pt_.get_optional<T>(keyWithSection(section, key));
  if (!optional)
    return false;

  value = optional.get();
  return true;
}

template <typename T>
void PropertyServer::put(const std::string& section, const std::string& key, const T& value)
{
  pt_.put(keyWithSection(section, key), value);
}

template <typename SrvType>
bool PropertyServer::getCb(SrvType::Request& req, SrvType::Response& res)
{
  if (get(req.section, req.key, res.value))
  {
    res.success = true;
    res.message = "";
  }
  else
  {
    res.success = false;
    res.message = "Failed to get " + req.key + " in section " + req.section + ".";
  }

  return true;
}

template <typename SrvType>
bool PropertyServer::putCb(SrvType::Request& req, SrvType::Response& res)
{
  put(req.section, req.key, req.value);

  res.success = true;
  res.message = "";

  return true;
}
}  // namespace ptree
