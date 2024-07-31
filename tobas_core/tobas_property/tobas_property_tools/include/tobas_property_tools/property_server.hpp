#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <std_srvs/Trigger.h>

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
class PropertyServer
{
  static constexpr char kIniPathParam[] = "ini_path";

  using self = PropertyServer;

public:
  explicit PropertyServer(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh);

private:
  // ROS parameters
  std::string ini_path_;

  // Property tree
  boost::property_tree::ptree pt_;

  // Service servers
  rclcpp::ServiceServer get_bool_ss_;
  rclcpp::ServiceServer get_int_ss_;
  rclcpp::ServiceServer get_double_ss_;
  rclcpp::ServiceServer get_string_ss_;
  rclcpp::ServiceServer set_bool_ss_;
  rclcpp::ServiceServer set_int_ss_;
  rclcpp::ServiceServer set_double_ss_;
  rclcpp::ServiceServer set_string_ss_;
  rclcpp::ServiceServer save_file_ss_;

  inline std::string keyWithSection(const std::string& section, const std::string& key) const;

  template <typename T>
  bool get(const std::string& section, const std::string& key, T& value) const;
  template <typename T>
  void set(const std::string& section, const std::string& key, const T& value);

  template <typename ReqType, typename ResType>
  bool getCb(ReqType& req, ResType& res);
  template <typename ReqType, typename ResType>
  bool setCb(ReqType& req, ResType& res);

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
void PropertyServer::set(const std::string& section, const std::string& key, const T& value)
{
  pt_.put(keyWithSection(section, key), value);
}

template <typename ReqType, typename ResType>
bool PropertyServer::getCb(ReqType& req, ResType& res)
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

template <typename ReqType, typename ResType>
bool PropertyServer::setCb(ReqType& req, ResType& res)
{
  set(req.section, req.key, req.value);

  res.success = true;
  res.message = "";

  return true;
}
}  // namespace ptree
