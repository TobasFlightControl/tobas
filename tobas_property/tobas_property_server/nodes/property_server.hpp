#pragma once

#include <boost/property_tree/ini_parser.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_node/node.hpp>

#include <tobas_property_msgs/srv/get_bool.hpp>
#include <tobas_property_msgs/srv/get_int.hpp>
#include <tobas_property_msgs/srv/get_double.hpp>
#include <tobas_property_msgs/srv/get_string.hpp>
#include <tobas_property_msgs/srv/set_bool.hpp>
#include <tobas_property_msgs/srv/set_int.hpp>
#include <tobas_property_msgs/srv/set_double.hpp>
#include <tobas_property_msgs/srv/set_string.hpp>

namespace ptree
{
class PropertyServer : public tobas::BaseNode
{
  using self = PropertyServer;
  using super = tobas::BaseNode;

public:
  explicit PropertyServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ROS parameters
  std::string ini_path_;

  // Property tree
  boost::property_tree::ptree pt_;

  // Service servers
  rclcpp::Service<tobas_property_msgs::srv::GetBool>::SharedPtr get_bool_ss_;
  rclcpp::Service<tobas_property_msgs::srv::GetInt>::SharedPtr get_int_ss_;
  rclcpp::Service<tobas_property_msgs::srv::GetDouble>::SharedPtr get_double_ss_;
  rclcpp::Service<tobas_property_msgs::srv::GetString>::SharedPtr get_string_ss_;
  rclcpp::Service<tobas_property_msgs::srv::SetBool>::SharedPtr set_bool_ss_;
  rclcpp::Service<tobas_property_msgs::srv::SetInt>::SharedPtr set_int_ss_;
  rclcpp::Service<tobas_property_msgs::srv::SetDouble>::SharedPtr set_double_ss_;
  rclcpp::Service<tobas_property_msgs::srv::SetString>::SharedPtr set_string_ss_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_file_ss_;

  inline std::string keyWithSection(const std::string& section, const std::string& key) const;

  template <typename T>
  bool get(const std::string& section, const std::string& key, T& value) const;
  template <typename T>
  void set(const std::string& section, const std::string& key, const T& value);

  template <typename SrvType>
  void getCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);
  template <typename SrvType>
  void setCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);

  void saveFileCb(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
    const std_srvs::srv::Trigger::Response::SharedPtr& res);
};

inline std::string PropertyServer::keyWithSection(const std::string& section, const std::string& key) const
{
  return section + "." + key;
}

template <typename T>
bool PropertyServer::get(const std::string& section, const std::string& key, T& value) const
{
  RCLCPP_DEBUG_STREAM(get_logger(), "Get requested: " << section << ", " << key);

  const auto optional = pt_.get_optional<T>(keyWithSection(section, key));
  if (!optional)
    return false;

  value = optional.get();
  return true;
}

template <typename T>
void PropertyServer::set(const std::string& section, const std::string& key, const T& value)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "Set requested: " << section << ", " << key << ", " << value);

  pt_.put(keyWithSection(section, key), value);
}

template <typename SrvType>
void PropertyServer::getCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res)
{
  if (get(req->section, req->key, res->value))
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
  set(req->section, req->key, req->value);

  res->success = true;
  res->message = "";
}
}  // namespace ptree
