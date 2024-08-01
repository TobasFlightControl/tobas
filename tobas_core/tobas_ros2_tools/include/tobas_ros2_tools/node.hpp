#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_std_tools/vector.hpp>

namespace ros2
{
class Node : public rclcpp::Node
{
  using super = rclcpp::Node;
  using self = Node;

public:
  explicit Node(const std::string& node_name);

protected:
  template <typename SrvType, typename Obj>
  typename rclcpp::Service<SrvType>::SharedPtr createService(
    const std::string& srv_name,
    void (Obj::*fp)(const typename SrvType::Request::ConstSharedPtr&, const typename SrvType::Response::SharedPtr&),
    Obj* obj);

  bool getBoolParam(const std::string& name);
  long getIntParam(const std::string& name);
  double getDoubleParam(const std::string& name);
  std::string getStringParam(const std::string& name);
  std::vector<bool> getBoolArrayParam(const std::string& name);
  std::vector<uint8_t> getByteArrayParam(const std::string& name);
  std::vector<long> getIntArrayParam(const std::string& name);
  std::vector<double> getDoubleArrayParam(const std::string& name);
  std::vector<std::string> getStringArrayParam(const std::string& name);

  bool getBoolParam(const std::string& name, const bool& _default);
  long getIntParam(const std::string& name, const long& _default);
  double getDoubleParam(const std::string& name, const double& _default);
  std::string getStringParam(const std::string& name, const std::string& _default);
  std::vector<bool> getBoolArrayParam(const std::string& name, const std::vector<bool>& _default);
  std::vector<uint8_t> getByteArrayParam(const std::string& name, const std::vector<uint8_t>& _default);
  std::vector<long> getIntArrayParam(const std::string& name, const std::vector<long>& _default);
  std::vector<double> getDoubleArrayParam(const std::string& name, const std::vector<double>& _default);
  std::vector<std::string> getStringArrayParam(const std::string& name, const std::vector<std::string>& _default);

private:
  template <typename T>
  void declareParam(const std::string& name, const T& _default);
};

template <typename SrvType, typename Obj>
typename rclcpp::Service<SrvType>::SharedPtr Node::createService(
  const std::string& srv_name,
  void (Obj::*fp)(const typename SrvType::Request::ConstSharedPtr&, const typename SrvType::Response::SharedPtr&),
  Obj* obj)
{
  return create_service<SrvType>(srv_name, std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2));
}

template <typename T>
void Node::declareParam(const std::string& name, const T& _default)
{
  try
  {
    declare_parameter<T>(name);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
    RCLCPP_WARN_STREAM(get_logger(), "Parameter \"" << name << "\" is already declared.");
    return;
  }
  catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException&)
  {
    RCLCPP_WARN_STREAM(
      get_logger(), "Parameter \"" << name << "\" is not specified. The default \"" << _default << "\" is set.");
    declare_parameter(name, _default);
  }
}
}  // namespace ros2
