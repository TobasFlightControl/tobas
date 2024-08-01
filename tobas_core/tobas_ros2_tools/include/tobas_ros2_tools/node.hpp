#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
class Node : public rclcpp::Node
{
  using super = rclcpp::Node;
  using self = Node;

public:
  explicit Node(const std::string& node_name);

protected:
  template <typename SrvType, typename T>
  typename rclcpp::Service<SrvType>::SharedPtr createService(
    const std::string& srv_name,
    void (T::*fp)(const typename SrvType::Request::ConstSharedPtr&, const typename SrvType::Response::SharedPtr&),
    T* obj);
};

template <typename SrvType, typename T>
typename rclcpp::Service<SrvType>::SharedPtr Node::createService(
  const std::string& srv_name,
  void (T::*fp)(const typename SrvType::Request::ConstSharedPtr&, const typename SrvType::Response::SharedPtr&),
  T* obj)
{
  return create_service<SrvType>(srv_name, std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2));
}
}  // namespace ros2
