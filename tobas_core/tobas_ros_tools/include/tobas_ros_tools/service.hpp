#pragma once

#include <ros/ros.h>

namespace tobas_ros
{
template <typename SrvType>
class ServiceClientWrapper
{
public:
  typename SrvType::Request req;
  typename SrvType::Response res;

  explicit ServiceClientWrapper(ros::NodeHandle& nh, const std::string& service_name)
  {
    client_ = nh.serviceClient<SrvType>(service_name);
  }

  bool waitForExistence(double timeout = -1)
  {
    return client_.waitForExistence(ros::Duration(timeout));
  }

  bool call()
  {
    return client_.call(req, res);
  }

private:
  ros::ServiceClient client_;
};
}  // namespace tobas_ros
