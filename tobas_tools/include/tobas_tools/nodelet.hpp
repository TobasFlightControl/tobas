#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

namespace tobas
{
template <typename NodeType>
class Nodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    node_.reset(new NodeType(getNodeHandle(), getPrivateNodeHandle(), getName()));
  }

private:
  std::shared_ptr<NodeType> node_;
};
}  // namespace tobas
