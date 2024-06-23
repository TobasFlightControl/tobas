#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_property_tools/property_server.hpp"

namespace ptree
{
class PropertyServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PropertyServer> node_;
};
}  // namespace ptree
