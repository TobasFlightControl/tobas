#pragma once

#include <vector>

#include "./network_interface.hpp"

namespace tobas
{
namespace cyclonedds
{
struct Data
{
  std::vector<NetworkInterface> interfaces;
};
}  // namespace cyclonedds
}  // namespace tobas
